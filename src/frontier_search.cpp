#include <frontier_search.h>

#include <mutex>

#include <geometry_msgs/Point.h>

#include <costmap_tools.h>

#include <limits>

namespace frontier_exploration
{
	using costmap_2d::LETHAL_OBSTACLE;
	using costmap_2d::NO_INFORMATION;
	using costmap_2d::FREE_SPACE;

	FrontierSearch::FrontierSearch(costmap_2d::Costmap2D* costmap,
		double potential_scale, double gain_scale,
		double min_frontier_size,
		bool early_stop_enable, float steer_distance, int rrt_max_iter)
		: costmap_(costmap)
		, potential_scale_(potential_scale)
		, gain_scale_(gain_scale)
		, min_frontier_size_(min_frontier_size)
		, early_stop_enable(early_stop_enable)
		, steer_distance(steer_distance)
		, rrt_max_iter(rrt_max_iter)
	{
	}

	std::vector<Frontier> FrontierSearch::searchFrom(geometry_msgs::Point position)
	{
		std::vector<Frontier> frontier_list;

		// Sanity check that robot is inside costmap bounds before searching
		unsigned int mx, my;
		if (!costmap_->worldToMap(position.x, position.y, mx, my)) {
			ROS_ERROR("Robot out of costmap bounds, cannot search for frontiers");
			return frontier_list;
		}


		// make sure map is consistent and locked for duration of search
		std::lock_guard<costmap_2d::Costmap2D::mutex_t> lock(*(costmap_->getMutex()));

		map_ = costmap_->getCharMap();
		size_x_ = costmap_->getSizeInCellsX();
		size_y_ = costmap_->getSizeInCellsY();

		// initialize flag arrays to keep track of visited and frontier cells
		std::vector<bool> frontier_flag(size_x_ * size_y_, false);

		// initialize breadth first search
		std::queue<unsigned int> bfs;
		std::unordered_map<node, node> parents;

		// find closest clear cell to start search
		unsigned int start, clear, pos = costmap_->getIndex(mx, my);
		if (nearestCell(clear, pos, FREE_SPACE, *costmap_)) {
			start = clear;
			bfs.push(clear);
		}
		else {
			start = pos;
			bfs.push(pos);
			ROS_WARN("Could not find nearby clear cell to start search");
		}
		parents[start] = NODE_NONE;

		bool found_goal = false;

		for (int iter = 0; iter < rrt_max_iter; iter++) {
			auto rand_node = getRandomPoint();
			auto nearest_node = getClosestNode(rand_node, parents);
			auto new_node = steer(rand_node, nearest_node);
			node front;
			auto status = getStatus(nearest_node, new_node, front);

			if (status == BLOCKED)
				continue;

			addToTree(new_node, nearest_node, parents);

			if (status == FRONTIER && isNewFrontierCell(front, frontier_flag)) {
				frontier_flag[front] = true;
				Frontier new_frontier = buildNewFrontier(front, pos, frontier_flag);
				if (new_frontier.size * costmap_->getResolution() >= min_frontier_size_) {
					frontier_list.push_back(new_frontier);
					found_goal = true;
				}
			}

			if (early_stop_enable && found_goal)
				break;
		}

		// set costs of frontiers
		for (auto& frontier : frontier_list) {
			frontier.cost = frontierCost(frontier);
		}
		std::sort(
			frontier_list.begin(), frontier_list.end(),
			[](const Frontier& f1, const Frontier& f2) { return f1.cost < f2.cost; });

		return frontier_list;
	}

	node FrontierSearch::getRandomPoint() {
		node idx;
		do {
			idx = rand() % (size_x_ * size_y_);
		} while (map_[idx] != LETHAL_OBSTACLE);
		return idx;
	}

	node FrontierSearch::getClosestNode(node n, std::unordered_map<node, node>& parents) {
		node closest = NODE_NONE;
		float min_dist_sq = std::numeric_limits<float>::infinity();

		float src_x, src_y;

		src_x = n % size_x_;
		src_y = n / size_x_;

		for (auto pair : parents) {

			node a = pair.first;
			float dx, dy;
			dx = a % size_x_ - src_x;
			dy = a / size_x_ - src_y;

			float dist_sq = dx * dx + dy * dy;

			if (dist_sq < min_dist_sq) {
				closest = a;
				min_dist_sq = dist_sq;
			}
		}

		return closest;
	}

	node FrontierSearch::steer(node rand, node closest) {
		float dx = closest % size_x_ - rand % size_x_;
		float dy = closest / size_x_ - rand / size_x_;

		float d = sqrt(dx * dx + dy * dy);

		float dxn = dx / d;
		float dyn = dy / d;

		unsigned nx = closest % size_x_ + dxn * steer_distance;
		unsigned ny = closest / size_x_ + dyn * steer_distance;

		return ny * size_x_ + nx;
	}

	bool FrontierSearch::isClear(node from, node to) {
		int x1, x2, y1, y2;
		x1 = from % size_x_;
		x2 = to % size_y_;
		y1 = from / size_x_;
		y2 = to / size_x_;

		int dx = std::abs(x2 - x1);
		int dy = std::abs(y2 - y1);
		int sx = (x1 < x2) ? 1 : -1;
		int sy = (y1 < y2) ? 1 : -1;

		int error = dx - dy;
		int currentX = x1;
		int currentY = y1;

		while (currentX != x2 || currentY != y2) {
			// Check if the current cell is an obstacle
			if (map_[currentX + currentY * size_x_] == LETHAL_OBSTACLE) {
				return false;  // Path is blocked by an obstacle
			}

			int error2 = error * 2;

			if (error2 > -dy) {
				error -= dy;
				currentX += sx;
			}

			if (error2 < dx) {
				error += dx;
				currentY += sy;
			}
		}

		return true;  // Path is clear of obstacles
	}

	node FrontierSearch::containsFrontier(node from, node to) {
		int x1, x2, y1, y2;
		x1 = from % size_x_;
		x2 = to % size_y_;
		y1 = from / size_x_;
		y2 = to / size_x_;

		int dx = std::abs(x2 - x1);
		int dy = std::abs(y2 - y1);
		int sx = (x1 < x2) ? 1 : -1;
		int sy = (y1 < y2) ? 1 : -1;

		int error = dx - dy;
		int currentX = x1;
		int currentY = y1;

		while (currentX != x2 || currentY != y2) {
			// Check if the current cell unknown
			if (map_[currentX + currentY * size_x_] == NO_INFORMATION) {
				return currentX + currentY * size_x_;  // Path contains frontier
			}

			int error2 = error * 2;

			if (error2 > -dy) {
				error -= dy;
				currentX += sx;
			}

			if (error2 < dx) {
				error += dx;
				currentY += sy;
			}
		}

		return NODE_NONE;  // Path is through known area
	}



	Frontier FrontierSearch::buildNewFrontier(unsigned int initial_cell,
		unsigned int reference,
		std::vector<bool>& frontier_flag)
	{
		// initialize frontier structure
		Frontier output;
		output.centroid.x = 0;
		output.centroid.y = 0;
		output.size = 1;
		output.min_distance = std::numeric_limits<double>::infinity();

		// record initial contact point for frontier
		unsigned int ix, iy;
		costmap_->indexToCells(initial_cell, ix, iy);
		costmap_->mapToWorld(ix, iy, output.initial.x, output.initial.y);

		// push initial gridcell onto queue
		std::queue<unsigned int> bfs;
		bfs.push(initial_cell);

		// cache reference position in world coords
		unsigned int rx, ry;
		double reference_x, reference_y;
		costmap_->indexToCells(reference, rx, ry);
		costmap_->mapToWorld(rx, ry, reference_x, reference_y);

		while (!bfs.empty()) {
			unsigned int idx = bfs.front();
			bfs.pop();

			// try adding cells in 8-connected neighborhood to frontier
			for (unsigned int nbr : nhood8(idx, *costmap_)) {
				// check if neighbour is a potential frontier cell
				if (isNewFrontierCell(nbr, frontier_flag)) {
					// mark cell as frontier
					frontier_flag[nbr] = true;
					unsigned int mx, my;
					double wx, wy;
					costmap_->indexToCells(nbr, mx, my);
					costmap_->mapToWorld(mx, my, wx, wy);

					geometry_msgs::Point point;
					point.x = wx;
					point.y = wy;
					output.points.push_back(point);

					// update frontier size
					output.size++;

					// update centroid of frontier
					output.centroid.x += wx;
					output.centroid.y += wy;

					// determine frontier's distance from robot, going by closest gridcell
					// to robot
					double distance = sqrt(pow((double(reference_x) - double(wx)), 2.0) +
						pow((double(reference_y) - double(wy)), 2.0));
					if (distance < output.min_distance) {
						output.min_distance = distance;
						output.middle.x = wx;
						output.middle.y = wy;
					}

					// add to queue for breadth first search
					bfs.push(nbr);
				}
			}
		}

		// average out frontier centroid
		output.centroid.x /= output.size;
		output.centroid.y /= output.size;
		return output;
	}

	bool FrontierSearch::isNewFrontierCell(unsigned int idx,
		const std::vector<bool>& frontier_flag)
	{
		// check that cell is unknown and not already marked as frontier
		if (map_[idx] != NO_INFORMATION || frontier_flag[idx]) {
			return false;
		}

		// frontier cells should have at least one cell in 4-connected neighbourhood
		// that is free
		for (unsigned int nbr : nhood4(idx, *costmap_)) {
			if (map_[nbr] == FREE_SPACE) {
				return true;
			}
		}

		return false;
	}

	double FrontierSearch::frontierCost(const Frontier& frontier)
	{
		return (potential_scale_ * frontier.min_distance *
			costmap_->getResolution()) -
			(gain_scale_ * frontier.size * costmap_->getResolution());
	}
}
