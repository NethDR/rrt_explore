#include <frontier_search.h>
#include <mutex>
#include <geometry_msgs/Point.h>
#include <costmap_tools.h>
#include <limits>
#include <fstream>
#include <random>
#include <visualization_msgs/Marker.h>
#include <algorithm>
#include <explore.h>

namespace frontier_exploration
{
	using costmap_2d::LETHAL_OBSTACLE;
	using costmap_2d::NO_INFORMATION;
	using costmap_2d::FREE_SPACE;

	void initMarker(visualization_msgs::Marker &m,int32_t type, float r, float g, float b, float a, double px, double py, double pz, double sx, double sy, double sz, std::string ns) {
		m.type = type;
		m.color.r = r;
		m.color.b = g;
		m.color.g = b;
		m.color.a = a;
		m.scale.x = sx;
		m.scale.y = sy;
		m.scale.z = sz;
		m.action = visualization_msgs::Marker::ADD;
		m.pose.position.x = px;
		m.pose.position.y = py;
		m.pose.position.z = pz;
		m.pose.orientation.x = m.pose.orientation.y = m.pose.orientation.z = 0;
		m.pose.orientation.w = 1;
		m.lifetime = ros::Duration(0);
		m.header.frame_id = explore::ref_frame;
		m.header.stamp = ros::Time();
		m.ns = ns;
	}

	FrontierSearch::FrontierSearch(costmap_2d::Costmap2D* costmap,
		double potential_scale, double gain_scale,
		double min_frontier_size,
		bool early_stop_enable, float steer_distance, int rrt_max_iter,
		ros::Publisher debug_publisher, bool use_rrt_star, bool hinting)
		: costmap_(costmap)
		, potential_scale_(potential_scale)
		, gain_scale_(gain_scale)
		, min_frontier_size_(min_frontier_size)
		, early_stop_enable(early_stop_enable)
		, steer_distance(steer_distance)
		, rrt_max_iter(rrt_max_iter)
		, debug_publisher(debug_publisher)
	{
		initMarker(generated_points,
			visualization_msgs::Marker::CUBE_LIST,
			1,1,0,1,
			0,0,0,
			0.1, 0.1, 0.1,
			"generated");
		initMarker(attempted_points,
			visualization_msgs::Marker::CUBE_LIST,
			1,1,0.5,1,
			0,0,0,
			0.1, 0.1, 0.1,
			"attempted");
		initMarker(tree_marker,
			visualization_msgs::Marker::LINE_LIST,
			0,0,1,1,
			0,0,0,
			0.1,0,0,
			"rrt"
		);
	}

	void FrontierSearch::publishTree(const std::unordered_map<node, node> &parents) {
		tree_marker.points.clear();
		for (auto p : parents) {
			if (p.second == NODE_NONE)
				continue;
			unsigned int xto, xfrom, yto, yfrom;
			geometry_msgs::Point to, from;
			costmap_->indexToCells(p.first, xto, yto);
			costmap_->indexToCells(p.second, xfrom, yfrom);
			costmap_->mapToWorld(xto, yto, to.x, to.y);
			costmap_->mapToWorld(xfrom, yfrom, from.x, from.y);
			tree_marker.points.emplace_back(from);
			tree_marker.points.emplace_back(to);
		}
		debug_publisher.publish(tree_marker);
		debug_publisher.publish(generated_points);
		debug_publisher.publish(attempted_points);
	}

	double FrontierSearch::distance(node a, node b) {
		unsigned int ax, ay, bx, by;
		costmap_->indexToCells(a, ax, ay);
		costmap_->indexToCells(b, bx, by);
		return sqrt(ax * ax + ay * ay);
	}

	node FrontierSearch::chooseParent(node x_new, node x_near,
		std::vector<node> nearest,
		std::unordered_map<node, double> costs, node &frontier, bool &is_front) {
			auto cost_min = costs[x_near] + distance(x_new, x_near);
			auto idx_min = x_near;

			for (auto candidate: nearest) {
				double cost = costs[candidate] + distance(x_new, candidate);
				node front;
				auto status = getStatus(candidate, x_new, front);

				if (status != BLOCKED && cost < cost_min) {
					cost_min = cost;
					idx_min = candidate;
					frontier = front;
					is_front = status == FRONTIER;
				}
			}
			return idx_min;
	}

	std::vector<node> FrontierSearch::near(node x, std::unordered_map<node, node> parents)
	{
		std::vector<node> v;

		auto card_v = parents.size();

		auto r = std::max(k_rrt * sqrt(std::log(card_v)/card_v), (double)steer_distance);

		for (auto p : parents) {
			auto a = p.first;
			if (distance(a, x) < r) {
				v.push_back(a);
			}
		}
		return v;
	}

	void propagate(node x, double delta, std::unordered_map<node, node> &parents, std::unordered_map<node, double> &costs) {
		for (auto p : parents) {
			if (p.second != x)
				continue;

			costs[p.first] -= delta;
			propagate(p.first, delta, parents, costs);
		}
	}

	void FrontierSearch::rewire(node x, std::vector<node> nearby_nodes, std::unordered_map<node, node> &parents, std::unordered_map<node, double> &costs) {
		double cx = costs[x];
		for (node idx : nearby_nodes) {
			node f;
			auto status = getStatus(x, idx, f);
			if (status != CLEAR)
				continue;
			double cost = cx + distance(x, idx);
			if (cost < costs[idx]) {
				double delta = costs[idx] - cost;
				parents[idx] = x;
				costs[idx] = cost;
				propagate(idx, delta, parents, costs);
			}
		}
	}

	node get_middle_step(node n, std::unordered_map<node, node> parents) {
		node tgt = n;

		while (n != NODE_NONE) {
			n = parents[n];
			if (n != NODE_NONE) {
				n = parents[n];
				tgt = parents[tgt];
			}
		}

		return tgt;
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
		std::unordered_map<node, node> parents;
		std::unordered_map<node, double> costs;

		// find closest clear cell to start search
		unsigned int start, clear, pos = costmap_->getIndex(mx, my);



		if (nearestCell(clear, pos, FREE_SPACE, *costmap_)) {
			start = clear;
		}
		else {
			start = pos;
			ROS_WARN("Could not find nearby clear cell to start search");
		}

		parents[start] = NODE_NONE;
		if (use_rrt_star)
			costs[start] = 0;

		bool found_goal = false;

		for (int iter = 0; iter < rrt_max_iter; iter++) {

			ROS_DEBUG_COND(iter % 1000 == 0, "iter %d", iter);

			if (iter % 1000 == 0) {
				publishTree(parents);
			}


			auto rand_node = getRandomPoint();
			auto nearest_node = getClosestNode(rand_node, parents);
			if (nearest_node == NODE_NONE)
				continue;
			auto new_node = steer(rand_node, nearest_node);
			node end;
			auto status = getStatus(nearest_node, new_node, end);

			if (status == BLOCKED || end == nearest_node)
				continue;

			bool is_front = status == FRONTIER;

			node parent;

			if (use_rrt_star) {
				auto nearest_nodes = near(new_node, parents);

				auto parent = chooseParent(new_node, nearest_node, nearest_nodes, costs, end, is_front);

				parents[end] = parent;
				costs[end] = costs[parent] + distance(end, parent);

				rewire(new_node, nearest_nodes, parents, costs);
			} else {
				parents[end] = nearest_node;
			}

			if (is_front && isNewFrontierCell(end, frontier_flag)) {
				frontier_flag[end] = true;
				Frontier new_frontier = buildNewFrontier(end, pos, frontier_flag);
				if (new_frontier.size * costmap_->getResolution() >= min_frontier_size_) {
					new_frontier.target = end;
					frontier_list.push_back(new_frontier);
					found_goal = true;
				}
			}

			if (early_stop_enable && found_goal)
				break;
		}
		publishTree(parents);
		generated_points.points.clear();
		attempted_points.points.clear();

		// set costs of frontiers
		for (auto& frontier : frontier_list) {
			if (use_rrt_star)
				frontier.distance = frontier.target;
			frontier.cost = frontierCost(frontier);
			if (hinting)
				frontier.target = get_middle_step(frontier.target, parents);
		}
		std::sort(
			frontier_list.begin(), frontier_list.end(),
			[](const Frontier& f1, const Frontier& f2) { return f1.cost < f2.cost; });


		return frontier_list;
	}


	node FrontierSearch::getRandomPoint() {
		node idx;
		unsigned int x, y;
		// std::default_random_engine generator();
		std::mt19937_64 rng(std::chrono::steady_clock::now().time_since_epoch().count());
		std::uniform_int_distribution<int> distributionX(0,size_x_);
		std::uniform_int_distribution<int> distributionY(0,size_y_);
		int n = 0;
		// do {
			x = distributionX(rng);
			y = distributionY(rng);
			idx = costmap_->getIndex(x, y);
			n++;
		// } while (map_[idx] == LETHAL_OBSTACLE);

		double rx, ry;
		geometry_msgs::Point np;

		costmap_->mapToWorld(x,y,rx,ry);

		// ROS_DEBUG("%u %u %u %d", x, y, idx, n);
		np.x = rx;
		np.y = ry;
		generated_points.points.emplace_back(np);

		return idx;
	}

	node FrontierSearch::getClosestNode(node n, std::unordered_map<node, node>& parents) {
		node closest = NODE_NONE;
		float min_dist_sq = std::numeric_limits<float>::infinity();

		unsigned int src_x, src_y;
		costmap_->indexToCells(n, src_x, src_y);

		for (auto pair : parents) {

			node a = pair.first;
			if (a == n)
				return NODE_NONE;
			if (map_[a] == NO_INFORMATION || map_[a] == LETHAL_OBSTACLE)
				continue;
			unsigned int candidate_x, candidate_y;
			costmap_->indexToCells(a, candidate_x, candidate_y);

			unsigned int dx, dy;
			dx = candidate_x - src_x;
			dy = candidate_y - src_y;

			float dist_sq = dx * dx + dy * dy;

			if (dist_sq < min_dist_sq) {
				closest = a;
				min_dist_sq = dist_sq;
				// ROS_DEBUG("%f", dist_sq);
			}
		}

		return closest;
	}

	node FrontierSearch::steer(node rand, node closest) {
		unsigned int rx, ry, cx, cy;
		costmap_->indexToCells(rand, rx, ry);
		costmap_->indexToCells(closest, cx, cy);

		int dx = - cx + rx;
		int dy = - cy + ry;

		float d = sqrt(dx * dx + dy * dy);

		unsigned int nx, ny;

		if (d <= steer_distance) {
			nx = rx;
			ny = ry;
		} else {
			float dxn = dx / d;
			float dyn = dy / d;

			nx = cx + dxn * steer_distance;
			ny = cy + dyn * steer_distance;
		}

		geometry_msgs::Point np;

		costmap_->mapToWorld(nx, ny, np.x, np.y);

		attempted_points.points.emplace_back(np);

		return costmap_->getIndex(nx,ny);

	}

	FrontierSearch::path_status FrontierSearch::getStatus(node from, node to, node& end) {
		unsigned int x1, x2, y1, y2;
		costmap_->indexToCells(from, x1, y1);
		costmap_->indexToCells(to, x2, y2);

		int dx = std::abs((int)x2 - (int)x1);
		int dy = -std::abs((int)y2 - (int)y1);
		int sx = (x1 < x2) ? 1 : -1;
		int sy = (y1 < y2) ? 1 : -1;

		int error = dx + dy;
		int currentX = x1;
		int currentY = y1;

		while (currentX != x2 || currentY != y2) {
			if (map_[costmap_->getIndex(currentX, currentY)] == LETHAL_OBSTACLE) {
				return BLOCKED;
			} else if(map_[costmap_->getIndex(currentX, currentY)] == NO_INFORMATION) {
				end = costmap_->getIndex(currentX, currentY);
				return FRONTIER;
			}

			int error2 = error * 2;

			if (error2 >= dy) {
				error += dy;
				currentX += sx;
			}

			if (error2 <= dx) {
				error += dx;
				currentY += sy;
			}
			end = costmap_->getIndex(currentX, currentY);
		}
		end = to;

		return CLEAR;
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
		output.distance = std::numeric_limits<double>::infinity();

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
					if (distance < output.distance) {
						output.distance = distance;
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
		return (potential_scale_ * frontier.distance *
			costmap_->getResolution()) -
			(gain_scale_ * frontier.size * costmap_->getResolution());
	}

}
