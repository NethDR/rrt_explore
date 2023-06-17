#ifndef FRONTIER_SEARCH_H_
#define FRONTIER_SEARCH_H_

#include <costmap_2d/cost_values.h>
#include <costmap_2d/costmap_2d.h>
#include <unordered_map>
#include <ros/ros.h>
#include <ros/publisher.h>


namespace frontier_exploration
{
	/**
	 * @brief Represents a frontier
	 *
	 */
	struct Frontier {
		std::uint32_t size;
		double min_distance;
		double cost;
		geometry_msgs::Point initial;
		geometry_msgs::Point centroid;
		geometry_msgs::Point middle;
		std::vector<geometry_msgs::Point> points;
	};

	typedef unsigned int node;

	/**
	 * @brief Thread-safe implementation of a frontier-search task for an input
	 * costmap.
	 */
	class FrontierSearch
	{
	public:
		FrontierSearch()
		{
		}

		/**
		 * @brief Constructor for search task
		 * @param costmap Reference to costmap data to search.
		 */
		FrontierSearch(costmap_2d::Costmap2D* costmap, double potential_scale,
			double gain_scale, double min_frontier_size,
			bool early_stop_enable, float steer_distance, int rrt_max_iter, ros::Publisher debug_publisher);

		/**
		 * @brief Runs search implementation, outward from the start position
		 * @param position Initial position to search from
		 * @return List of frontiers, if any
		 */
		std::vector<Frontier> searchFrom(geometry_msgs::Point position);

	protected:
		/**
		 * @brief Starting from an initial cell, build a frontier from valid adjacent
		 * cells
		 * @param initial_cell Index of cell to start frontier building
		 * @param reference Reference index to calculate position from
		 * @param frontier_flag Flag vector indicating which cells are already marked
		 * as frontiers
		 * @return new frontier
		 */
		Frontier buildNewFrontier(unsigned int initial_cell, unsigned int reference,
			std::vector<bool>& frontier_flag);

		/**
		 * @brief isNewFrontierCell Evaluate if candidate cell is a valid candidate
		 * for a new frontier.
		 * @param idx Index of candidate cell
		 * @param frontier_flag Flag vector indicating which cells are already marked
		 * as frontiers
		 * @return true if the cell is frontier cell
		 */
		bool isNewFrontierCell(unsigned int idx,
			const std::vector<bool>& frontier_flag);

		/**
		 * @brief computes frontier cost
		 * @details cost function is defined by potential_scale and gain_scale
		 *
		 * @param frontier frontier for which compute the cost
		 * @return cost of the frontier
		 */
		double frontierCost(const Frontier& frontier);

	private:
		costmap_2d::Costmap2D* costmap_;
		unsigned char* map_;
		unsigned int size_x_, size_y_;
		double potential_scale_, gain_scale_;
		double min_frontier_size_;


		bool early_stop_enable;
		float steer_distance;
		int rrt_max_iter;
		ros::Publisher debug_publisher;


		node getRandomPoint();
		node getClosestNode(node n, std::unordered_map<node,node>& parents);
		node steer(node rand, node closest);

		enum path_status {
			CLEAR,
			FRONTIER,
			BLOCKED
		};

		path_status getStatus(node from, node to, node& frontier);

		bool isClear(node from, node to);
		node containsFrontier(node from, node to);
		inline void addToTree(node n, node parent, std::unordered_map<node,node>& parents) {parents[n] = parent;}
		inline bool isUnexplored(node n) { return map_[n] == costmap_2d::NO_INFORMATION;}


		static const node NODE_NONE = -1;

	};
}
#endif
