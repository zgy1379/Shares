#include "astar_planner.h"
#include <pluginlib/class_list_macros.h>
#include <cmath>
// register this planner as a BaseGlobalPlanner plugin
PLUGINLIB_EXPORT_CLASS(astar_planner::AstarPlanner, nav_core::BaseGlobalPlanner)

namespace astar_planner
{
    AstarPlanner::AstarPlanner() {}

    AstarPlanner::AstarPlanner(std::string name, costmap_2d::Costmap2DROS *costmap_ros)
    {
        initialize(name, costmap_ros);
    }

    void AstarPlanner::initialize(std::string name, costmap_2d::Costmap2DROS *costmap_ros)
    {
    if(!initialized_){
   ros::NodeHandle private_nh("move_base/");
            cost_ = costmap_ros->getCostmap()->getCharMap();
            costmap_ros_ = costmap_ros;
            costmap_ = costmap_ros_->getCostmap();
            width = costmap_->getSizeInCellsX();
            height = costmap_->getSizeInCellsY();
            map_size = width * height;
            spline_smoother_ = new PathSplineSmoother();

            frame_id_ = costmap_ros->getGlobalFrameID();

          plan_pub_ = private_nh.advertise<nav_msgs::Path>("Astar_plan", 1);

           initialized_ = true;

            private_nh.param("heuristic_name", heuristic_name_, std::string("Manhattan"));
            private_nh.getParam("heuristic_factor", heuristic_factor_);
            private_nh.getParam("inaccessible_cost", inaccessible_cost_);
            private_nh.getParam("goal_search_tolerance", goal_search_tolerance_);
            private_nh.getParam("cost_weight", cost_weight);
            private_nh.getParam("ymovecost", ymovecost);
            private_nh.getParam("xmovecost", xmovecost);
            private_nh.getParam("yymovecost", yymovecost);
            private_nh.getParam("xxmovecost", xxmovecost);
            goal_search_tolerance_ = goal_search_tolerance_/costmap_ros_->getCostmap()->getResolution();

           private_nh.getParam("smooth_", smooth);

    }
         else
         {
             ROS_WARN("already initialized!");
         }
      
    }

    bool AstarPlanner::makePlan(const geometry_msgs::PoseStamped &start,
                                const geometry_msgs::PoseStamped &goal, std::vector<geometry_msgs::PoseStamped> &plan)
    {

        if (!initialized_)
        {
            ROS_ERROR("The planner has not been initialized, please call initialize() to use the planner");
            return false;
        }
unsigned int s_x, s_y,g_x, g_y;
//检测起点和终点
if (!costmap_->worldToMap(start.pose.position.x,
                                              start.pose.position.y,
                                              s_x,
                                              s_y)) {
    ROS_WARN("Failed to transform start pose from map frame to costmap frame");
    return false;
  }
  if (!costmap_->worldToMap(goal.pose.position.x,
                                             goal.pose.position.y,
                                             g_x,
                                             g_y)) {
    ROS_WARN("Failed to transform goal pose from map frame to costmap frame");
    return false;
  }


        cost_ = costmap_ros_->getCostmap()->getCharMap();
     
        double wx = start.pose.position.x;
        double wy = start.pose.position.y;
        unsigned int start_x, start_y;
        costmap_->worldToMap(wx, wy, start_x, start_y);
        int start_index = costmap_->getIndex(start_x, start_y);

        wx = goal.pose.position.x;
        wy = goal.pose.position.y;

        unsigned int goal_x, goal_y;
        costmap_->worldToMap(wx, wy, goal_x, goal_y);
        int goal_index = costmap_->getIndex(goal_x, goal_y);

        vector<float> gCosts(map_size, infinity);
        vector<int> cameFrom(map_size, -1);

        multiset<Node> priority_costs;
        gCosts[start_index] = 0;

        Node currentNode;
        currentNode.index = start_index;
        currentNode.cost = gCosts[start_index] + 0;
        priority_costs.insert(currentNode);

        plan.clear();

        while (!priority_costs.empty())
        {
            // Take the element from the top
            currentNode = *priority_costs.begin();
            // Delete the element from the top
            priority_costs.erase(priority_costs.begin());
            if (currentNode.index == goal_index)
            {
                break;
            }
            // Get neighbors
            vector<int> neighborIndexes = get_neighbors(currentNode.index);

            for (int i = 0; i < neighborIndexes.size(); i++)
            {
                //添加约束

                //该点为致命代价值，不可到达点
                 if( cost_[neighborIndexes[i]]>=inaccessible_cost_)
                   continue;
                //该临接节点不在地图范围内，则跳过该节点，该点为不可到达点
      if (neighborIndexes[i] < 0 ||
          neighborIndexes[i] >= height* width) {
        continue;
      }

                if (cameFrom[neighborIndexes[i]] == -1)
                {
                    gCosts[neighborIndexes[i]] = gCosts[currentNode.index] + getMoveCost(currentNode.index, neighborIndexes[i]) + cost_weight*cost_[neighborIndexes[i]];
                    Node nextNode;
                    nextNode.index = neighborIndexes[i];
                    //unsigned int x, y;
                    //costmap_->indexToCells(neighborIndexes[i], x, y);
                    // nextNode.cost = gCosts[neighborIndexes[i]]+ getHeuristic(neighborIndexes[i], goal_index)*    (1+(((int)abs(atan(x*6+y*4)))% 7/map_size));    //A* Algorithm
                    nextNode.cost = gCosts[neighborIndexes[i]] + getHeuristic(neighborIndexes[i], goal_index) * (1 + move_cost / 10000); // A* Algorithm
                    cameFrom[neighborIndexes[i]] = currentNode.index;
                    priority_costs.insert(nextNode);
                }

                 if (cameFrom[neighborIndexes[i]] != -1)
                {

                    if(gCosts[neighborIndexes[i]] >gCosts[currentNode.index] + getMoveCost(currentNode.index, neighborIndexes[i]) + cost_weight*cost_[neighborIndexes[i]])
                    {
                         gCosts[neighborIndexes[i]] = gCosts[currentNode.index] + getMoveCost(currentNode.index, neighborIndexes[i]) + cost_weight*cost_[neighborIndexes[i]];
                        cameFrom[neighborIndexes[i]] = currentNode.index;
                    }
                    
                }

           
            }
        }

        if (cameFrom[goal_index] == -1)
        {

            ROS_WARN("Goal not reachable, failed making a global path.");
            return false;
        }

        if (start_index == goal_index)
            return false;
        // Finding the best path
        vector<int> bestPath;
        currentNode.index = goal_index;
        while (currentNode.index != start_index)
        {
            bestPath.push_back(cameFrom[currentNode.index]);
            currentNode.index = cameFrom[currentNode.index];
        }
        reverse(bestPath.begin(), bestPath.end());


        if (smooth == false)
        {

            ros::Time plan_time = ros::Time::now();
            for (int i = 0; i < bestPath.size(); i++)
            {
                unsigned int tmp1, tmp2;
                costmap_->indexToCells(bestPath[i], tmp1, tmp2);
                double x, y;
                costmap_->mapToWorld(tmp1, tmp2, x, y);

                geometry_msgs::PoseStamped pose;
                pose.header.stamp = plan_time;
                pose.header.frame_id = costmap_ros_->getGlobalFrameID();
                pose.pose.position.x = x;
                pose.pose.position.y = y;
                pose.pose.position.z = 0.0;

                pose.pose.orientation.x = 0.0;
                pose.pose.orientation.y = 0.0;
                pose.pose.orientation.z = 0.0;
                pose.pose.orientation.w = 1.0;

                plan.push_back(pose);
            }
        }

        else if (smooth == true)
        {
            vector<RealPoint> path_smoothed = SmoothPlan(bestPath);
            int size_path_smoothed = (int)(path_smoothed.size());
            // ROS_WARN("Size of the smoothed path %d", size_path_smoothed);
            for (int j = 0; j < size_path_smoothed; j++)
            {

                geometry_msgs::PoseStamped next_node;
                next_node.header.stamp = ros::Time::now();
                next_node.header.frame_id = costmap_ros_->getGlobalFrameID();

                next_node.pose.position.x = path_smoothed[j].x;
                next_node.pose.position.y = path_smoothed[j].y;

                next_node.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(0, 0, path_smoothed[j].theta);

                plan.push_back(next_node);
            }
        }

        plan.push_back(goal);
        publishPlan(plan);
        
        return true;
    }

    double AstarPlanner::getMoveCost(int firstIndex, int secondIndex)
    {
        unsigned int tmp1, tmp2;
        costmap_->indexToCells(firstIndex, tmp1, tmp2);
        int firstXCord = tmp1, firstYCord = tmp2;
        costmap_->indexToCells(secondIndex, tmp1, tmp2);
        int secondXCord = tmp1, secondYCord = tmp2;

        int difference = abs(firstXCord - secondXCord) + abs(firstYCord - secondYCord);
       
        if (difference != 1 && difference != 2)
        {
            ROS_ERROR("Astar global planner: Error in getMoveCost - difference not valid");
            return 10000;
        }
        if (difference == 1)
        {
            if(firstXCord==secondXCord)
             move_cost = ymovecost;
             else
              move_cost = xmovecost;

        }
           
        else
        {
            if(firstYCord>secondYCord)
             move_cost = yymovecost;
             else
              move_cost = xxmovecost;
        }
            

        return move_cost;
    }

    double AstarPlanner::getHeuristic(int cell_index, int goal_index)
    {
        
        double distance;
        if (heuristic_name_ == "Manhattan")
        {
            // //ROS_WARN("Manhattan");
           distance = heuristic_factor_ * 10 * (fabs(cell_index / width - goal_index / width) + fabs(cell_index % width - goal_index % width));
             
        }

        else if (heuristic_name_ == "Chebyshev")
        {
            int a = fabs(cell_index / width - goal_index / width);
            int b = fabs(cell_index % width - goal_index % width);
            int max = a > b ? a : b;
            distance = heuristic_factor_ * 10 * max;
            // //ROS_WARN("Chebyshev Heuristic");
        }
        else if (heuristic_name_ == "Dialog")
        {
            int dx = fabs(cell_index / width - goal_index / width);
            int dy = fabs(cell_index % width - goal_index % width);
            distance = dx + dy + (1.414 - 2) * std::min(dx, dy);
            // //ROS_WARN("Dialog Heuristic");
        }
        else if (heuristic_name_ == "Euler")
        {
            int dx = fabs(cell_index / width - goal_index / width);
            int dy = fabs(cell_index % width - goal_index % width);
            distance = std::sqrt(std::pow(dx, 2) + std::pow(dy, 2));
            // //ROS_WARN("Euler Heuristic");
        }

        return distance;
    }

    bool AstarPlanner::isInBounds(int x, int y)
    {
        if (x < 0 || y < 0 || x >= height || y >= width)
            return false;
        return true;
    }

    vector<int> AstarPlanner::get_neighbors(int current_cell)
    {
        vector<int> neighborIndexes;
       /* for (int i = -1; i <= 1; i++)
        {
            for (int j = -1; j <= 1; j++)
            {
                unsigned tmp1, tmp2;
                costmap_->indexToCells(current_cell, tmp1, tmp2);
                int nextX = tmp1 + i;
                int nextY = tmp2 + j;
                int nextIndex = costmap_->getIndex(nextX, nextY);
                if (!(i == 0 && j == 0) && isInBounds(nextX, nextY) && OGM[nextIndex])
                {
                    neighborIndexes.push_back(nextIndex);
                }
            }
        }*/
         if(current_cell - width >= 0){
      neighborIndexes.push_back(current_cell - width);       //up
    }
    if(current_cell - width - 1 >= 0 && (current_cell - width - 1 + 1) % width!= 0){
      neighborIndexes.push_back(current_cell - width - 1); //left_up
    }
    if(current_cell - 1 >= 0 && (current_cell - 1 + 1) % width!= 0){
      neighborIndexes.push_back(current_cell - 1);        //left
    }
    if(current_cell + width - 1 < width* height
        && (current_cell + width - 1 + 1) % width!= 0){
      neighborIndexes.push_back(current_cell + width - 1); //left_down
    }
    if(current_cell + width < width* height){
      neighborIndexes.push_back(current_cell + width);     //down
    }
    if(current_cell + width + 1 < width* height
        && (current_cell + width + 1 ) % width!= 0){
      neighborIndexes.push_back(current_cell + width + 1); //right_down
    }
    if(current_cell  + 1 < width* height
        && (current_cell  + 1 ) % width!= 0) {
      neighborIndexes.push_back(current_cell + 1);                   //right
    }
    if(current_cell - width + 1 >= 0
        && (current_cell - width + 1 ) % width!= 0) {
      neighborIndexes.push_back(current_cell - width + 1); //right_up
    }
        return neighborIndexes;
    }

    void AstarPlanner::publishPlan(const std::vector<geometry_msgs::PoseStamped> &path)
    {
        if (!initialized_)
        {
            ROS_ERROR(
                "This planner has not been initialized yet, but it is being used, please call initialize() before use");
            return;
        }

        // create a message for the plan
        nav_msgs::Path gui_path;
        gui_path.poses.resize(path.size());

        gui_path.header.frame_id = frame_id_;
        gui_path.header.stamp = ros::Time::now();

        // Extract the plan in world co-ordinates, we assume the path is all in the same frame
        for (unsigned int i = 0; i < path.size(); i++)
        {
            gui_path.poses[i] = path[i];
        }

        plan_pub_.publish(gui_path);
    }

    /// ==================================================================================
    /// SmoothPlan(list<state> path)
    /// smoothing using splines
    /// ==================================================================================
    vector<RealPoint> AstarPlanner::SmoothPlan(vector<int> path)
    {
        // ROS_WARN("SmoothPlan / getting costmap infos");
        double costmap_resolution = costmap_->getResolution();
        double origin_costmap_x = costmap_->getOriginX();
        double origin_costmap_y = costmap_->getOriginY();

        /// copying the path in a different format
        int initial_path_size = (int)path.size();
        vector<RealPoint> input_path;
        input_path.clear();
        if (initial_path_size == 0)
        {
            ROS_ERROR("Path not valid for smoothing, returning");
            return input_path;
        }

        // ROS_WARN("SmoothPlan, filling the points");

        // std::vector<int>::const_iterator iterator;
        // int iterator;
        double t, old_x, old_y, old_th, dt;
        dt = 0.5;
        int cnt = 0;
        for (int i = 0; i < path.size(); ++i)
        {

            int node = path[i];

            /// giving as input path the cartesian path
            unsigned int nodex, nodey;
            costmap_->indexToCells(node, nodex, nodey);
            double x = (nodex + 0.5) * costmap_resolution + origin_costmap_x;
            double y = (nodey + 0.5) * costmap_resolution + origin_costmap_y;

            if (cnt > 0)
            {

                t = dt;

                while (t < 1)
                {
                    RealPoint p_new;
                    p_new.x = (x - old_x) * t + old_x;
                    p_new.y = (y - old_y) * t + old_y;
                    p_new.theta = 0;
                    input_path.push_back(p_new);
                    // ROS_WARN("Adding point %f %f ", p_new.x, p_new.y);
                    t = t + dt;
                }
            }
            else
            {

                RealPoint p;
                p.x = x;
                p.y = y;
                p.theta = 0;
                input_path.push_back(p);
                // ROS_WARN("Adding Initial point %f %f of a segment ", x, y);
            }

            old_x = x;
            old_y = y;
            old_th = 0;
            cnt++;
        }

        // do not smooth if the path has not enough points
        if (initial_path_size < 30)
        {
            // ROS_WARN("Returning path, without smoothing it");
            return input_path;
        }

        ROS_DEBUG("SmoothPlan, Providing the path to the smoother");
        spline_smoother_->readPathFromStruct(input_path);
        // ROS_WARN("SmoothPlan, Filtering path");
        spline_smoother_->filterPath(1);
        // ROS_WARN("SmoothPlan, Smoothing path");
        spline_smoother_->smoothWhileDistanceLessThan(0.05, 1.01);
        // ROS_WARN("SmoothPlan, getting path");
        vector<RealPoint> smooth_path = spline_smoother_->getSmoothPath();

        return smooth_path;
    }

};
// Required for multiset sorting
bool operator<(const Node &x, const Node &y)
{
    return x.cost < y.cost;
}
