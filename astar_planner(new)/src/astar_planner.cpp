#include "astar_planner.h"
#include "pluginlib/class_list_macros.h"
#include "math.h"
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
        if (!initialized_)
        {
            cost_ = costmap_ros->getCostmap()->getCharMap();
            // 初始化动态代价地图
            costmap_ros_ = costmap_ros;
            // 获取静态代价地图
            costmap_ = costmap_ros_->getCostmap();
            // 获取全局地图坐标系的ID
            frame_id_ = costmap_ros->getGlobalFrameID();
            // 给节点加命名空间，变为私有节点
            ros::NodeHandle private_nh("move_base/");
            // 用于发布全局路径
            plan_pub_ = private_nh.advertise<nav_msgs::Path>("Astar_plan", 1);
            // 地图宽度
            width = costmap_->getSizeInCellsX();
            // 地图高度
            height = costmap_->getSizeInCellsY();
            // 地图大小
            map_size = width * height;

            spline_smoother_ = new PathSplineSmoother();

            initialized_ = true;

            private_nh.param("heuristic_name", heuristic_name_, std::string("Manhattan"));
            private_nh.getParam("heuristic_factor", heuristic_factor_);
            private_nh.getParam("inaccessible_cost", inaccessible_cost_);
            private_nh.getParam("goal_search_tolerance", goal_search_tolerance_);
            private_nh.getParam("cost_weight", cost_weight);

            private_nh.getParam("smooth_", smooth);

            ROS_INFO("Astar全局规划器初始化成功!");
        }
        else
        {
            ROS_WARN("Astar全局规划器初始化失败...");
        }
    }

    bool AstarPlanner::makePlan(const geometry_msgs::PoseStamped &start,
                                const geometry_msgs::PoseStamped &goal,
                                std::vector<geometry_msgs::PoseStamped> &plan)
    {
        // 判断是否初始化
        if (!initialized_)
        {
            ROS_ERROR("Astar全局规划器尚未初始化,请先初始化全局规划器!");
            return false;
        }

        // 世界坐标(转移用)
        double wx, wy;
        // 栅格地图start/goal
        unsigned int start_x, start_y, goal_x, goal_y;
        int start_index, goal_index;
        // 从start移动到栅格地图某点的代价
        vector<float> gCosts(map_size, infinity);
        // 栅格地图某点的母节点
        vector<int> cameFrom(map_size, -1);
        // 待扩散列表
        multiset<Node> list_to_expand;
        // 当前节点
        Node currentNode;
        // 将时间设为当前时间
        ros::Time plan_time = ros::Time::now();
        // 可视化点的属性
        geometry_msgs::PoseStamped pose;

        // 初始化start/goal/gCost/cameFrom/plan/plan_time/pose
        {
            ROS_INFO("Got a start: %.2f, %.2f, and a goal: %.2f, %.2f", start.pose.position.x, start.pose.position.y,
                     goal.pose.position.x, goal.pose.position.y);
            // 清空规划
            plan.clear();
            // 转入start坐标
            if (!costmap_->worldToMap(start.pose.position.x, start.pose.position.y, start_x, start_y))
            {
                ROS_WARN("无法将起点坐标从世界地图转入栅格地图!");
                return false;
            }
            else
            {
                start_index = costmap_->getIndex(start_x, start_y);
            }
            // 转入goal坐标
            if (!costmap_->worldToMap(goal.pose.position.x, goal.pose.position.y, goal_x, goal_y))
            {
                ROS_WARN("无法将终点坐标从世界地图转入栅格地图!");
                return false;
            }
            else
            {
                goal_index = costmap_->getIndex(goal_x, goal_y);
            }
            // start点代价为0
            gCosts[start_index] = 0;
            // 当前节点为start点
            currentNode.index = start_index;
            currentNode.cost = gCosts[start_index] + 0;
            list_to_expand.insert(currentNode);
            // 初始化可视化点的属性
            pose.pose.orientation.x = 0.0;
            pose.pose.orientation.y = 0.0;
            pose.pose.orientation.z = 0.0;
            pose.pose.orientation.w = 1.0;
            pose.pose.position.x = 0.0;
            pose.pose.position.y = 0.0;
            pose.pose.position.z = 0.0;
        }

        while (!list_to_expand.empty())
        {
            // Take the element from the top
            currentNode = *list_to_expand.begin();
            // Delete the element from the top
            list_to_expand.erase(list_to_expand.begin());
            if (currentNode.index == goal_index)
            {
                break;
            }
            // 若已有到达该点的更优路径,跳过该点
            if (currentNode.cost - getHeuristic(currentNode.index, goal_index) > gCosts[currentNode.index])
            {
                continue;
            }
            // 找到合适的相邻点
            vector<int> neighborIndexes = get_neighbors(currentNode.index);
            // 遍历所有合适的相邻点,并将符合条件的加入待扩展列表
            for (int i = 0; i < neighborIndexes.size(); i++)
            {
                if ((cameFrom[neighborIndexes[i]] == -1) ||
                    (gCosts[currentNode.index] + getMoveCost(currentNode.index, neighborIndexes[i]) < gCosts[neighborIndexes[i]]))
                {
                    gCosts[neighborIndexes[i]] = gCosts[currentNode.index] + getMoveCost(currentNode.index, neighborIndexes[i]);
                    Node nextNode;
                    nextNode.index = neighborIndexes[i];
                    nextNode.cost = gCosts[neighborIndexes[i]] + getHeuristic(neighborIndexes[i], goal_index) * cost_weight; // A* Algorithm
                    cameFrom[neighborIndexes[i]] = currentNode.index;
                    list_to_expand.insert(nextNode);
                }
            }
        }

        // 若无法到达goal
        if (cameFrom[goal_index] == -1 && getHeuristic(start_index, goal_index) > goal_search_tolerance_)
        {
            cout << "无法找到到达终点的路径!" << endl;
            return false;
        }

        // 将came_from[]中的所有Index转换成world下的Pose并publish。

        // 复现最短路径
        vector<int> bestPath;
        currentNode.index = goal_index;
        while (currentNode.index != start_index)
        {
            bestPath.push_back(cameFrom[currentNode.index]);
            currentNode.index = cameFrom[currentNode.index];
        }
        // 逆向排列bestPath使之回归从start到goal的顺序
        reverse(bestPath.begin(), bestPath.end());

        // Publish
        if (smooth == false)
        {
            for (int i = 0; i < bestPath.size(); i++)
            {
                unsigned int tmp1, tmp2;
                costmap_->indexToCells(bestPath[i], tmp1, tmp2);
                double x, y;
                // mapToWorld将map下的坐标点映射到world
                costmap_->mapToWorld(tmp1, tmp2, x, y);

                geometry_msgs::PoseStamped pose;
                pose.header.stamp = plan_time;
                pose.header.frame_id = costmap_ros_->getGlobalFrameID();
                pose.pose.position.x = x;
                pose.pose.position.y = y;

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

        double difference = sqrt(pow(firstXCord - secondXCord, 2) + pow(firstYCord - secondYCord, 2));
        // 异常检测
        if (abs(firstXCord - secondXCord) > 1 || abs(firstYCord - secondYCord) > 1)
        {
            ROS_ERROR("Astar global planner: Error in getMoveCost - difference not valid");
            return 1.0;
        }
        return difference;
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
        for (int i = -1; i <= 1; i++)
        {
            for (int j = -1; j <= 1; j++)
            {
                unsigned tmp1, tmp2;
                costmap_->indexToCells(current_cell, tmp1, tmp2);
                int nextX = tmp1 + i;
                int nextY = tmp2 + j;
                int nextIndex = costmap_->getIndex(nextX, nextY);
                if (!(i == 0 && j == 0) && isInBounds(nextX, nextY) && cost_[nextIndex] < inaccessible_cost_)
                {
                    neighborIndexes.push_back(nextIndex);
                }
            }
        }

        return neighborIndexes;
    }

    void AstarPlanner::publishPlan(const std::vector<geometry_msgs::PoseStamped> &path)
    {
        if (!initialized_)
        {
            ROS_ERROR("Astar全局规划器尚未初始化,请先初始化全局规划器!");
            return;
        }

        // 为路径创建一个消息
        nav_msgs::Path show_path;
        show_path.poses.resize(path.size());

        show_path.header.frame_id = frame_id_;
        show_path.header.stamp = ros::Time::now();

        // 在世界坐标中显示路径规划，我们假设路径在同一个框架中
        for (unsigned int i = 0; i < path.size(); i++)
        {
            show_path.poses[i] = path[i];
        }

        plan_pub_.publish(show_path);
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

// 给予 multiset (list_to_expand) 排序规则
bool operator<(const Node &x, const Node &y)
{
    return x.cost < y.cost;
}
