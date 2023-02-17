#include <path_planner_astar.h>
#include <pluginlib/class_list_macros.h>
#include <math.h>

// Export path_planner_astar::PathPlannerAstar class as the ROS plug-in
PLUGINLIB_EXPORT_CLASS(path_planner_astar::PathPlannerAstar, nav_core::BaseGlobalPlanner)

namespace path_planner_astar
{
    PathPlannerAstar::PathPlannerAstar() {}

    PathPlannerAstar::PathPlannerAstar(std::string name, costmap_2d::Costmap2DROS *costmap_ros)
    {
        // 通过构造函数初始化并同时调用初始化函数
        initialize(name, costmap_ros);
    }

    void PathPlannerAstar::initialize(std::string name, costmap_2d::Costmap2DROS *costmap_ros)
    {
        if (!initialized_)
        {
            // 初始化动态代价地图
            costmap_ros_ = costmap_ros;
            // 获取静态代价地图
            costmap_ = costmap_ros_->getCostmap();
            // 获取全局地图坐标系的ID
            frame_id_ = costmap_ros->getGlobalFrameID();
            // 给节点加命名空间，变为私有节点
            ros::NodeHandle private_nh("move_base/");
            // 用于发布全局路径
            plan_pub_ = private_nh.advertise<nav_msgs::Path>("path_plan_astar", 1);
            // 地图宽度
            width = costmap_->getSizeInCellsX();
            // 地图高度
            height = costmap_->getSizeInCellsY();
            // 地图大小
            map_size = width * height;

            // 创建一个int类型的数组，数组大小为栅格的个数，存储每个栅格的状态
            mine_map.resize(map_size);
            /*
            遍历所有栅格的代价值，并把其分为 道路(true) 和 障碍(false)，存在一个bool数组中
              乘以 width 的原因：从栅格地图的左下角第一个栅格开始数，依次存入mine_map数组中
            */
            for (int i = 0; i < width; i++)
            {
                for (int j = 0; j < height; j++)
                {
                    unsigned int cost = static_cast<int>(costmap_->getCost(j, i));
                    // get_cost << cost << endl;
                    // cout << "i:, j:" << cost << endl;

                    if (cost == 0)
                        mine_map[i * width + j] = true;
                    else
                        mine_map[i * width + j] = false;
                }
            }

            ROS_INFO("Astar全局规划器初始化成功!");

            initialized_ = true;
        }
        else
            ROS_WARN("Astar全局规划器初始化失败...");
    }

    // makePlan inherit nav_core::BaseGlobalPlanner ,publish plan
    bool PathPlannerAstar::makePlan(const geometry_msgs::PoseStamped &start,
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
            // 读取start坐标
            wx = start.pose.position.x;
            wy = start.pose.position.y;
            costmap_->worldToMap(wx, wy, start_x, start_y);
            start_index = costmap_->getIndex(start_x, start_y);
            // 读取goal坐标
            wx = goal.pose.position.x;
            wy = goal.pose.position.y;
            costmap_->worldToMap(wx, wy, goal_x, goal_y); // wordToMap将world坐标点映射到map
            goal_index = costmap_->getIndex(goal_x, goal_y);
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
            // 选取最前端(代价最低)的点
            currentNode = *list_to_expand.begin();
            // 删除该点
            list_to_expand.erase(list_to_expand.begin());
            // 若为终点,结束循环
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
                    nextNode.cost = gCosts[neighborIndexes[i]] + getHeuristic(neighborIndexes[i], goal_index); // A* Algorithm
                    cameFrom[neighborIndexes[i]] = currentNode.index;
                    list_to_expand.insert(nextNode);
                }
            }
        }

        // 若无法到达goal
        if (cameFrom[goal_index] == -1)
        {
            cout << "无法找到到达终点的路径!" << endl;
            return false;
        }

        // 将came_from[]中的所有Index转换成world下的Pose并publish。

        if (start_index == goal_index)
            return false;
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
        for (int i = 0; i < bestPath.size(); i++)
        {
            unsigned int tmp1, tmp2;
            costmap_->indexToCells(bestPath[i], tmp1, tmp2);
            double x, y;
            costmap_->mapToWorld(tmp1, tmp2, x, y); // mapToWorld将map下的坐标点映射到world

            pose.header.stamp = plan_time;
            pose.header.frame_id = costmap_ros_->getGlobalFrameID();
            pose.pose.position.x = x;
            pose.pose.position.y = y;

            plan.push_back(pose);
        }
        plan.push_back(goal);
        publishPlan(plan);
        return true;
    }

    double PathPlannerAstar::getMoveCost(int firstIndex, int secondIndex)
    {
        unsigned int tmp1, tmp2;
        costmap_->indexToCells(firstIndex, tmp1, tmp2);
        int firstXCord = tmp1, firstYCord = tmp2;
        costmap_->indexToCells(secondIndex, tmp1, tmp2);
        int secondXCord = tmp1, secondYCord = tmp2;

        double difference = sqrt(pow(firstXCord - secondXCord, 2) + pow(firstYCord - secondYCord, 2));
        // Error checking
        if (abs(firstXCord - secondXCord) > 1 || abs(firstYCord - secondYCord) > 1)
        {
            ROS_ERROR("Astar global planner: Error in getMoveCost - difference not valid");
            return 1.0;
        }
        return difference;
    }

    double PathPlannerAstar::getHeuristic(int cell_index, int goal_index)
    {
        unsigned int tmp1, tmp2;
        costmap_->indexToCells(cell_index, tmp1, tmp2);
        int startX = tmp1, startY = tmp2;
        costmap_->indexToCells(goal_index, tmp1, tmp2);
        int goalX = tmp1, goalY = tmp2;
        int min = (abs(goalY - startY) < abs(goalX - startX) ? abs(goalY - startY) : abs(goalX - startX));

        return abs(goalY - startY) + abs(goalX - startX) + (sqrt(2) - 2) * min;
    }

    bool PathPlannerAstar::isInBounds(int x, int y)
    {
        if (x < 0 || y < 0 || x >= height || y >= width)
            return false;
        return true;
    }

    vector<int> PathPlannerAstar::get_neighbors(int current_cell)
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
                if (!(i == 0 && j == 0) && isInBounds(nextX, nextY) && mine_map[nextIndex])
                {
                    neighborIndexes.push_back(nextIndex);
                }
            }
        }

        // {
        //     if (current_cell - width >= 0 && mine_map[current_cell - width])
        //     {
        //         neighborIndexes.push_back(current_cell - width); // up
        //     }
        //     if (current_cell - width - 1 >= 0 && (current_cell - width - 1 + 1) % width != 0 && mine_map[current_cell - width - 1])
        //     {
        //         neighborIndexes.push_back(current_cell - width - 1); // left_up
        //     }
        //     if (current_cell - 1 >= 0 && (current_cell - 1 + 1) % width != 0 && mine_map[current_cell - 1])
        //     {
        //         neighborIndexes.push_back(current_cell - 1); // left
        //     }
        //     if (current_cell + width - 1 < width * height && (current_cell + width - 1 + 1) % width != 0 && mine_map[current_cell + width - 1])
        //     {
        //         neighborIndexes.push_back(current_cell + width - 1); // left_down
        //     }
        //     if (current_cell + width < width * height && mine_map[current_cell + width])
        //     {
        //         neighborIndexes.push_back(current_cell + width); // down
        //     }
        //     if (current_cell + width + 1 < width * height && (current_cell + width + 1) % width != 0 && mine_map[current_cell + width + 1])
        //     {
        //         neighborIndexes.push_back(current_cell + width + 1); // right_down
        //     }
        //     if (current_cell + 1 < width * height && (current_cell + 1) % width != 0 && mine_map[current_cell + 1])
        //     {
        //         neighborIndexes.push_back(current_cell + 1); // right
        //     }
        //     if (current_cell - width + 1 >= 0 && (current_cell - width + 1) % width != 0 && mine_map[current_cell - width + 1])
        //     {
        //         neighborIndexes.push_back(current_cell - width + 1); // right_up
        //     }
        // }

        return neighborIndexes;
    }

    // 路径可视化
    void PathPlannerAstar::publishPlan(const std::vector<geometry_msgs::PoseStamped> &path)
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
};
// 给予 multiset (list_to_expand) 排序规则
bool operator<(const Node &x, const Node &y)
{
    return x.cost < y.cost;
}