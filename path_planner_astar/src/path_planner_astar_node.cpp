#include <path_planner_astar.h>
#include <pluginlib/class_list_macros.h>
#include <math.h>

using namespace std;

const int rant = 40, ROAD = 0, WALL = 100, ORIGIN = 30, FINISH = 60, KNOWN = 15, UNKNOWN = 85, TRACK = 45;
// The wall generation probability is rant/100
const double WEIGHT = 1.0;

bool operator<(const Node &x, const Node &y);
int randNext(int left, int right);
double distance(int x1, int y1, int x2, int y2);
int sort(vector<vector<double>> a, int terminal);
bool cmp(double i, double j);
int Astar_search();
int spread();
int backtrack();

geometry_msgs::Point p;

vector<vector<vector<int>>> mine_map;
// mine_map[x][y][0]=road/wall/origin/finish/known/unknown/track
// mine_map[x][y][1]=distance to the finish
// mine_map[x][y][2]=the orientation of the parent node
vector<vector<double>> line;
vector<geometry_msgs::Point> bestPath;
// line[n][1]=x, line[n][2]=y, line[n][3]=f(x)
double line_end[2];
// end point coordinates
int tail = 0, weight_max = 0, height, width;

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

            // 创建一个int类型的数组，数组大小为栅格的个数，存储每个栅格的状态
            mine_map.resize(width * height * 3);
            line.resize(9 * 3);
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
                        mine_map[i][j][0] = ROAD;
                    else
                        mine_map[i][j][0] = WALL;
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
        unsigned int start_x, start_y, goal_x, goal_y, i, j;
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
            line[0][0] = start_x;
            line[0][1] = start_y;
            line[0][2] = 0;
            mine_map[int(line[0][0])][int(line[0][1])][0] = ORIGIN;
            // 读取goal坐标
            wx = goal.pose.position.x;
            wy = goal.pose.position.y;
            costmap_->worldToMap(wx, wy, goal_x, goal_y); // wordToMap将world坐标点映射到map
            line_end[0] = goal_x;
            line_end[1] = goal_y;
            mine_map[int(line_end[0])][int(line_end[1])][0] = FINISH;

            for (i = 0; i < height; i++)
            {
                for (j = 0; j < width; j++)
                {
                    mine_map[i][j][1] = 0;
                    mine_map[i][j][2] = 0;
                }
            }
            // 初始化可视化点的属性
            pose.pose.orientation.x = 0.0;
            pose.pose.orientation.y = 0.0;
            pose.pose.orientation.z = 0.0;
            pose.pose.orientation.w = 1.0;
            pose.pose.position.x = 0.0;
            pose.pose.position.y = 0.0;
            pose.pose.position.z = 0.0;
        }

        if (Astar_search())
        {
            // if there is a solution
            ;
        }
        else
        {
            // if there is no solution
            cout << "无法找到到达终点的路径!" << endl;
            return false;
        }

        // 将came_from[]中的所有Index转换成world下的Pose并publish。

        if (start_x == goal_x && start_y == goal_y)
            return false;
        // 复现最短路径
        // 逆向排列bestPath使之回归从start到goal的顺序
        reverse(bestPath.begin(), bestPath.end());
        // Publish
        for (int i = 0; i < bestPath.size(); i++)
        {
            unsigned int tmp1, tmp2;
            bestPath[i].x = tmp1;
            bestPath[i].y = tmp2;
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

double distance(int x1, int y1, int x2, int y2) // calculate the map distance between the two points
{
    int min, max;
    if (abs(x1 - x2) > abs(y1 - y2))
    {
        min = abs(y1 - y2);
        max = abs(x1 - x2);
    }
    else
    {
        min = abs(x1 - x2);
        max = abs(y1 - y2);
    }
    return (sqrt(2) * min + max - min);
}

int Astar_search()
{
    while (tail >= 0 && (line[tail][0] != line_end[0] || line[tail][1] != line_end[1]))
    {
        spread();
        if (line[tail][0] == line_end[0] && line[tail][1] == line_end[1])
        {
            break;
        }
        else
        {
            sort(line, tail + 1);
            line.resize(tail + 8 * 3);
        }
    }
    if (line[tail][0] == line_end[0] && line[tail][1] == line_end[1])
    {
        while (mine_map[int(line[0][0])][int(line[0][1])][0] != ORIGIN)
        {
            backtrack();
        }
    }
    else
    {
        return 0;
    }
    return 1;
}

int spread() // expand the child nodes to search for paths
{
    int i, j, x = line[0][0], y = line[0][1];
    double dis = line[0][2];
    line[0][2] = weight_max;
    if (mine_map[x][y][0] == UNKNOWN)
    {
        mine_map[x][y][0] = KNOWN;
    }
    for (i = -1; i <= 1; i++)
    {
        if ((i + x < 0) || (i + x >= height))
        {
            continue;
        }
        for (j = -1; j <= 1; j++)
        {
            if ((i == 0 && j == 0) || (j + y < 0) || (j + y >= width))
            {
                continue;
            }
            double new_dis = dis + distance(0, 0, i, j);
            if (mine_map[x + i][y + j][0] == ROAD || mine_map[x + i][y + j][0] == FINISH)
            {
                tail++;
                line[tail][0] = x + i;
                line[tail][1] = y + j;
                line[tail][2] = new_dis;
                mine_map[x + i][y + j][1] = new_dis;
                mine_map[x + i][y + j][2] = i * 3 + j;
                if (mine_map[x + i][y + j][0] == FINISH)
                {
                    return 0;
                }
                mine_map[x + i][y + j][0] = UNKNOWN;
            }
            else if (mine_map[x + i][y + j][1] > new_dis)
            {
                mine_map[x + i][y + j][1] = new_dis;
                mine_map[x + i][y + j][2] = i * 3 + j;
                if (mine_map[x + i][y + j][0] == UNKNOWN)
                {
                    int k;
                    for (k = 1; k <= tail; k++)
                    {
                        if ((x + i == line[k][0]) && (y + j == line[k][1]))
                        {
                            line[k][2] = new_dis;
                            break;
                        }
                    }
                }
                else if (mine_map[x + i][y + j][0] == KNOWN)
                {
                    tail++;
                    line[tail][0] = x + i;
                    line[tail][1] = y + j;
                    line[tail][2] = new_dis;
                }
            }
        }
    }
    line[0][0] = 0;
    line[0][1] = 0;
    tail--;
    return 0;
}

int backtrack() // repeat the path with the parent node orientation
{
    int x = line[0][0], y = line[0][1],
        dx = int(mine_map[x][y][2] + 4) / 3 - 1, dy = int(mine_map[x][y][2] + 4) % 3 - 1;
    line[0][0] = x - dx;
    line[0][1] = y - dy;
    mine_map[x][y][0] = TRACK;
    // Create the vertices for the lines
    p.x = y;
    p.y = x;
    bestPath.push_back(p);
    return 0;
}

int sort(vector<vector<double>> a, int terminal) // sort from small to large according to f(x)
{
    int i, j;
    double exchange;
    for (i = 0; i < terminal; i++)
    {
        for (j = i + 1; j <= terminal; j++)
        {
            if (a[i][2] + distance(a[i][0], a[i][1], line_end[0], line_end[1]) * WEIGHT > a[j][2] + distance(a[j][0], a[j][1], line_end[0], line_end[1]) * WEIGHT)
            {
                exchange = a[i][0];
                a[i][0] = a[j][0];
                a[j][0] = exchange;
                exchange = a[i][1];
                a[i][1] = a[j][1];
                a[j][1] = exchange;
                exchange = a[i][2];
                a[i][2] = a[j][2];
                a[j][2] = exchange;
            }
        }
    }
    return 0;
}
