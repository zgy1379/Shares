#include "iostream"
#include "math.h"
#include "time.h"
#include "ros/ros.h"
#include "nav_msgs/OccupancyGrid.h"
#include "visualization_msgs/Marker.h"
using namespace std;

const int HIGHT = 100, WIDTH = 100, LENGTH = 1000, rant = 40,
          ROAD = 0, WALL = 100, ORIGIN = 30, FINISH = 60, KNOWN = 15, UNKNOWN = 85, TRACK = 45;
// The wall generation probability is rant/100
const double WEIGHT = 1.0;

geometry_msgs::Point p;
visualization_msgs::Marker point1, point2, line_strip;
nav_msgs::OccupancyGrid A_map;

double mine_map[HIGHT][WIDTH][3];
// mine_map[x][y][0]=road/wall/origin/finish/known/unknown/track
// mine_map[x][y][1]=distance to the finish
// mine_map[x][y][2]=the orientation of the parent node
double line[LENGTH][3];
// line[n][1]=x, line[n][2]=y, line[n][3]=f(x)
double line_end[2];
// end point coordinates
int tail = 0, weight_max = 0, hight, width;

int init_Astar();
int randNext(int left, int right);
double distance(int x1, int y1, int x2, int y2);
int sort(int a[LENGTH][3], int terminal);
bool cmp(double i, double j);
int create_map(int hight, int width);
int Astar_search();
int spread();
int backtrack();

int main(int argc, char **argv)
{
    ros::init(argc, argv, "Astar_search_node");

    ros::NodeHandle n;
    ros::Publisher marker_pub = n.advertise<visualization_msgs::Marker>("/Astar_marker", 10);
    ros::Publisher map_pub = n.advertise<nav_msgs::OccupancyGrid>("/Astar_map", 10);

    ros::Rate r(1);
    while (ros::ok())
    {
        // header
        {
            point1.header.frame_id = point2.header.frame_id = line_strip.header.frame_id = A_map.header.frame_id = "map";
            point1.header.stamp = point2.header.stamp = line_strip.header.stamp = A_map.header.stamp = ros::Time::now();
            point1.ns = point2.ns = line_strip.ns = "points_and_lines";
            point1.action = point2.action = line_strip.action = visualization_msgs::Marker::ADD;
            point1.pose.orientation.w = point2.pose.orientation.w = line_strip.pose.orientation.w = 1.0;
        }

        init_Astar();

        // map_init
        {
            A_map.info.origin.position.x = 0;
            A_map.info.origin.position.y = 0;
            A_map.info.resolution = 1.0;
            A_map.info.width = width;
            A_map.info.height = hight;
        }

        // marker_init
        {
            point1.id = 0;
            point2.id = 1;
            line_strip.id = 2;
            point1.type = point2.type = visualization_msgs::Marker::POINTS;
            line_strip.type = visualization_msgs::Marker::LINE_STRIP;

            point1.points.clear();
            point2.points.clear();
            line_strip.points.clear();
        }

        // map_data
        {
            A_map.data.resize(width * hight);
        }

        // points markers use x and y scale for width/height respectively
        {
            point1.scale.x = point2.scale.x = 0.8;
            point1.scale.y = point2.scale.y = 0.8;
        }

        // line_strip markers use only the x component of scale, for the line width
        {
            line_strip.scale.x = 0.3;
        }

        // original point is green
        {
            point1.color.g = 1.0;
            point1.color.a = 1.0;
        }

        // final point is red
        {
            point2.color.r = 1.0;
            point2.color.a = 1.0;
        }

        // line strip is blue
        {
            line_strip.color.b = 1.0;
            line_strip.color.a = 1.0;
        }

        // Create the vertices for the points
        {
            p.z = 0.5;
            p.y = line_end[0];
            p.x = line_end[1];
            point2.points.push_back(p);
            line_strip.points.push_back(p);

            p.y = line[0][0];
            p.x = line[0][1];
            point1.points.push_back(p);
        }

        // search pathways with the A* algorithm
        {
            if (Astar_search())
            {
                // if there is a solution
                ;
            }
            else
            {
                // if there is no solution
                printf("Blind Alley!\n");
            }
            int i, j;
            for (i = 0; i < hight; i++)
            {
                for (j = 0; j < width; j++)
                {
                    A_map.data[i * width + j] = mine_map[i][j][0];
                }
            }
        }

        // publish
        {
            map_pub.publish(A_map);
            marker_pub.publish(point1);
            marker_pub.publish(point2);
            marker_pub.publish(line_strip);
        }

        r.sleep();
    }

    return 0;
}

int init_Astar()
{
    scanf("%d%d", &hight, &width); // get the map size
    if (hight < 2 || width < 2)
    {
        printf("The map is too cramped!\n");
        return 0;
    }
    else if (hight > 100 || width > 100)
    {
        printf("The map is too big!\n");
        return 0;
    }
    weight_max = 2147483647;
    create_map(hight, width); // create the random map
    return 0;
}

int randNext(int left, int right) // generate a random integer from left to right
{
    static unsigned int seed = 0;
    seed++;
    srand((unsigned)time(NULL) + seed * seed);
    return rand() % (right - left + 1) + left;
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

int sort(double a[LENGTH][3], int terminal) // sort from small to large according to f(x)
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

int create_map(int hight, int width)
{
    int i, j;
    for (i = 1; i < LENGTH; i++)
    {
        line[i][0] = 0;
        line[i][1] = 0;
        line[i][2] = weight_max;
    }
    line_end[0] = randNext(0, hight - 1);
    line_end[1] = randNext(0, width - 1);
    mine_map[int(line_end[0])][int(line_end[1])][0] = FINISH;
    line[0][0] = randNext(0, hight - 1);
    line[0][1] = randNext(0, width - 1);
    while (line[0][0] == line_end[0] && line[0][1] == line_end[1])
    {
        line[0][0] = randNext(0, hight - 1);
        line[0][1] = randNext(0, width - 1);
    }
    line[0][2] = 0;
    mine_map[int(line[0][0])][int(line[0][1])][0] = ORIGIN;
    for (i = 0; i < hight; i++)
    {
        for (j = 0; j < width; j++)
        {
            mine_map[i][j][1] = 0;
            if ((i == line[0][0] && j == line[0][1]) ||
                (i == line_end[0] && j == line_end[1]))
            {
                continue;
            }
            if (randNext(1, 100) <= rant)
            {
                mine_map[i][j][0] = WALL;
            }
            else
            {
                mine_map[i][j][0] = ROAD;
            }
        }
    }
    return 0;
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
        }
    }
    if (line[tail][0] == line_end[0] && line[tail][1] == line_end[1])
    {
        while (mine_map[int(line[0][0])][int(line[0][1])][0] != ORIGIN)
        {
            backtrack();
        }
        p.y = line[0][0];
        p.x = line[0][1];
        line_strip.points.push_back(p);
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
        if ((i + x < 0) || (i + x >= hight))
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
    line_strip.points.push_back(p);
    return 0;
}
