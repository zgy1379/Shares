#include "iostream"
#include "math.h"
#include "time.h"
using namespace std;

const int HIGHT = 100, WIDTH = 100, LENGTH = 1000, rant = 40,
          ROAD = 95, WALL = 79, ORIGIN = 64, FINISH = 36, KNOWN = 46, UNKNOWN = 63, TRACK = 42;
// The wall generation probability is rant/100
const double WEIGHT = 1;

double map[HIGHT][WIDTH][3];
// map[x][y][0]=road/wall/origin/finish/known/unknown/track
// map[x][y][1]=distance to the finish
// map[x][y][2]=the orientation of the parent node
double line[LENGTH][3];
// line[n][1]=x, line[n][2]=y, line[n][3]=f(x)
double line_end[2];
// end point coordinates
int tail = 0, weight_max = 0, hight, width;

int randNext(int left, int right);
double distance(int x1, int y1, int x2, int y2);
int sort(int a[LENGTH][3], int terminal);
bool cmp(double i, double j);
int create_map(int hight, int width);
int print_map(int hight, int width);
int Astar_search();
int spread();
int backtrack();

int main()
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
    // print_map(hight, width);  // print the original map
    if (Astar_search()) // search pathways with the A* algorithm
    {
        print_map(hight, width); // if there is a solution
    }
    else
    {
        print_map(hight, width); // if there is no solution
        printf("Blind Alley!\n");
    }
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
            if (a[i][2] > a[j][2])
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
    map[int(line_end[0])][int(line_end[1])][0] = FINISH;
    map[int(line_end[0])][int(line_end[1])][1] = 0;
    line[0][0] = randNext(0, hight - 1);
    line[0][1] = randNext(0, width - 1);
    while (line[0][0] == line_end[0] && line[0][1] == line_end[1])
    {
        line[0][0] = randNext(0, hight - 1);
        line[0][1] = randNext(0, width - 1);
    }
    line[0][2] = 0;
    map[int(line[0][0])][int(line[0][1])][0] = ORIGIN;
    map[int(line[0][0])][int(line[0][1])][1] = distance(line_end[0], line_end[1], line[0][0], line[0][1]) * WEIGHT;
    for (i = 0; i < hight; i++)
    {
        for (j = 0; j < width; j++)
        {
            if ((i == line[0][0] && j == line[0][1]) ||
                (i == line_end[0] && j == line_end[1]))
            {
                continue;
            }
            if (randNext(1, 100) <= rant)
            {
                map[i][j][0] = WALL;
            }
            else
            {
                map[i][j][0] = ROAD;
            }
            map[i][j][1] = distance(i, j, line_end[0], line_end[1]) * WEIGHT;
        }
    }
    return 0;
}

int print_map(int hight, int width)
{
    int i, j;
    for (i = 0; i < hight; i++)
    {
        for (j = 0; j < width; j++)
        {
            printf("%c", int(map[i][j][0]));
        }
        printf("\n");
    }
    printf("\n");
    // for (i = 0; i < hight; i++)
    // {
    //     for (j = 0; j < width; j++)
    //     {
    //         printf("%.2lf\t", map[i][j][2]);
    //     }
    //     printf("\n");
    // }
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
        while (map[int(line[0][0])][int(line[0][1])][0] != ORIGIN)
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
    int i, j, x = line[0][0], y = line[0][1], dis = line[0][2];
    line[0][2] = weight_max;
    if (map[x][y][0] == UNKNOWN)
    {
        map[x][y][0] = KNOWN;
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
            else if (map[x + i][y + j][0] == ROAD || map[x + i][y + j][0] == FINISH)
            {
                tail++;
                line[tail][0] = x + i;
                line[tail][1] = y + j;
                line[tail][2] = dis + map[x + i][y + j][1] + distance(0, 0, i, j);
                map[x + i][y + j][2] = i * 3 + j;
                if (map[x + i][y + j][0] == FINISH)
                {
                    return 0;
                }
                map[x + i][y + j][0] = UNKNOWN;
            }
            else if (map[x + i][y + j][0] == UNKNOWN)
            {
                int k;
                for (k = 1; k <= tail; k++)
                {
                    if ((x + i == line[k][0]) && (y + j == line[k][1]))
                    {
                        if (line[k][2] > dis + map[x + i][y + j][1] + distance(0, 0, i, j))
                        {
                            line[k][2] = dis + map[x + i][y + j][1] + distance(0, 0, i, j);
                            map[x + i][y + j][2] = i * 3 + j;
                        }
                        break;
                    }
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
        dx = int(map[x][y][2] + 4) / 3 - 1, dy = int(map[x][y][2] + 4) % 3 - 1;
    line[0][0] = x - dx;
    line[0][1] = y - dy;
    map[x][y][0] = TRACK;
    return 0;
}
