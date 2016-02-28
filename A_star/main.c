#include "astar_methods.h"

#define map_x 9
#define map_y 10
#define debug 1


int main()
{
    short unsigned int main_map[map_y][map_x] = {   {255, 255, 255, 255, 255, 255, 255, 255, 255},
                                                    {255, 000, 000, 000, 000, 000, 000, 000, 255},
                                                    {255, 000, 255, 255, 255, 255, 255, 000, 255},
                                                    {255, 000, 255, 000, 000, 000, 000, 000, 255},
                                                    {255, 000, 255, 255, 255, 000, 255, 255, 255},
                                                    {255, 000, 255, 000, 000, 000, 000, 000, 255},
                                                    {255, 000, 255, 000, 000, 000, 000, 000, 255},
                                                    {255, 000, 255, 000, 000, 000, 000, 000, 255},
                                                    {255, 000, 255, 000, 000, 000, 000, 000, 255},
                                                    {255, 255, 255, 255, 255, 255, 255, 255, 255}};
    int goal_x = 7;
    int goal_y = 8;
    int robot_x_init = 1;
    int robot_y_init = 1;
    int step = 0;

    step = make_path(main_map, map_x, map_y, robot_x_init, robot_y_init, goal_x, goal_y);
    if(debug)
    {
        print_trajectory(main_map, map_x, map_y,robot_x_init,robot_y_init,goal_x,goal_y);
        printf("%d\n",step);
    }
    return 0;
}
