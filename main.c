#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#define map_x 9
#define map_y 10
#define debug 1

void print_trajectory(unsigned short int map_param[map_x][map_x], int start_x, int start_y)
{
    system("cls");
    for (int j = 0; j < map_y; j++)
    {
        for (int i = 0; i < map_x; i++)
        {
            printf("\t");
            if((i!=start_x)||(j!=start_y))
            {
                switch(map_param[j][i])
                {
                    case 180:
                        printf("*");
                        break;
                    case 255:
                        printf("||");
                        break;
                    case 0:
                        printf("G");//Il faut atteindre le point G ^_^
                        break;
                    default:
                        printf(" ");
                        break;
                }
                //printf("%d",map_param[j][i]);
            }
            else
            {
                printf("R");//On part d'ici
            }
        }
        printf("\n");
    }
}

void build_cost_map(short unsigned int map_param[map_y][map_x], int start_x, int start_y, int end_x, int end_y)
{
    for (int j = 0; j < map_y; j++)
    {
        for (int i = 0; i < map_x; i++)
        {
            if (map_param[j][i] != 255)
            {
                map_param[j][i] = round(sqrt(pow((j-end_y),2)+pow((i-end_x),2)));
            }
        }
    }
    map_param[end_y][end_x] = 0;
}

void next_case(short unsigned int map_param[map_y][map_x],int pos_x, int pos_y, int* delta_x, int* delta_y)
{
    unsigned short int lesser_cost = 255;
    int tmp_x = 0;
    int tmp_y = 0;
    int lesser_cost_x = 0;
    int lesser_cost_y = 0;

    for (int j = -1; j <= 1; j++)
    {
        for (int i = -1; i <= 1; i++)
        {
            tmp_x = pos_x + i;
            tmp_y = pos_y + j;

            if ((tmp_x >= 0) && (tmp_x < map_x) && (tmp_y >= 0) && (tmp_y < map_y) && ((tmp_x!=pos_x) || (tmp_y!=pos_y)))
            {
                if (map_param[tmp_y][tmp_x] < lesser_cost)
                {
                    lesser_cost = map_param[tmp_y][tmp_x];
                    lesser_cost_x = i;
                    lesser_cost_y = j;
                }
            }
        }
    }
    *delta_x = lesser_cost_x;
    *delta_y = lesser_cost_y;
}

int make_path(short unsigned int map_param[map_y][map_x], int start_x, int start_y, int end_x, int end_y)
{
    int robot_x = start_x;
    int robot_y = start_y;
    int next_x = 0;
    int next_y = 0;
    int nb_iter = 0;

    while ((robot_x != end_x) || (robot_y != end_y))
    {
        next_case(map_param, robot_x, robot_y, &next_x, &next_y);
        if((map_param[robot_y][robot_x]>=180)&&(map_param[robot_y][robot_x]<250))
        {
            map_param[robot_y][robot_x]+=10;
        }
        else
        {
            map_param[robot_y][robot_x]=180;
        }
        robot_x += next_x;
        robot_y += next_y;
        nb_iter+=1;
        print_trajectory(map_param,robot_x,robot_y);
        printf("Appuyer sur entre pour continuer\n");
        getchar();
    }
    return nb_iter;
}

int main()
{
    short unsigned int main_map[map_y][map_x] = {   {255, 255, 255, 255, 255, 255, 255, 255, 255},
                                                    {255, 000, 000, 000, 000, 000, 000, 000, 255},
                                                    {255, 000, 255, 255, 255, 255, 255, 000, 255},
                                                    {255, 000, 000, 000, 255, 000, 255, 000, 255},
                                                    {255, 000, 255, 255, 255, 000, 255, 000, 255},
                                                    {255, 000, 000, 000, 255, 000, 255, 000, 255},
                                                    {255, 000, 255, 255, 255, 000, 255, 000, 255},
                                                    {255, 000, 000, 000, 255, 000, 255, 000, 255},
                                                    {255, 000, 255, 000, 255, 000, 000, 000, 255},
                                                    {255, 255, 255, 255, 255, 255, 255, 255, 255}};
    int goal_x = 7;
    int goal_y = 4;

    int robot_x_init = 3;
    int robot_y_init = 8;

    int step = 0;

    build_cost_map(main_map, robot_x_init, robot_y_init, goal_x, goal_y);
    step = make_path(main_map, robot_x_init, robot_y_init, goal_x, goal_y);
    printf("%d\n",step);
    if(debug)
    {
        print_trajectory(main_map,robot_x_init,robot_y_init);
    }
    return 0;
}
