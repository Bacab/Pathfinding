#ifndef NAVIGATION_H_INCLUDED
#define NAVIGATION_H_INCLUDED

#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#define max_map_x 60
#define max_map_y 40

typedef struct node node;
struct node
{
    int x, y;
    int visite;
    float distance;
};

void    print_trajectory(unsigned char map_param[max_map_y][max_map_x], int start_x, int start_y, int end_x, int end_y);
void    print_node_map(node node_map[max_map_y][max_map_x]);
void    build_node_map(unsigned char map_param[max_map_y][max_map_x], node node_map[max_map_y][max_map_x], int end_x, int end_y);
void    next_case(node node_map[max_map_y][max_map_x],int pos_x, int pos_y, int* delta_x, int* delta_y);
int     walk_the_path(unsigned char map_param[max_map_y][max_map_x], node node_map[max_map_y][max_map_x], int start_x, int start_y, int end_x, int end_y);

#endif // NAVIGATION_H_INCLUDED
