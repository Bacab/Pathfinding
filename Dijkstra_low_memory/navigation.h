#ifndef NAVIGATION_H_INCLUDED
#define NAVIGATION_H_INCLUDED

#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#define max_map_x 60
#define max_map_y 40

typedef struct node node;

struct node {
	unsigned heuristique : 16;
	unsigned position : 15;
	unsigned visite : 1;
};

void    print_trajectory(unsigned char map_param[max_map_y][max_map_x], int start_x, int start_y, int end_x, int end_y);
void    print_node_map(node node_map[max_map_y*max_map_x]);
void    build_node_map(unsigned char map_param[max_map_y][max_map_x], node node_map[max_map_y*max_map_x], float end_x, float end_y, int team);
void    next_case(node node_map[max_map_y*max_map_x],int pos_x, int pos_y, int* delta_x, int* delta_y);
void	team_to_nav(float abs_team_x, float abs_team_y, int *nav_x, int *nav_y, int team);
void	nav_to_team(int nav_x, int nav_y, float *abs_team_x, float *abs_team_y, int team);
int		walk_the_path(node node_map[max_map_y*max_map_x], float end_x, float end_y, int team);
int		get_dividende(int number, int diviseur);

#endif // NAVIGATION_H_INCLUDED
