//Include code from https://fr.wikibooks.org/wiki/Structures_de_donn%C3%A9es_en_C/Les_listes_simples
//Algorithm from http://web.mit.edu/eranki/www/tutorials/search/
//Author: Florian Loiseau

#ifndef ASTAR_METHODS_H_INCLUDED
#define ASTAR_METHODS_H_INCLUDED

#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <string.h>

#ifdef __linux__
#define donotusecls 1
#endif // linux

#ifdef _WIN32
#define donotusecls 0
#endif // _WIN32

#define MAXI(a,b) (((a)>(b))?(a):(b))

typedef struct node node;
struct node
{
    node* parent;
    int x, y;
    int f, g, h;
};

typedef struct s_List List;
struct s_List
{
    List *next;
    node *data;
};

node*    create_node(node* parent, int x, int y, int goal_x, int goal_y);

List*   list_create(node* data);
List*   list_append(List *list_adress, node* data);
List*   node_delete(List *list_adress, node data);
void    delete_list(List *list_adress);

void    print_trajectory(unsigned short int map_param[10][9], int map_x, int map_y, int start_x, int start_y, int end_x, int end_y);
int     make_path(unsigned short int map_param[10][9], int map_x, int map_y, int start_x, int start_y, int end_x, int end_y);
#endif // ASTAR_METHODS_H_INCLUDED
