#include "navigation.h"

/*DEBUG ONLY
Enregistre le chemin suivi par le robot*/
void print_trajectory(unsigned char map_param[max_map_y][max_map_x], int start_x, int start_y, int end_x, int end_y)
{
    int j = 0;
    int i = 0;
    FILE* my_file = fopen("result_trajectory.txt","w");
    if(my_file)
    {
        for (j = 0; j < max_map_y; j++)
        {
            for (i = 0; i < max_map_x; i++)
            {
                fprintf(my_file, "\t");
                if(((i!=start_x)||(j!=start_y))&&((i!=end_x)||(j!=end_y)))
                {
                    switch(map_param[j][i])
                    {
                        case 180:
                            fprintf(my_file,"*");
                            break;
                        case 255:
                            fprintf(my_file,"||");
                            break;
                        default:
                            fprintf(my_file," ");
                            break;
                    }
                }
                else if((i==start_x)&&(j==start_y))
                {
                    fprintf(my_file,"R");/*On part d'ici*/
                }
                else if((i==end_x)&&(j==end_y))
                {
                    fprintf(my_file,"G");/*Il faut atteindre le point G ^_^*/
                }
            }
            fprintf(my_file,"\n");
        }
        fclose(my_file);
    }
}
/*DEBUG ONLY
Enregistre l'heuristique de chaque noeud*/
void print_node_map(node node_map[max_map_y][max_map_x])
{
    int j = 0;
    int i = 0;
    FILE* my_file = fopen("result_heuristic.txt","w");
    if(my_file)
    {
        for (j = 0; j < max_map_y; j++)
        {
            for (i = 0; i < max_map_x; i++)
            {
                fprintf(my_file,"\t");
                if(node_map[j][i].distance!=9999.0)
                    fprintf(my_file, "%f",node_map[j][i].distance);
                else
                    fprintf(my_file,"||");
            }
            fprintf(my_file, "\n");
        }
    }
}

/*Construit l'heuristique de chaque noeud de maniere a
ce que, en se deplacant de case de moindre cout en case de moindre cout, peut
importe la position de depart du robot, le chemin vers le but soit optimal*/
void build_node_map(unsigned char map_param[max_map_y][max_map_x], node node_map[max_map_y][max_map_x], int end_x, int end_y)
{
    float smallest_dist = 9999.0;
    int curr_x = end_x;
    int curr_y = end_y;
    int tmp_x = 0;
    int tmp_y = 0;
    int nb_node = 0;

    node* a_traiter[max_map_x*max_map_y];

    for (int j=0; j<max_map_y; j++)
    {
        for(int i=0; i<max_map_x; i++)
        {
            node_map[j][i].x = i;
            node_map[j][i].y = j;
            if(map_param[j][i]!=255)
            {
                node_map[j][i].distance = 9999.0;
                node_map[j][i].visite = 0;
                a_traiter[nb_node]=&node_map[j][i];
                nb_node+=1;
            }
            else
            {
                node_map[j][i].distance = 9999.0;
                node_map[j][i].visite = 1;
            }
        }
    }

    node_map[end_y][end_x].distance = 0.0;

    do
    {
        for (int j=-1; j<=1; j++)
        {
            for(int i=-1; i<=1; i++)
            {
                tmp_x = curr_x+i;
                tmp_y = curr_y+j;
                if((tmp_x<max_map_x)&&(tmp_y<max_map_y)&&(tmp_x>=0)&&(tmp_y>=0)&&((i!=0)||(j!=0)))
                {
                    if((node_map[tmp_y][tmp_x].visite==0)&&(node_map[curr_y][curr_x].distance + sqrt(i*i+j*j) < node_map[tmp_y][tmp_x].distance))
                    {
                        node_map[tmp_y][tmp_x].distance = node_map[curr_y][curr_x].distance + sqrt(i*i+j*j);
                    }
                }
            }
        }
        node_map[curr_y][curr_x].visite = 1;
        smallest_dist  =9999.0;
        for(int l=0;l<nb_node;l++)
        {
            if((a_traiter[l]->distance<smallest_dist)&&(a_traiter[l]->visite==0))
            {
                smallest_dist = a_traiter[l]->distance;
                curr_x = a_traiter[l]->x;
                curr_y = a_traiter[l]->y;
            }
        }
    }while(smallest_dist!=9999.0);
}

/*Choisi la case adjacente de moindre coût*/
void next_case(node node_map[max_map_y][max_map_x],int pos_x, int pos_y, int* delta_x, int* delta_y)
{
    float lesser_cost = 9999.0;
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

            if ((tmp_x >= 0) && (tmp_x < max_map_x) && (tmp_y >= 0) && (tmp_y < max_map_y) && ((tmp_x!=pos_x) || (tmp_y!=pos_y)))
            {
                if (node_map[tmp_y][tmp_x].distance < lesser_cost)
                {
                    lesser_cost = node_map[tmp_y][tmp_x].distance;
                    lesser_cost_x = i;
                    lesser_cost_y = j;
                }
            }
        }
    }
    *delta_x = lesser_cost_x;
    *delta_y = lesser_cost_y;
}

/*Parcours le chemin en fonction des heuristiques et retounr le nombre d'etape avant d'arriver au but, modifie la carte !*/
int walk_the_path(unsigned char map_param[max_map_y][max_map_x], node node_map[max_map_y][max_map_x], int start_x, int start_y, int end_x, int end_y)
{
    int robot_x = start_x;
    int robot_y = start_y;
    int next_x = 0;
    int next_y = 0;
    int nb_iter = 0;

    while ((robot_x != end_x) || (robot_y != end_y))
    {
        next_case(node_map, robot_x, robot_y, &next_x, &next_y);
        map_param[robot_y][robot_x]=180;

        robot_x += next_x;
        robot_y += next_y;
        nb_iter+=1;
    }
    return nb_iter;
}
