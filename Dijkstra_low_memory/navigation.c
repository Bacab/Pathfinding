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
void print_node_map(node node_map[max_map_y * max_map_x])
{
    int i = 0;
    FILE* my_file = fopen("result_heuristic.txt","w");
    if(my_file)
    {
        for (i = 0; i < max_map_x*max_map_y; i++)
        {
            fprintf(my_file,"\t");
            if(node_map[i].heuristique!=0xFFFF)
                fprintf(my_file, "%d",node_map[i].heuristique);
            else
                fprintf(my_file,"||");
			if (i%max_map_x == 0)
			{
				fprintf(my_file, "\n");
			}
        }  
    }
}

/*Construit l'heuristique de chaque noeud de maniere a
ce que, en se deplacant de case de moindre cout en case de moindre cout, peut
importe la position de depart du robot, le chemin vers le but soit optimal*/
void build_node_map(unsigned char map_param[max_map_y][max_map_x], node node_map[max_map_x*max_map_y], int end_x, int end_y)
{
    unsigned short int smallest_heur = 0xFFFF;
	node * tmp;
	int curr_x = 0;
    int curr_y = 0;
	int i = 0;

	for (i = 0; i<max_map_y*max_map_x; i++)
    {
		node_map[i].position = i;
		curr_x = i%max_map_x;
		curr_y = (i - i%max_map_x) / max_map_x;
		//(float)sqrt((curr_y - end_y)*(curr_y - end_y) - (curr_x - end_x)*(curr_x - end_x));
		if ((curr_x == end_x) && (curr_y == end_y))
		{
			node_map[i].heuristique = 0;
			node_map[i].visite = 0;
		}
		else if (map_param[curr_y][curr_x] != 255)
		{
			node_map[i].heuristique = 0xFFFF;
			node_map[i].visite = 0;
		}
		else
		{
			node_map[i].heuristique = 0xFFFF;
			node_map[i].visite = 1;
        }
    }
	do
	{
		smallest_heur = 0xFFFF;
		for (i = 0; i < max_map_y*max_map_x; i++)
		{
			if ((node_map[i].visite == 0) && (node_map[i].heuristique<smallest_heur))
			{
				tmp = &node_map[i];
				smallest_heur = tmp->heuristique;
			}
		}
		for (i = 0; i < 8; i++)
		{
			switch(i)
			{
				case 0:
					if (tmp->position - 1 >= 0)
					{
						if ((node_map[tmp->position - 1].visite == 0) && (node_map[tmp->position - 1].heuristique > (tmp->heuristique + 1)))
						{
							node_map[tmp->position - 1].heuristique = tmp->heuristique + 1;
						}
					}
					break;
				case 1:
					if (tmp->position + 1 < max_map_y*max_map_x)
					{
						if ((node_map[tmp->position + 1].visite == 0) && (node_map[tmp->position + 1].heuristique > (tmp->heuristique + 1)))
						{
							node_map[tmp->position + 1].heuristique = tmp->heuristique + 1;
						}
					}
					break;
				case 2:
					if (tmp->position + max_map_x < max_map_y*max_map_x)
					{
						if ((node_map[tmp->position + max_map_x].visite == 0) && (node_map[tmp->position + max_map_x].heuristique > (tmp->heuristique + 1)))
						{
							node_map[tmp->position + max_map_x].heuristique = tmp->heuristique + 1;
						}
					}
					break;
				case 3:
					if (tmp->position - max_map_x >= 0)
					{
						if ((node_map[tmp->position - max_map_x].visite == 0) && (node_map[tmp->position - max_map_x].heuristique > (tmp->heuristique + 1)))
						{
							node_map[tmp->position - max_map_x].heuristique = tmp->heuristique + 1;
						}
					}
					break;
				case 4:
					if (tmp->position + max_map_x + 1 < max_map_y*max_map_x)
					{
						if ((node_map[tmp->position + max_map_x + 1].visite == 0) && (node_map[tmp->position + max_map_x + 1].heuristique > (tmp->heuristique + 1)))
						{
							node_map[tmp->position + max_map_x + 1].heuristique = tmp->heuristique + 2;
						}
					}
					break;
				case 5:
					if (tmp->position + max_map_x - 1 < max_map_y*max_map_x)
					{
						if ((node_map[tmp->position + max_map_x - 1].visite == 0) && (node_map[tmp->position + max_map_x - 1].heuristique > (tmp->heuristique + 1)))
						{
							node_map[tmp->position + max_map_x - 1].heuristique = tmp->heuristique + 2;
						}
					}
					break;
				case 6:
					if (tmp->position - max_map_x - 1 >= 0)
					{
						if ((node_map[tmp->position - max_map_x - 1].visite == 0) && (node_map[tmp->position - max_map_x - 1].heuristique > (tmp->heuristique + 1)))
						{
							node_map[tmp->position - max_map_x - 1].heuristique = tmp->heuristique + 2;
						}
					}
					break;
				case 7:
					if (tmp->position - max_map_x + 1 >= 0)
					{
						if ((node_map[tmp->position - max_map_x + 1].visite == 0) && (node_map[tmp->position - max_map_x + 1].heuristique > (tmp->heuristique + 1)))
						{
							node_map[tmp->position - max_map_x + 1].heuristique = tmp->heuristique + 2;
						}
					}
					break;
			}
		}
		tmp->visite = 1;
	} while (smallest_heur != 0xFFFF);
}

/*Choisi la case adjacente de moindre coût*/
void next_case(node node_map[max_map_y * max_map_x],int pos_x, int pos_y, int* delta_x, int* delta_y)
{
	unsigned short int smallest_heur = 0xFFFF;
    int tmp_x = 0;
    int tmp_y = 0;
    int lesser_cost_x = 0;
    int lesser_cost_y = 0;
	int j = 0;
	int i = 0;
	unsigned short int position = 0;

    for (j = -1; j <= 1; j++)
    {
        for (i = -1; i <= 1; i++)
        {
            tmp_x = pos_x + i;
            tmp_y = pos_y + j;
			position = tmp_x + tmp_y*max_map_x;

            if ((tmp_x >= 0) && (tmp_x < max_map_x) && (tmp_y >= 0) && (tmp_y < max_map_y) && ((tmp_x!=pos_x) || (tmp_y!=pos_y)))
            {
				if (node_map[position].heuristique < smallest_heur)
                {
					smallest_heur = node_map[position].heuristique;
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
int walk_the_path(unsigned char map_param[max_map_y][max_map_x], node node_map[max_map_y*max_map_x], int start_x, int start_y, int end_x, int end_y)
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
