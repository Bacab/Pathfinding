#include "astar_methods.h"

//cree un noeud
node*   create_node(node* parent, int x, int y, int goal_x, int goal_y)
{
    node* tmp = malloc(sizeof(node));
    tmp->x = x;
    tmp->y = y;
    tmp->parent = parent;
    tmp->g = parent->g+1;
    tmp->h = (int)(pow((y-goal_y),2)+pow((x-goal_x),2));
    tmp->f = tmp->g+tmp->h;
    return tmp;
}

//cree une liste avec les donnees d'un noeud
List* list_create(node* data_to_create)
{
    List *list_adress = malloc(sizeof(List));
    if (list_adress)
    {
        list_adress->data = data_to_create;
        list_adress->next = NULL;
    }
    return list_adress;
}

//ajoute un noeud a une liste
List* list_append(List* list_adress, node* data_to_add)
{
    List* tmp = list_adress;

    while (tmp->next!=NULL)
    {
        tmp = tmp->next;
    }
    tmp->next = list_create(data_to_add);

    return list_adress;
}

//supprime un noeud d'une liste
List* node_delete(List *list_adress, node data_to_delete)
{
    List* tmp = list_adress;
    List* previous_node = NULL;

    while(tmp)
    {
        //si c'est le noeud a supprimer
        if((tmp->data->x == data_to_delete.x)&&(tmp->data->y == data_to_delete.y)&&(tmp->data->f == data_to_delete.f))
        {
            //si tmp est le dernier element de la liste mais pas le premier
            if((tmp->next==NULL)&&(previous_node!=NULL))
            {
                //l'element precedent devient le dernier element
                previous_node->next = NULL;
                free(tmp);
                return list_adress;
            }
            //si tmp n'est ni le dernier element, ni le premier
            else if((tmp->next!=NULL)&&(previous_node!=NULL))
            {
                previous_node->next = tmp->next;
                free(tmp);
                return list_adress;
            }
            //si tmp est le premier et dernier element de la liste
            else if((tmp->next==NULL)&&(previous_node==NULL))
            {
                free(tmp);
                return NULL;
            }
            //si tmp est le premier mais pas le dernier element de la liste
            else if((tmp->next!=NULL)&&(previous_node==NULL))
            {
                previous_node = tmp;
                tmp = tmp->next;
                free(previous_node);
                return tmp;
            }
        }
        previous_node = tmp;
        tmp = tmp->next;
    }
    return list_adress;
}

void   delete_list(List *list_adress)
{
    List* to_delete;
    List* next_one = list_adress;

    while(next_one)
    {
        to_delete = next_one;
        next_one = next_one->next;
        free(to_delete->data);
        free(to_delete);
    }
}

void print_trajectory(unsigned short int map_param[10][9], int map_x, int map_y, int start_x, int start_y, int end_x, int end_y)
{
    if(donotusecls)
    {
        system("clear");
    }
    else
    {
        system("cls");
    }
    for (int j = 0; j < map_y; j++)
    {
        for (int i = 0; i < map_x; i++)
        {
            printf("\t");
            if(((i!=start_x)||(j!=start_y))&&((i!=end_x)||(j!=end_y)))
            {
                switch(map_param[j][i])
                {
                    case 180:
                        printf("*");
                        break;
                    case 255:
                        printf("||");
                        break;
                    case 190:
                        printf("**");
                        break;
                    default:
                        printf(" ");
                        break;
                }
                //printf("%d",map_param[j][i]);
            }
            else if((i==start_x)&&(j==start_y))
            {
                printf("R");//On part d'ici
            }
            else if((i==end_x)&&(j==end_y))
            {
                printf("G");//Il faut atteindre le point G ^_^
            }
        }
        printf("\n");
    }
}

int     make_path(unsigned short int map_param[10][9], int map_x, int map_y, int start_x, int start_y, int end_x, int end_y)
{
    node starting_node;//le noeud de depart, seul noeud avec f=g=h=0 et sans parent
    node* tmp;//le noeud a analyser
    node* successor;//un candidat pour le prochain noeud a analyser

    int tmp_x = 0;
    int tmp_y = 0;

    int mini_f = 1;//f minimal
    int add_to_open_list = 1;//indique si le noeud est un candidat viable
    int step = 0;//le nombre de cases parcourues pour atteindre le but

    //On construit le noeud de départ
    starting_node.x = start_x;
    starting_node.y = start_y;
    starting_node.parent = NULL;
    starting_node.f = 0;
    starting_node.g = 0;
    starting_node.h = 0;

    //On construit les listes chainees qui representeront notre choix de parcours
    List* open_list = list_create(&starting_node);//la liste des noeuds dont il faut analyser les alentours
    List* closed_list = NULL;//la liste des noeuds deja analyses
    List* next_list = NULL;//permet de parcourir les listes chainees

    while(open_list!=NULL)
    {
        next_list = open_list;
        //Parmi les noeuds a analyser on ne garde que le noeud de plus petit f
        mini_f = 9999;
        while(next_list)
        {
            if(next_list->data->f<mini_f)
            {
                tmp = next_list->data;
                mini_f = tmp->f;
            }
            next_list = next_list->next;
        }
        //Le noeud elu n'est plus a analyser donc on le supprime de la liste des noeuds a analyser
        open_list = node_delete(open_list,*tmp);

        //On regarde les huit cases adjacentes au noeud elu
        for (int j = -1; j <= 1; j++)
        {
            for (int i = -1; i <= 1; i++)
            {
                tmp_x = tmp->x + i;
                tmp_y = tmp->y + j;
                //si les coordonnees de cette case sont dans la carte et ne sont pas celles du noeud tmp
                if ((tmp_x >= 0) && (tmp_x < map_x) && (tmp_y >= 0) && (tmp_y < map_y) && ((tmp_x!=tmp->x) || (tmp_y!=tmp->y)))
                {
                    //si cette case ne represente pas un obstacle
                    if (map_param[tmp_y][tmp_x] != 255)
                    {
                        //On cree un noeud correspondant a cette case avec pour parent le noeud elu
                        successor = create_node(tmp, tmp_x, tmp_y, end_x, end_y);
                       //Si c'est notre but alors inutile de l'ajouter aux noeuds a analyser et on arrete de chercher
                        if((successor->x==end_x)&&(successor->y==end_y))
                        {
                                //On parcours le chemin calcule en modifiant la carte en conséquence
                                //pour pouvoir afficher un resultat lisible a l'ecran
                                node* next_node = successor;
                                step = 0;
                                while(next_node)
                                {
                                    map_param[next_node->y][next_node->x] = 180;
                                    step+=1;
                                    next_node = next_node->parent;
                                }
                                delete_list(closed_list);
                                delete_list(open_list);
                                return step;
                        }
                        else
                        {
                            //si on a deja ajoute ce noeud a la liste des noeuds interessants
                            //mais avec un chemin depuis le depart plus court, alors on n'ajoute pas ce noeud
                            next_list = open_list;
                            while(next_list)
                            {
                                if((next_list->data->x==successor->x)&&(next_list->data->y==successor->y)&&(next_list->data->f<=successor->f))
                                {
                                    add_to_open_list = 0;
                                }
                                next_list = next_list->next;
                            }
                            //si on a deja ajoute ce noeud a la liste des noeuds analyses
                            //mais avec un chemin depuis le depart plus court, alors on n'ajoute pas ce noeud
                            next_list = closed_list;
                            while(next_list)
                            {
                                if((next_list->data->x==successor->x)&&(next_list->data->y==successor->y)&&(next_list->data->f<=successor->f))
                                {
                                    add_to_open_list = 0;
                                }
                                next_list = next_list->next;
                            }
                            //sinon on ajoute ce noeud a la liste des noeuds a analyser
                            if(add_to_open_list==1)
                            {
                                if(open_list==NULL)
                                {
                                    open_list = list_create(successor);
                                }
                                else
                                {
                                    open_list = list_append(open_list, successor);
                                }
                            }
                            else
                            {
                                //On remet a 1 notre indicateur et on supprime le noeud car il n'est pas interessant
                                add_to_open_list=1;
                                free(successor);
                            }
                        }
                    }
                }
            }
        }
        //On ajoute le noeud a la liste des noeuds que l'on a analyses
        if(closed_list==NULL)
        {
            closed_list = list_create(tmp);
        }
        else
        {
            closed_list = list_append(closed_list,tmp);
        }
    }
    delete_list(closed_list);
    delete_list(open_list);
    return -1;
}

