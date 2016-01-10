//Pathfinding v0.1
//Created by Florian Loiseau

#define map_x 10
#define map_y 10

int robot_x=1;
int robot_y=1;

int goal_x=7;
int goal_y=4;

int next_x=0;
int next_y=0;

bool displayed=0;

short unsigned int home_map[map_x][map_y]={{255, 255, 255, 255, 255, 255, 255, 255, 255, 255},
                                           {255, 000, 000, 000, 000, 000, 000, 000, 000, 255},
                                           {255, 000, 255, 255, 255, 255, 255, 000, 000, 255},
                                           {255, 000, 000, 000, 000, 000, 000, 000, 000, 255},
                                           {255, 000, 000, 000, 000, 000, 000, 000, 000, 255},
                                           {255, 000, 000, 000, 000, 000, 000, 000, 000, 255},
                                           {255, 000, 000, 000, 000, 000, 000, 000, 000, 255},
                                           {255, 000, 000, 000, 000, 000, 000, 000, 000, 255},
                                           {255, 000, 000, 000, 000, 000, 000, 000, 000, 255},
                                           {255, 255, 255, 255, 255, 255, 255, 255, 255, 255}};


void setup() {
  reinit_map(home_map,goal_x,goal_y);
  Serial.begin(9600);
}

void loop() {
  if((robot_x!=goal_x)||(robot_y!=goal_y))
  {
      next_x=0;
      next_y=0;
      next_case(home_map,robot_x,robot_y,&next_x,&next_y);
      home_map[robot_x][robot_y]=180;
      robot_x+=next_x;
      robot_y+=next_y;
  }else if(displayed==0)
  {
    debug(home_map);
    displayed=1;
  }
}



unsigned short int get_distance(int cellule_x,int cellule_y,int goal_x,int goal_y)
{
  return (abs(cellule_x-goal_x)+abs(cellule_y-goal_y)); 
}

void reinit_map(unsigned short int map_to_reinit[map_x][map_y],int goal_x,int goal_y)
{
  for (int i=0;i<map_x;i++)
  {
      for (int j=0;j<map_y;j++)
      {
        if(map_to_reinit[i][j]!=255)
          map_to_reinit[i][j]=get_distance(i,j,goal_x,goal_y);
      }
  }
  map_to_reinit[goal_x][goal_y]=0;
}

void next_case(unsigned short int map_to_reinit[map_x][map_y],int robot_x, int robot_y,int * next_x, int * next_y)
{
  unsigned short int lesser_cost=255;
  int lesser_cost_x=0;
  int lesser_cost_y=0;
  
  for(int i=-1;i<=1;i++)
  {
    for(int j=-1;j<=1;j++)
    {
      if(((robot_x+i)>=0)&&((robot_x+i)<=map_x)&&((robot_y+j)>=0)&&((robot_y+j)<=map_y))
      {
        if(map_to_reinit[robot_x+i][robot_y+j]<lesser_cost)
        {
          lesser_cost=map_to_reinit[robot_x+i][robot_y+j];
          lesser_cost_x=i;
          lesser_cost_y=j;
        }
      }
    }
  }
  *next_x=lesser_cost_x;
  *next_y=lesser_cost_y;
}

void debug(unsigned short int map_to_display[map_x][map_y])
{
    for (int j=0;j<map_y;j++)
  {
      for (int i=0;i<map_x;i++)
      {
        Serial.print(map_to_display[i][j]);
        Serial.print(' ');
      }
      Serial.print("\n");
  }
}

