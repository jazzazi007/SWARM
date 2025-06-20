#include "../include/swarm.h"

void safe_lioter(sts *sts)
{
    int init_loiter = 150;
    int distance = 30;
    int lioter_0 = (UAV_COUNT-1)/2;
    sts->loiter[0] = 150 + (distance * lioter_0);
    int turn = 0;
    for (int i = 1; i < UAV_COUNT; i++)
    {
            sts->loiter[i] = init_loiter + (distance * turn);
            if (i+1 > UAV_COUNT/2)
            {
                turn+=2;
            }
            else
            {
                turn+=1;
            }
    }

}