#include <stdio.h>
#include <string.h>
#include <stdlib.h>

typedef struct _vehicle
{
	float start_state[6];
	float s[3];
	float d[3];
} vehicle;

vehicle *init_vehicle(void)
{
	vehicle *tmp = (vehicle *)malloc(sizeof(vehicle));
	memset(tmp->start_state, 0x0, sizeof(vehicle));

	return tmp;
}

void *init_predict(vehicle *v)
{
	void *tmp = (void *)malloc(sizeof(vehicle));
	tmp = (void *)v;

	return tmp;
}

void ptg(float *ss, float *sd, float target, float *delta, float T, void *pred)
{
	vehicle *target = (vehicle *)pred;
	float *all_goals = NULL;
	float timestep = 0.5;
	float t;

	t = T - 4 * timestep;

	while(t <= T + 4 * timestep)
	{
		
	}
}

int main(void)
{
	void *pred = NULL;
	vehicle *veh = NULL;
	float target = 0;
	float delta[7] = {0, 0, 0, 0, 0, 0};
	float start_s[4] = {0, 0, 0};
	float start_d[4] = {0, 0, 0};
	float T = 5.0;

	veh = init_vehicle();
	pred = init_predict(veh);
	ptg(start_s, start_d, target, delta, T, pred);
	
	return 0;
}
