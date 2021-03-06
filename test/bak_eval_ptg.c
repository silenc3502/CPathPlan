#include <math.h>
#include <time.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>

int n_sam = 10;
float sigma_s[3] = {10.0, 4.0, 2.0};
float sigma_d[3] = {1.0, 1.0, 1.0};
float sigma_t = 2.0;

float max_jerk = 10.0;
float max_accel = 10.0;

float expected_jerk_in_one_sec = 2.0;
float expected_acc_in_one_sec = 1.0;

float speed_limit = 30.0;
float vehicle_radius = 1.5;

typedef struct _vehicle
{
	float start_state[6];
	float s[3];
	float d[3];
	float state[6];
} vehicle;

void print_arr(float **data, int col, int row)
{
	int i, j;

	printf("data = \n");

	for(i = 0; i < col; i++)
	{
		for(j = 0; j < row; j++)
			printf("%4f ", data[i][j]);

		printf("\n");
	}
}

vehicle *init_vehicle(float *init_data)
{
	vehicle *tmp = (vehicle *)malloc(sizeof(vehicle));
	memset(tmp->start_state, 0x0, sizeof(vehicle));
	memcpy(tmp->start_state, init_data, 24);

	tmp->s[0] = init_data[0];
	tmp->s[1] = init_data[1];
	tmp->s[2] = init_data[2];

	tmp->s[3] = init_data[3];
	tmp->s[4] = init_data[4];
	tmp->s[5] = init_data[5];

	return tmp;
}

void *init_predict(vehicle *v)
{
	void *tmp = (void *)malloc(sizeof(vehicle));
	tmp = (void *)v;

	return tmp;
}

void state_in(float *ts, vehicle *v, float *delta, float t)
{
	ts[0] = (v->s[0] + (v->s[1] * t) + v->s[2] * t * t / 2.0) + delta[0];
	ts[1] = (v->s[1] + v->s[2] * t) + delta[1];
	ts[2] = v->s[2] + delta[2];

	ts[3] = (v->d[0] + (v->d[1] * t) + v->d[2] * t * t / 2.0) + delta[3];
	ts[4] = (v->d[1] + v->d[2] * t) + delta[4];
	ts[5] = v->d[2] + delta[5];
}

void gen_gaussian_rand_num_arr(float *data, float *mean, float *std, int len)
{
	int i;

	for(i = 0; i < len; i++)
	{
		float x1, x2, s = 0, tmp = 0;

		do
		{
			x1 = 2 * ((float) rand() / RAND_MAX) - 1;
			x2 = 2 * ((float) rand() / RAND_MAX) - 1;
			s = pow(x1, 2.0) + pow(x2, 2.0);
		} while(s >= 1 || s == 0);

		s = sqrt((-2 * log(s)) / s);
		tmp = x1 * s;
		tmp = (std[i] * tmp) + mean[i];

		data[i] = tmp;		
	}
}

#if 0
float gen_gaussian_rand_num(float mu, float sigma)
{
	float U1, U2, W, mult;
	static float X1, X2;
	static int call = 0;

	if (call == 1)
	{
		call = !call;
		return (mu + sigma * (float) X2);
	}

	do
	{
		U1 = -1 + ((float)rand () / RAND_MAX) * 2;
		U2 = -1 + ((float)rand () / RAND_MAX) * 2;
		W = pow (U1, 2) + pow (U2, 2);
	}
	while (W >= 1 || W == 0);

	mult = sqrt ((-2 * log (W)) / W);
	X1 = U1 * mult;
	X2 = U2 * mult;

	call = !call;

	return (mu + sigma * (float)X1);
}
#endif

float gen_gaussian_rand_num(float average, float stdev) {
	double v1, v2, s, temp;

	srand(time(NULL));

	do {
		v1 =  2 * ((float) rand() / RAND_MAX) - 1; // -1.0 ~ 1.0 까지의 값
		v2 =  2 * ((float) rand() / RAND_MAX) - 1; // -1.0 ~ 1.0 까지의 값
		s = v1 * v1 + v2 * v2;
	} while (s >= 1 || s == 0);

	s = sqrt( (-2 * log(s)) / s );

	temp = v1 * s;
	temp = (stdev * temp) + average;

	return temp;
}

void perturb_goal(float *perturb, float *gs, float *gd)
{
	int i;
	float new_s_goal[4] = {0};
	float new_d_goal[4] = {0};

#if 0
	for(i = 0; i < 3; i++)
	{
		new_s_goal[i] = gen_gaussian_rand_num(gs[i], sigma_s[i]);
		new_d_goal[i] = gen_gaussian_rand_num(gd[i], sigma_s[i]);
	}
#endif
	gen_gaussian_rand_num_arr(new_s_goal, gs, sigma_s, 3);
	gen_gaussian_rand_num_arr(new_d_goal, gd, sigma_s, 3);

	memcpy(perturb, new_s_goal, 12);
	memcpy(&perturb[3], new_d_goal, 12);
}

void ptg(float *ss, float *sd, float tv, float *delta, float T, void *pred)
{
	vehicle *target = (vehicle *)pred;

	float target_state[7] = {0};
	float goal_s[4] = {0};
	float goal_d[4] = {0};
	float perturb[7] = {0};

	float **goals = NULL;
	float **all_goals = NULL;

	float *trajectories = NULL;

	float timestep = 0.5;
	float t;

	int tmp = (n_sam + 1) * sizeof(float) * 7;
	int cnt = 0;
	int i;

	goals = (float **)malloc(sizeof(float *) * (n_sam  + 1));

	for(i = 0; i < n_sam + 1; i++)
		goals[i] = (float *)malloc(sizeof(float) * 7);
		//memset(&goals[i], 0x0, sizeof(float) * 7 * n_sam);

	all_goals = (float **)malloc(sizeof(float *) * (n_sam + 1) * (4 / timestep));

	for(i = 0; i < (n_sam + 1) * (4 / timestep); i++)
		all_goals[i] = (float *)malloc(sizeof(float) * 7);

	t = T - 4 * timestep;

	while(t <= T + 4 * timestep)
	{
		state_in(target_state, target, delta, t);
		memcpy(goal_s, target_state, 12);
		memcpy(goal_d, &target_state[3], 12);
		memcpy(&goals[0][0], target_state, 24);

		for(i = 0; i < n_sam; i++)
		{
			perturb_goal(perturb, goal_s, goal_d);
			memcpy(&goals[i + 1][0], perturb, 24);
			goals[i + 1][6] = t;
		}

		//print_arr(goals, n_sam + 1, 7);

		memcpy(&all_goals[(1 + n_sam) * cnt++], &goals[0], (n_sam + 1) * sizeof(float) * 7);
		print_arr(all_goals, (n_sam + 1), 7);

		t += timestep;
	}
}

int main(void)
{
	void *pred = NULL;
	vehicle *veh = NULL;
	float target = 0;
	//float init[7] = {0, 10, 0, 0, 0, 0};
	float init[7] = {0, 1, 2, 3, 4, 5};
	float delta[7] = {0, 0, 0, 0, 0, 0};
	float start_s[4] = {0, 0, 0};
	float start_d[4] = {0, 0, 0};
	float T = 5.0;

	srand(time(NULL));

	veh = init_vehicle(init);
	pred = init_predict(veh);
	ptg(start_s, start_d, target, delta, T, pred);
	
	return 0;
}
