#include <math.h>
#include <time.h>
#include <stdio.h>
#include <unistd.h>
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

void print_single_arr(float *data, int len)
{
	int i;

	printf("data = \n");

	for(i = 0; i < len; i++)
		printf("%4f\n", data[i]);
}

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

void print_tri_arr(float ***data, int dim, int col, int row)
{
	int i, j, k;

	printf("data = \n");

	for(i = 0; i < dim; i++)
	{
		for(j = 0; j < col; j++)
		{
			for(k = 0; k < row; k++)
				printf("%4f ", data[i][j][k]);

			printf("\n");
		}

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

#if 0
void get_s_d_goals(float ***all_goals, float *s_coeff, float *d_coeff, int cnt)
{
	int i, j, k = 0, tmp = 0;
	int col = (n_sam + 1) * cnt;

	float **s_goal = NULL;
	float **d_goal = NULL;
	float *t = NULL;

	printf("col = %d\n", col);

	s_goal = (float **)malloc(sizeof(float *) * col);
	d_goal = (float **)malloc(sizeof(float *) * col);
	t = (float *)malloc(sizeof(float) * col);

	for(i = 0; i < col; i++)
	{
		s_goal[i] = (float *)malloc(sizeof(float) * 3);
		d_goal[i] = (float *)malloc(sizeof(float) * 3);

		for(j = 0; j < 3; j++)
		{
			s_goal[i][j] = all_goals[tmp][k][j];
			d_goal[i][j] = all_goals[tmp][k][j + 3];
		}

		t[i] = all_goals[tmp][k][6];

		k++;

		if(k == n_sam + 1)
		{
			tmp++;
			k = 0;
		}
	}

	//print_arr(s_goal, col, 3);
	//print_arr(d_goal, col, 3);
	//print_single_arr(t, col);

	jerk_min_trajectory();
}
#endif

void get_s_d_goals(float ***all_goals, float **s_goal, float **d_goal, float *t, int cnt)
{
	int i, j, k = 0, tmp = 0;
	int col = (n_sam + 1) * cnt;

	//float **s_goal = NULL;
	//float **d_goal = NULL;
	//float *t = NULL;

	printf("col = %d\n", col);

	for(i = 0; i < col; i++)
	{
		for(j = 0; j < 3; j++)
		{
			s_goal[i][j] = all_goals[tmp][k][j];
			d_goal[i][j] = all_goals[tmp][k][j + 3];
		}

		t[i] = all_goals[tmp][k][6];

		k++;

		if(k == n_sam + 1)
		{
			tmp++;
			k = 0;
		}
	}

	print_arr(s_goal, col, 3);
	//print_arr(d_goal, col, 3);
	//print_single_arr(t, col);

	//jerk_min_trajectory();
}

void jerk_min_trajectory(float **trajectory, float *sstart, float *dstart,
			 float **send, float **dend, float *t, int len)
{
	float a_s0[len];
	float a_d0[len];
	float a_s1[len];
	float a_d1[len];
	float a_s2[len];
	float a_d2[len];

	float c_s0[len];
	float c_d0[len];
	float c_s1[len];
	float c_d1[len];
	float c_s2[len];
	float c_d2[len];

	float ***A_s = NULL;
	float ***A_d = NULL;
	float **B_s = NULL;
	float **B_d = NULL;

	int i, j, k;

	A_s = (float ***)malloc(sizeof(float **) * len);
	B_s = (float **)malloc(sizeof(float *) * len);

	A_d = (float ***)malloc(sizeof(float **) * len);
	B_d = (float **)malloc(sizeof(float *) * len);

	for(i = 0; i < len; i++)
	{
		a_s0[i] = sstart[0];
		a_d0[i] = dstart[0];
		a_s1[i] = sstart[1];
		a_d1[i] = dstart[1];
		a_s2[i] = sstart[2];
		a_d2[i] = dstart[2];

		c_s0[i] = a_s0[i] + a_s1[i] * t[i] + a_s2[i] * t[i] * t[i];
		c_d0[i] = a_d0[i] + a_d1[i] * t[i] + a_d2[i] * t[i] * t[i];

		c_s1[i] = a_s1[i] + 2 * a_s2[i] * t[i];
		c_d1[i] = a_d1[i] + 2 * a_d2[i] * t[i];

		c_s2[i] = 2 * a_s2[i];
		c_d2[i] = 2 * a_d2[i];

		A_s[i] = (float **)malloc(sizeof(float *) * 3);
		B_s[i] = (float *)malloc(sizeof(float) * 3);

		A_d[i] = (float **)malloc(sizeof(float *) * 3);
		B_d[i] = (float *)malloc(sizeof(float) * 3);

		for(j = 0; j < 3; j++)
		{
			A_s[i][j] = (float *)malloc(sizeof(float) * 3);
			A_d[i][j] = (float *)malloc(sizeof(float) * 3);
		}

		A_s[i][0][0] = pow(t[i], 3);
		A_s[i][0][1] = pow(t[i], 4);
		A_s[i][0][2] = pow(t[i], 5);
		A_s[i][1][0] = 3 * pow(t[i], 2);
		A_s[i][1][1] = 4 * pow(t[i], 3);
		A_s[i][1][2] = 5 * pow(t[i], 4);
		A_s[i][2][0] = 6 * t[i];
		A_s[i][2][1] = 12 * pow(t[i], 2);
		A_s[i][2][2] = 20 * pow(t[i], 3);

		B_s[i][0] = send[i][0] - c_s0[i];
		B_s[i][1] = send[i][1] - c_s1[i];
		B_s[i][2] = send[i][2] - c_s2[i];

		A_d[i][0][0] = pow(t[i], 3);
		A_d[i][0][1] = pow(t[i], 4);
		A_d[i][0][2] = pow(t[i], 5);
		A_d[i][1][0] = 3 * pow(t[i], 2);
		A_d[i][1][1] = 4 * pow(t[i], 3);
		A_d[i][1][2] = 5 * pow(t[i], 4);
		A_d[i][2][0] = 6 * t[i];
		A_d[i][2][1] = 12 * pow(t[i], 2);
		A_d[i][2][2] = 20 * pow(t[i], 3);

		B_d[i][0] = dend[i][0] - c_d0[i];
		B_d[i][1] = dend[i][1] - c_d1[i];
		B_d[i][2] = dend[i][2] - c_d2[i];
	}
}

#if 0
void mov_arr_to_arr(float ***all_goals, float (*goals)[7], int cnt, int col, int row)
{
	int i, j;

	for(i = 0; i < col; i++)
		for(j = 0; j < row; j++)
			all_goals[cnt][i][j] = goals[i][j];
}
#endif

void mov_arr_to_arr(float ***all_goals, float **goals, int cnt, int col, int row)
{
	int i, j;

	for(i = 0; i < col; i++)
		for(j = 0; j < row; j++)
			all_goals[cnt][i][j] = goals[i][j];
}

void ptg(float *ss, float *sd, float tv, float *delta, float T, void *pred)
{
	vehicle *target = (vehicle *)pred;

	float target_state[7] = {0};
	float goal_s[4] = {0};
	float goal_d[4] = {0};
	float perturb[7] = {0};

#if 1
	float **goals = NULL;
#endif

#if 0
	float goals[n_sam + 1][7];
#endif
	float ***all_goals = NULL;

	float **trajectories = NULL;

	float **s_goal = NULL;
	float **d_goal = NULL;
	float *time = NULL;

	float timestep = 0.5;
	float t;

	int tmp = (n_sam + 1) * sizeof(float) * 7;
	int col, cnt = 0;
	int i, j;

	//printf("sizeof(goals[n_sam + 1][7]) = %lu\n", sizeof(goals));
	printf("(n_sam + 1) * sizeof(float) * 7 = %lu\n", (n_sam + 1) * sizeof(float) * 7);


#if 1
	goals = (float **)malloc(sizeof(float *) * (n_sam  + 1));

	for(i = 0; i < n_sam + 1; i++)
		goals[i] = (float *)malloc(sizeof(float) * 7);
		//memset(&goals[i], 0x0, sizeof(float) * 7 * n_sam);
#endif

#if 0
	all_goals = (float **)malloc(sizeof(float *) * (n_sam + 1) * (4 / timestep));

	for(i = 0; i < (n_sam + 1) * (4 / timestep); i++)
		all_goals[i] = (float *)malloc(sizeof(float) * 7);
#endif

	all_goals = (float ***)malloc(sizeof(float **) * ((4 / timestep) + 1));

	for(i = 0; i < ((4 / timestep) + 1); i++)
	{
		all_goals[i] = (float **)malloc(sizeof(float *) * (n_sam + 1));

		for(j = 0; j < n_sam + 1; j++)
			all_goals[i][j] = (float *)malloc(sizeof(float) * 7);
	}

	t = T - 4 * timestep;

	while(t <= T + 4 * timestep)
	{
		state_in(target_state, target, delta, t);
		memcpy(goal_s, target_state, 12);
		memcpy(goal_d, &target_state[3], 12);
		memcpy(&goals[0][0], target_state, 24);
		goals[0][6] = t;

		for(i = 0; i < n_sam; i++)
		{
			perturb_goal(perturb, goal_s, goal_d);
			memcpy(&goals[i + 1][0], perturb, 24);
			goals[i + 1][6] = t;
		}

		//print_arr(goals, n_sam + 1, 7);

		//memcpy(&all_goals[(1 + n_sam) * cnt++], &goals[0], (n_sam + 1) * sizeof(float) * 7);
		//print_arr(goals, (n_sam + 1), 7);

		//printf("t = %f\n", t);
		//memcpy(&all_goals[cnt++][0][0], &goals[0][0], (n_sam + 1) * sizeof(float *) * 7);
		//memcpy(&all_goals[cnt++], goals, (n_sam + 1) * sizeof(float) * 7);
		mov_arr_to_arr(all_goals, goals, cnt, n_sam + 1, 7);
		//memmove(&all_goals[cnt++][0][0], &goals[0][0], (n_sam + 1) * sizeof(float) * 7);
		cnt++;

		t += timestep;
	}

	print_tri_arr(all_goals, (4 / timestep) + 1, (n_sam + 1), 7);

	time = (float *)malloc(sizeof(float) * ((4 / timestep) + 1) * (n_sam + 1));

    col = (n_sam + 1) * cnt;

    s_goal = (float **)malloc(sizeof(float *) * col);
    d_goal = (float **)malloc(sizeof(float *) * col);

    for(i = 0; i < col; i++)
    {   
        s_goal[i] = (float *)malloc(sizeof(float) * 3); 
        d_goal[i] = (float *)malloc(sizeof(float) * 3);
	}

	get_s_d_goals(all_goals, s_goal, d_goal, time, cnt);
	jerk_min_trajectory(trajectories, ss, sd, s_goal, d_goal, time, (n_sam + 1) * cnt);
}

int main(void)
{
	void *pred = NULL;
	vehicle *veh = NULL;
	float target = 0;
	//float init[7] = {0, 10, 0, 0, 0, 0};
	float init[7] = {0, 1, 2, 3, 4, 5};
	float delta[7] = {0, 0, 0, 0, 0, 0};
	float start_s[4] = {10, 10, 0};
	float start_d[4] = {4, 0, 0};
	float T = 5.0;

	srand(time(NULL));

	veh = init_vehicle(init);
	pred = init_predict(veh);
	ptg(start_s, start_d, target, delta, T, pred);
	
	return 0;
}
