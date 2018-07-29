#include <stdio.h>
#include <math.h>

typedef struct cubicSpline{
	float xi; // x initial
	float x_int; // x intermediate
	float xf; // x final
	float yi; // y initial
	float y_int; // y intermediate
	float yf; // y final
	float X; // interpolation x coordinate
	float Y; // interpolation Y coordinate
	float theta_set_fsm;
	float theta_set_fsm_int;
	float res_factor; // resolution factor
	int time_state;
	float a1;
	float a2;
	float b1;
	float b2;
} CubicSpline;

float k1 = 1.0;
float k2 = 0.0;
float b = 0.0;
float theta_d = 0.0;
static float theta = 14.0;

static void solveTridiagonalMatrix(CubicSpline *cSpline);
static void calcCubicSpline(CubicSpline *cSpline);
CubicSpline cubicSpline;

int main() {
	float joint_angle = 14.0;
	// Initialization
	cubicSpline.theta_set_fsm = -10.0;
	cubicSpline.theta_set_fsm_int = joint_angle-((joint_angle-cubicSpline.theta_set_fsm)/4);
	cubicSpline.res_factor = 30.0;
	cubicSpline.x_int = cubicSpline.res_factor * 0.7;
  cubicSpline.y_int = cubicSpline.theta_set_fsm_int;
	cubicSpline.xi = 0.0;
  cubicSpline.yi = joint_angle; //joint_angle
	cubicSpline.xf = cubicSpline.res_factor;
	cubicSpline.yf = cubicSpline.theta_set_fsm;
	// End initialization

	solveTridiagonalMatrix(&cubicSpline);

	// Initialization impedance controller parameters
cubicSpline.time_state = 0;

while(cubicSpline.time_state < 31){
	calcCubicSpline(&cubicSpline);
	cubicSpline.time_state++;
}

  return 0;
}

static void solveTridiagonalMatrix(CubicSpline *cSpline){
	float B[3], A[2], C[2], r[3];
	float x[3];
	float y[3];
	x[0] = cSpline->xi;
	x[1] = cSpline->x_int;
	x[2] = cSpline->xf;
	y[0] = cSpline->yi;
	y[1] = cSpline->y_int;
	y[2] = cSpline->yf;

	B[0] = 2.0 / (x[1] - x[0]);
	B[1] = 2.0 * ((1/(x[1]-x[0])) + (1/(x[2]-x[1])));
	B[2] = 2.0 / (x[2]-x[1]);
	A[0] = 1.0 / (x[1]-x[0]);
	A[1] = 1.0 / (x[2]-x[1]);
	C[0] = 1.0 / (x[1]-x[0]);
	C[1] = 1.0 / (x[2]-x[1]);
	r[0] = 3.0 * ((y[1]-y[0])/(pow(x[1]-x[0],2)));
	r[1] = 3.0 * (((y[1]-y[0])/(pow(x[1]-x[0],2))) + ((y[2]-y[1])/(pow(x[2]-x[1],2))));
	r[2] = 3.0 * ((y[2]-y[1])/(pow(x[2]-x[1],2)));

	float e[3], f[3], g[2];
	e[0] = 0;
	e[1] = A[0];
	e[2] = A[1];
	for(int i=0; i<3; i++){
		f[i] = B[i];
	}
	g[0] = C[0];
	g[1] = C[1];

	int n = 3; // f vector length
	// Forward elimination
	float factor;
	for(int i = 1; i < n; i++){
			factor = e[i] / f[i-1];
			f[i] = f[i] - (factor * g[i-1]);
			r[i] = r[i] - (factor * r[i-1]);
	}
  // Back substitution
	float k[3];
	k[n-1] = r[n-1] / f[n-1];
	for(int i = 1; i >= 0; i--){
		k[i] = (r[i] - (g[i] * k[i+1])) / f[i];
	}

	// ai and bi computation
	float a1, a2, b1, b2;
	a1 = k[0]*(x[1]-x[0]) - (y[1]-y[0]);
	a2 = k[1]*(x[2]-x[1]) - (y[2]-y[1]);
	b1 = -1.0*k[1]*(x[1]-x[0]) + (y[1]-y[0]);
	b2 = -1.0*k[2]*(x[2]-x[1]) + (y[2]-y[1]);
	cSpline->a1 = a1;
	cSpline->a2 = a2;
	cSpline->b1 = b1;
	cSpline->b2 = b2;
}

static void calcCubicSpline(CubicSpline *cSpline){
	float t;
	float q[2];
	float x[3];
	float y[3];
	float theta_set;
	float tor_d;
	x[0] = cSpline->xi;
	x[1] = cSpline->x_int;
	x[2] = cSpline->xf;
	y[0] = cSpline->yi;
	y[1] = cSpline->y_int;
	y[2] = cSpline->yf;

	t = (cubicSpline.time_state - x[0]) / (x[1]-x[0]);
	q[0] = (1-t)*y[0] + t*y[1] + (t*(1-t)*(cSpline->a1*(1-t)+(cSpline->b1*t)));

	t = (cubicSpline.time_state - x[1]) / (x[2]-x[1]);
	q[1] = (1-t)*y[1] + t*y[2] + (t*(1-t)*(cSpline->a2*(1-t)+(cSpline->b2*t)));

  if (theta > cSpline->theta_set_fsm_int)
  	theta_set = q[0];
  else theta_set = q[1];

	printf("Theta_set: %f \n", theta_set);
	tor_d = k1 * (theta_set - theta);
	if (theta > cSpline->theta_set_fsm)
        theta = theta + tor_d;
    else theta = theta;
}
