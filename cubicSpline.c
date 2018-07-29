#include <stdio.h>
#include <math.h>
// Cubic spline - S curve
typedef struct cubicSpline{
	float xi_1; // x initial
	float x_int_1; // x intermediate
	float xf_1; // x final
	float yi_1; // y initial
	float y_int_1; // y intermediate
	float yf_1; // y final
	float X; // interpolation x coordinate
	float Y; // interpolation Y coordinate
	float theta_set_fsm;
	float res_factor; // resolution factor
	int time_state;
	float a1_1; // coefficients for cubic functions
	float a2_1;
	float b1_1;
	float b2_1;
	float xi_2; // x initial
	float x_int_2; // x intermediate
	float xf_2; // x final
	float yi_2; // y initial
	float y_int_2; // y intermediate
	float yf_2; // y final
	float a1_2; // coefficients for cubic functions
	float a2_2;
	float b1_2;
	float b2_2;
} CubicSpline;
static void initializeCubicSplineParams(CubicSpline *cSpline);
static void solveTridiagonalMatrix(CubicSpline *cSpline);
static void calcCubicSpline(CubicSpline *cSpline);
CubicSpline cubicSpline;

int main() {
	initializeCubicSplineParams(&cubicSpline);
	solveTridiagonalMatrix(&cubicSpline);

	while(cubicSpline.time_state < 100){
		calcCubicSpline(&cubicSpline);
		printf("Theta_set: %f \n", cubicSpline.Y); }

  return 0;
}

static void initializeCubicSplineParams(CubicSpline *cSpline){
	cSpline->time_state = 0;
	cSpline->res_factor = 100.0;
	cSpline->theta_set_fsm = 10.0; //user write
	cSpline->xi_1 = 0.0;
	cSpline->x_int_1 = (cSpline->res_factor/2.0)*.4;
	cSpline->xf_1 = cSpline->res_factor/2.0;
	cSpline->yi_1 = 0.0; // actx->jointAngleDegrees
	cSpline->yf_1 = cSpline->yi_1 + ((cSpline->theta_set_fsm - cSpline->yi_1)/2.0);
	cSpline->y_int_1 = cSpline->yi_1 - ((cSpline->yi_1 - cSpline->yf_1) * .15);
	cSpline->xi_2 = cSpline->res_factor/2.0;
	cSpline->x_int_2 = (cSpline->res_factor-(cSpline->res_factor/2.0))*.6+(cSpline->res_factor/2.0);
	cSpline->xf_2 = cSpline->res_factor;
	cSpline->yi_2 = cSpline->yi_1 + ((cSpline->theta_set_fsm - cSpline->yi_1)/2.0);
	cSpline->yf_2 = cSpline->theta_set_fsm;
	cSpline->y_int_2 = cSpline->yf_2 + ((cSpline->yi_2 - cSpline->yf_2) * .15);
}

static void solveTridiagonalMatrix(CubicSpline *cSpline){
	float B[3], A[2], C[2], r[3];
	float e[3], f[3], g[2];
	float x[3];
	float y[3];
	int n = 3; // f vector length
	float factor;
	float k[3];
	float a1, a2, b1, b2;
	x[0] = cSpline->xi_1;
	x[1] = cSpline->x_int_1;
	x[2] = cSpline->xf_1;
	y[0] = cSpline->yi_1;
	y[1] = cSpline->y_int_1;
	y[2] = cSpline->yf_1;

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

	e[0] = 0;
	e[1] = A[0];
	e[2] = A[1];
	for(int i=0; i<3; i++){
		f[i] = B[i];
	}
	g[0] = C[0];
	g[1] = C[1];
	// Forward elimination
	for(int i = 1; i < n; i++){
			factor = e[i] / f[i-1];
			f[i] = f[i] - (factor * g[i-1]);
			r[i] = r[i] - (factor * r[i-1]);
	}
  // Back substitution
	k[n-1] = r[n-1] / f[n-1];
	for(int i = 1; i >= 0; i--){
		k[i] = (r[i] - (g[i] * k[i+1])) / f[i];
	}
	// ai and bi computation
	a1 = k[0]*(x[1]-x[0]) - (y[1]-y[0]);
	a2 = k[1]*(x[2]-x[1]) - (y[2]-y[1]);
	b1 = -1.0*k[1]*(x[1]-x[0]) + (y[1]-y[0]);
	b2 = -1.0*k[2]*(x[2]-x[1]) + (y[2]-y[1]);
	cSpline->a1_1 = a1;
	cSpline->a2_1 = a2;
	cSpline->b1_1 = b1;
	cSpline->b2_1 = b2;
	// -----S curve complementary trajectory-----
	x[0] = cSpline->xi_2;
	x[1] = cSpline->x_int_2;
	x[2] = cSpline->xf_2;
	y[0] = cSpline->yi_2;
	y[1] = cSpline->y_int_2;
	y[2] = cSpline->yf_2;

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

	e[0] = 0;
	e[1] = A[0];
	e[2] = A[1];
	for(int i=0; i<3; i++){
		f[i] = B[i];
	}
	g[0] = C[0];
	g[1] = C[1];
	// Forward elimination
	for(int i = 1; i < n; i++){
			factor = e[i] / f[i-1];
			f[i] = f[i] - (factor * g[i-1]);
			r[i] = r[i] - (factor * r[i-1]);
	}
  // Back substitution
	k[n-1] = r[n-1] / f[n-1];
	for(int i = 1; i >= 0; i--){
		k[i] = (r[i] - (g[i] * k[i+1])) / f[i];
	}
	// ai and bi computation
	a1 = k[0]*(x[1]-x[0]) - (y[1]-y[0]);
	a2 = k[1]*(x[2]-x[1]) - (y[2]-y[1]);
	b1 = -1.0*k[1]*(x[1]-x[0]) + (y[1]-y[0]);
	b2 = -1.0*k[2]*(x[2]-x[1]) + (y[2]-y[1]);
	cSpline->a1_2 = a1;
	cSpline->a2_2 = a2;
	cSpline->b1_2 = b1;
	cSpline->b2_2 = b2;
}

static void calcCubicSpline(CubicSpline *cSpline){
	float t;
	float q[2];
	float q2[2];
	float x[3];
	float x2[3];
	float y[3];
	float y2[3];
	x[0] = cSpline->xi_1;
	x[1] = cSpline->x_int_1;
	x[2] = cSpline->xf_1;
	y[0] = cSpline->yi_1;
	y[1] = cSpline->y_int_1;
	y[2] = cSpline->yf_1;
	x2[0] = cSpline->xi_2;
	x2[1] = cSpline->x_int_2;
	x2[2] = cSpline->xf_2;
	y2[0] = cSpline->yi_2;
	y2[1] = cSpline->y_int_2;
	y2[2] = cSpline->yf_2;

	if (cSpline->time_state <= (cSpline->res_factor/2.0)){
		t = ((float)cSpline->time_state - x[0]) / (x[1]-x[0]);
		q[0] = (1-t)*y[0] + t*y[1] + (t*(1-t)*(cSpline->a1_1*(1-t)+(cSpline->b1_1*t)));
		t = ((float)cSpline->time_state - x[1]) / (x[2]-x[1]);
		q[1] = (1-t)*y[1] + t*y[2] + (t*(1-t)*(cSpline->a2_1*(1-t)+(cSpline->b2_1*t)));
		if(cSpline->time_state <= ((cSpline->res_factor/2.0)*.4))
			cSpline->Y = q[0];
			else cSpline->Y = q[1];
	}
	else{
		t = ((float)cSpline->time_state - x2[0]) / (x2[1]-x2[0]);
		q2[0] = (1-t)*y2[0] + t*y2[1] + (t*(1-t)*(cSpline->a1_2*(1-t)+(cSpline->b1_2*t)));
		t = ((float)cSpline->time_state - x2[1]) / (x2[2]-x2[1]);
		q2[1] = (1-t)*y2[1] + t*y2[2] + (t*(1-t)*(cSpline->a2_2*(1-t)+(cSpline->b2_2*t)));
		if(cSpline->time_state <= ((cSpline->res_factor-(cSpline->res_factor/2.0))*.6+(cSpline->res_factor/2.0)))
			cSpline->Y = q2[0];
			else cSpline->Y = q2[1];
	}

	if((cSpline->yi_1 - cSpline->theta_set_fsm) > 0){
		if(cSpline->Y < cSpline->theta_set_fsm)
			cSpline->Y = cSpline->theta_set_fsm;
	}
	else{
		if(cSpline->Y > cSpline->theta_set_fsm)
			cSpline->Y = cSpline->theta_set_fsm;
	}

	cSpline->time_state++;
}
