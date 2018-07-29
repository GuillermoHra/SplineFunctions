#include <stdio.h>

typedef struct linearSpline{
	float xi; // x initial
	float xf; // x final
	float yi; // y initial
	float yf; // y final
	float X; // interpolation x coordinate
	float Y; // interpolation Y coordinate
	float theta_set_fsm;
	float res_factor;
	int time_state;
} LinearSpline;

static void initializeLinearSplineParams(LinearSpline *lSpline);
static void calcLinearSpline(LinearSpline *lSpline);

int main()
{
  LinearSpline linearSpline;
  initializeLinearSplineParams(&linearSpline);
	while(linearSpline.time_state < 100){
		calcLinearSpline(&linearSpline);
		printf("Theta_set: %f \n", linearSpline.Y); }

    return 0;
}

static void initializeLinearSplineParams(LinearSpline *lSpline){
	lSpline->time_state = 0;
	lSpline->res_factor = 100.0; // resolution factor
	lSpline->xi = 0.0; // caution: divide by zero
	lSpline->xf = lSpline->res_factor;
	lSpline->theta_set_fsm = 10.0; //user write
	lSpline->yi = 0.0; //actx->jointAngleDegrees
	lSpline->yf = lSpline->theta_set_fsm;
}

static void calcLinearSpline(LinearSpline *lSpline) {
	lSpline->Y = (((lSpline->yf - lSpline->yi) * ((float)lSpline->time_state - lSpline->xi)) / (lSpline->xf - lSpline->xi)) + lSpline->yi;
	if((lSpline->yi - lSpline->theta_set_fsm) > 0){
		if (lSpline->Y < lSpline->theta_set_fsm)
			lSpline->Y = lSpline->theta_set_fsm;
	}
	else{
		if(lSpline->Y > lSpline->theta_set_fsm)
			lSpline->Y = lSpline->theta_set_fsm;
	}

	lSpline->time_state++;
}
