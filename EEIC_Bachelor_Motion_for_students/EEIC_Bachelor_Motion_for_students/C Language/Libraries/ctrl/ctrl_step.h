#ifndef CTRL_STEP
#define CTRL_STEP
void ctrl_step_input(double A, double steptime, double t_step, double *out); 
void ctrl_step_input_with_end(double A, double begintime, double finitime, double t_step, double *out);
#endif