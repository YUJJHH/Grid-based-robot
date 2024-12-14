#include <stdint.h>



void position_pid_x(void);
void position_pid_y(void);

void v_t_graph(float targetS);

void dc_motor_pid_X(float input_encoder);
void dc_motor_pid_Y(float input_encoder) ;

void dc_motor_pid_X_with_photosensor1(uint32_t input_encoder);  //x축 pid
void dc_motor_pid_Y_with_photosensor2(uint32_t input_encoder);  //y축 pid


void dc_motor_RPM(void);

void v_t_graph_DIR_GO_X(float targetS);
void v_t_graph_DIR_BACK_X(float targetS);

void v_t_graph_DIR_GO_Y(float targetS_Y);
void v_t_graph_DIR_BACK_Y(float targetS_Y);

void VT_control_X(void);
void VT_control_Y(void);
