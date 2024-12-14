#include <stdint.h>


void shaft_step_motor(uint16_t step1,uint16_t cycle_time1);


void z_axis_step_motor(uint16_t step2,uint16_t cycle_time2);


void z_axis_UP(uint16_t step2,uint16_t cycle_time2);
void z_axis_DOWN(uint16_t step2,uint16_t cycle_time2);


void down_part_UP(uint16_t step1,uint16_t cycle_time1);
void down_part_DOWN(uint16_t step1,uint16_t cycle_time1);


void down_part_UP_accel(uint16_t step1) ;
void down_part_DOWN_accel(uint16_t step1) ;
