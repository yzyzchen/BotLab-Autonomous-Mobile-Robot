#include "controller.h"
#include <rc/math/filter.h>

// Values to clamp PWM signal between min and max
const float PWM_MIN = 0.1;
const float PWM_MAX = 0.9;


int mbot_init_ctlr(mbot_ctlr_cfg_t ctlr_cfg){
    left_wheel_pid = rc_filter_empty();
    right_wheel_pid = rc_filter_empty();
    // back_wheel_pid = rc_filter_empty(); // not using for diff drive
    mbot_vx_pid = rc_filter_empty();
    mbot_vy_pid = rc_filter_empty(); // should be 0. for diff drive
    mbot_wz_pid = rc_filter_empty();
    low_pass_filter = rc_filter_empty();

    // Initialize PID function from robot control (rc) library
    rc_filter_pid(&left_wheel_pid, ctlr_cfg.left.kp, ctlr_cfg.left.ki, ctlr_cfg.left.kd, ctlr_cfg.left.Tf, MAIN_LOOP_PERIOD);
    rc_filter_pid(&right_wheel_pid, ctlr_cfg.right.kp, ctlr_cfg.right.ki, ctlr_cfg.right.kd, ctlr_cfg.right.Tf, MAIN_LOOP_PERIOD);
    // rc_filter_pid(&back_wheel_pid, ctlr_cfg.back.kp, ctlr_cfg.back.ki, ctlr_cfg.back.kd, ctlr_cfg.back.Tf, MAIN_LOOP_PERIOD);
    rc_filter_pid(&mbot_vx_pid, ctlr_cfg.vx.kp, ctlr_cfg.vx.ki, ctlr_cfg.vx.kd, ctlr_cfg.vx.Tf, MAIN_LOOP_PERIOD);
    rc_filter_pid(&mbot_vy_pid, ctlr_cfg.vy.kp, ctlr_cfg.vy.ki, ctlr_cfg.vy.kd, ctlr_cfg.vy.Tf, MAIN_LOOP_PERIOD);
    rc_filter_pid(&mbot_wz_pid, ctlr_cfg.wz.kp, ctlr_cfg.wz.ki, ctlr_cfg.wz.kd, ctlr_cfg.wz.Tf, MAIN_LOOP_PERIOD);
    
    rc_filter_first_order_lowpass(&low_pass_filter, ctlr_cfg.lp_filter.dt, ctlr_cfg.lp_filter.time_const);
    return 0;
}

int mbot_motor_vel_ctlr(serial_mbot_motor_vel_t vel_cmd, serial_mbot_motor_vel_t vel, serial_mbot_motor_vel_t *mbot_motor_vel, mbot_params_t params){
    float right_error = vel_cmd.velocity[0] - vel.velocity[0];
    float left_error = vel_cmd.velocity[1] - vel.velocity[1];
    // float back_error = vel_cmd.velocity[2] - vel.velocity[2]; // not using for diff drive
    // printf("right_error: %.2f\n", right_error);
    // printf("left_error: %.2f\n", left_error);

    // get PID control command
    float right_cmd = rc_filter_march(&right_wheel_pid, right_error);
    float left_cmd = rc_filter_march(&left_wheel_pid, left_error);
    // float back_cmd = rc_filter_march(&back_wheel_pid, back_error);
    // printf("right_cmd: %.2f\n", right_cmd);
    // printf("left_cmd: %.2f\n", left_cmd);

    // Apply low pass filter on l,r commands; motor velocity
    // float right_cmd_lpf = rc_filter_march(&low_pass_filter, right_cmd);
    // float left_cmd_lpf = rc_filter_march(&low_pass_filter, left_cmd);
    // printf("right_cmd_lpf: %.2f\n", right_cmd_lpf);
    // printf("left_cmd_lpf: %.2f\n", left_cmd_lpf);

    // Clamp vel signal to min and max while preserving the sign
    // float right_cmd_abs_clamped = fmin(fmax(fabs(right_cmd), PWM_MIN), PWM_MAX);
    // float left_cmd_abs_clamped = fmin(fmax(fabs(left_cmd), PWM_MIN), PWM_MAX);
    // // Reapply the original sign
    // float right_cmd_clamped = (right_cmd > 0) ? right_cmd_abs_clamped : -right_cmd_abs_clamped;
    // float left_cmd_clamped = (left_cmd > 0) ? left_cmd_abs_clamped : -left_cmd_abs_clamped;
    // printf("right_cmd_clamped: %.2f\n", right_cmd_clamped);
    // printf("left_cmd_clamped: %.2f\n", left_cmd_clamped);

    // put vel value into a command
    mbot_motor_vel->velocity[0] = right_cmd;
    mbot_motor_vel->velocity[1] = left_cmd;

    // printf("\n");

    return 0;
}
