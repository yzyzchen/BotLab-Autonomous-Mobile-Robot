#ifndef CONTROLLER_H
#define CONTROLLER_H

#include <mbot_lcm_msgs_serial.h>
#include <mbot/defs/mbot_params.h>
#include <rc/math/filter.h>

typedef struct mbot_pid_cfg_t{
    float kp;
    float ki;
    float kd;
    float Tf;
} mbot_pid_cfg_t;

typedef struct mbot_lpf_cfg_t{
    float dt;
    float time_const;
} mbot_lpf_cfg_t;

typedef struct mbot_ctlr_cfg_t{
    mbot_pid_cfg_t right;
    mbot_pid_cfg_t left;
    mbot_pid_cfg_t back;
    mbot_pid_cfg_t vx;
    mbot_pid_cfg_t vy;
    mbot_pid_cfg_t wz;
    mbot_lpf_cfg_t lp_filter;
} mbot_ctlr_cfg_t;

rc_filter_t left_wheel_pid;
rc_filter_t right_wheel_pid;
rc_filter_t back_wheel_pid;
rc_filter_t mbot_vx_pid;
rc_filter_t mbot_vy_pid;
rc_filter_t mbot_wz_pid;
rc_filter_t low_pass_filter;


int mbot_init_ctlr(mbot_ctlr_cfg_t ctlr_cfg);
int mbot_motor_vel_ctlr(serial_mbot_motor_vel_t vel_cmd, serial_mbot_motor_vel_t vel, serial_mbot_motor_vel_t *mbot_motor_vel, mbot_params_t params);
int mbot_ctlr(serial_twist2D_t vel_cmd, serial_twist2D_t vel, serial_mbot_motor_vel_t *mbot_motor_vel);


#endif
