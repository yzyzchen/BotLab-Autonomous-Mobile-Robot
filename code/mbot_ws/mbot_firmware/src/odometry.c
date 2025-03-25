#include "odometry.h"
#include <math.h>

int mbot_calculate_diff_body_vel(float wheel_left_vel, float wheel_right_vel, serial_twist2D_t *mbot_vel){
    mbot_vel->vx =  DIFF_WHEEL_RADIUS * (wheel_left_vel - wheel_right_vel) / 2.0f;
    mbot_vel->vy = 0;
    mbot_vel->wz =  DIFF_WHEEL_RADIUS * (-wheel_left_vel - wheel_right_vel) / (2.0f * DIFF_BASE_RADIUS);
    return 0; // Return 0 to indicate success
}
int mbot_calculate_diff_body_vel_imu(float wheel_left_vel, float wheel_right_vel, serial_mbot_imu_t imu, serial_twist2D_t *mbot_vel){
    float linear_velocity = DIFF_WHEEL_RADIUS * (wheel_left_vel + wheel_right_vel) / 2.0f;
    // Using IMU data for angular velocity
    float angular_velocity_z = imu.gyro[2];

    mbot_vel->vx = linear_velocity;
    mbot_vel->vy = 0; // Differential drive robots typically do not move sideways
    mbot_vel->wz = angular_velocity_z;

    return 0; // Return 0 to indicate success
}
int mbot_calculate_omni_body_vel(float wheel_left_vel, float wheel_right_vel, float wheel_back_vel, serial_twist2D_t *mbot_vel){
    mbot_vel->vx =  OMNI_WHEEL_RADIUS * (wheel_left_vel * INV_SQRT3 - wheel_right_vel * INV_SQRT3);
    mbot_vel->vy =  OMNI_WHEEL_RADIUS * (-wheel_left_vel / 3.0 - wheel_right_vel / 3.0 + wheel_back_vel * (2.0 / 3.0));
    mbot_vel->wz =  OMNI_WHEEL_RADIUS * -(wheel_left_vel + wheel_right_vel + wheel_back_vel) / (3.0f * OMNI_BASE_RADIUS);
    return 0; // Return 0 to indicate success
}
int mbot_calculate_omni_body_vel_imu(float wheel_left_vel, float wheel_right_vel, float wheel_back_vel, serial_mbot_imu_t imu, serial_twist2D_t *mbot_vel){
    mbot_vel->vx = OMNI_WHEEL_RADIUS * (wheel_left_vel * INV_SQRT3 - wheel_right_vel * INV_SQRT3);
    mbot_vel->vy = OMNI_WHEEL_RADIUS * (-wheel_left_vel / 3.0 - wheel_right_vel / 3.0 + wheel_back_vel * (2.0 / 3.0));
    mbot_vel->wz = imu.gyro[2]; // Using IMU data for angular velocity

    return 0; // Return 0 to indicate success
}

int mbot_calculate_odometry(serial_twist2D_t mbot_vel, float dt, serial_pose2D_t *odometry){
    float vx_space = mbot_vel.vx * cos(odometry->theta) - mbot_vel.vy * sin(odometry->theta);
    float vy_space = mbot_vel.vx * sin(odometry->theta) + mbot_vel.vy * cos(odometry->theta);

    odometry->x += vx_space * dt;
    odometry->y += vy_space * dt;
    odometry->theta += mbot_vel.wz * dt;

    // Normalize theta to be between -pi and pi
    while (odometry->theta > M_PI) odometry->theta -= 2 * M_PI;
    while (odometry->theta <= -M_PI) odometry->theta += 2 * M_PI;

    return 0; // Return 0 to indicate success
}
