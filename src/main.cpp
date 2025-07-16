// This merged code integrates both line following and gimbal control using IMU and SensorBar
// Key components: DCMotor, SensorBar, IMU, Servo

#include "mbed.h"
#include "PESBoardPinMap.h"
#include "DebounceIn.h"
#include "DCMotor.h"
#include "SensorBar.h"
#include "IMU.h"
#include "Servo.h"
#include <Eigen/Dense>
#include <cmath>

#define M_PIf 3.14159265358979323846f

bool do_execute_main_task = false;
bool do_reset_all_once = false;

DebounceIn user_button(BUTTON1);
void toggle_do_execute_main_fcn() { do_execute_main_task = !do_execute_main_task; if (do_execute_main_task) do_reset_all_once = true; }

int main() {
    user_button.fall(&toggle_do_execute_main_fcn);
    const int main_task_period_ms = 20;
    Timer main_task_timer;
    DigitalOut user_led(LED1);
    DigitalOut led1(PB_9);
    DigitalOut enable_motors(PB_ENABLE_DCMOTORS);

    // Servo setup
    Servo servo_roll(PB_D0), servo_pitch(PB_D1);
    float servo_ang_min = 0.035f, servo_ang_max = 0.125f;
    servo_roll.calibratePulseMinMax(servo_ang_min, servo_ang_max);
    servo_pitch.calibratePulseMinMax(servo_ang_min, servo_ang_max);
    float angle_range_min = -M_PIf/2.0f, angle_range_max = M_PIf/2.0f;
    float normalised_angle_gain = 1.0f / M_PIf, normalised_angle_offset = 0.5f;
    float roll_servo_width = 0.5f, pitch_servo_width = 0.5f;

    // IMU setup
    ImuData imu_data;
    IMU imu(PB_IMU_SDA, PB_IMU_SCL);
    Eigen::Vector2f rp(0.0f, 0.0f);
    float phi_hat = 0.0f, theta_hat = 0.0f;
    float Ts = main_task_period_ms * 1.0e-3f, kp_roll = 3.0f; 
    float kp_pitch = 5.0f;

    // DCMotor setup
    float voltage_max = 12.0f, gear_ratio = 100.0f, kn = 140.0f / 12.0f;
    DCMotor motor_M1(PB_PWM_M1, PB_ENC_A_M1, PB_ENC_B_M1, gear_ratio, kn, voltage_max);
    DCMotor motor_M2(PB_PWM_M2, PB_ENC_A_M2, PB_ENC_B_M2, gear_ratio, kn, voltage_max);
    float d_wheel = 0.100f, b_wheel = 0.172f, bar_dist = 0.022f;
    float r1_wheel = d_wheel / 2.0f, r2_wheel = d_wheel / 2.0f;
    Eigen::Matrix2f Cwheel2robot;
    Cwheel2robot << r1_wheel/2.0f, r2_wheel/2.0f, r1_wheel/b_wheel, -r2_wheel/b_wheel;

    // SensorBar setup
    SensorBar sensor_bar(PB_9, PB_8, bar_dist);
    float angle = 0.0f, angular_vel = 0.0f, velocity = 0.0f;
    float Kp_line = 2.0f, Kp_nl = 5.0f, gain = 2.0f;
    float wheel_vel_max = 0.5f * M_PIf * motor_M2.getMaxPhysicalVelocity();

    main_task_timer.start();

    while (true) {
        main_task_timer.reset();

        imu_data = imu.getImuData();

        if (do_execute_main_task) {
            user_led = 1; led1 = 1; enable_motors = 1;
            if (!servo_roll.isEnabled()) servo_roll.enable();
            if (!servo_pitch.isEnabled()) servo_pitch.enable();

            float acc_x = imu_data.acc(0), acc_y = imu_data.acc(1), acc_z = imu_data.acc(2);
            float gyro_x = imu_data.gyro(0), gyro_y = imu_data.gyro(1);
            float phi_acc = atan2(acc_y, acc_z);
            float theta_acc = atan2(-acc_x, acc_z);

            phi_hat += Ts * (gyro_x + kp_roll * (phi_acc - phi_hat));
            theta_hat += Ts * (gyro_y + kp_pitch * (theta_acc - theta_hat));
            rp(0) = phi_hat; rp(1) = theta_hat;

            roll_servo_width = -normalised_angle_gain * rp(0) + normalised_angle_offset;
            pitch_servo_width =  normalised_angle_gain * rp(1) + normalised_angle_offset;
            if (angle_range_min <= rp(0) && rp(0) <= angle_range_max) servo_roll.setPulseWidth(roll_servo_width);
            if (angle_range_min <= rp(1) && rp(1) <= angle_range_max) servo_pitch.setPulseWidth(pitch_servo_width);

            if (sensor_bar.isAnyLedActive())
                angle = sensor_bar.getAvgAngleRad();

            if (angle >= 0.0f)
                angular_vel = Kp_line * angle - Kp_nl * angle * std::abs(angle);
            else
                angular_vel = Kp_line * angle - Kp_nl * angle * std::abs(angle);

            // velocity = wheel_vel_max * r1_wheel * exp(-gain * angle * angle);

            velocity = wheel_vel_max * r1_wheel * std::max(0.0f, cos(angle));

            // velocity = wheel_vel_max * r1_wheel * (1.0f / (1.0f + gain * angle * angle));


            Eigen::Vector2f robot_coord = {velocity, angular_vel};
            Eigen::Vector2f wheel_speed = Cwheel2robot.inverse() * robot_coord;

            motor_M1.setVelocity(wheel_speed(0) / (2.0f * M_PIf));
            motor_M2.setVelocity(wheel_speed(1) / (2.0f * M_PIf));
        } else {
            if (do_reset_all_once) {
                do_reset_all_once = false;
                roll_servo_width = pitch_servo_width = 0.5f;
                servo_roll.setPulseWidth(roll_servo_width);
                servo_pitch.setPulseWidth(pitch_servo_width);
                led1 = 0; enable_motors = 0;
            }
        }

        printf("Roll = %.2f | Pitch = %.2f\n", rp(0), rp(1));
        user_led = !user_led;

        int main_task_elapsed_time_ms = duration_cast<milliseconds>(main_task_timer.elapsed_time()).count();
        if (main_task_period_ms - main_task_elapsed_time_ms < 0)
            printf("Warning: Main task took too long\n");
        else
            thread_sleep_for(main_task_period_ms - main_task_elapsed_time_ms);
    }
}
