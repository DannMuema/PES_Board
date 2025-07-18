#include "mbed.h"

// pes board pin map
#include "PESBoardPinMap.h"

#include <chrono>
#include <thread>
#include <iostream>


#include "DebounceIn.h"
#include "IMU.h"
#include "Servo.h"
#include "DCMotor.h"
#include <Eigen/Dense>
#include "SensorBar.h"
#include <vector>
#include <stack>

#define M_PIf 3.14159265358979323846f // pi

float tanh_controller(float e);

bool do_execute_main_task = false; // This flag determines whether the main robot logic should execute.
                                   // It is toggled by pressing the blue button on the Nucleo board.
                                   // When true, the robot enters its operational modes (LEARNING, EXECUTE).
bool do_reset_all_once = false;    // Used to ensure one-time reset of critical state variables when the program starts or restarts.
                                   // Prevents unnecessary resets during normal execution and ensures clean initialization.

// objects for user button (blue button) handling on nucleo board
DebounceIn user_button(BUTTON1);   // create DebounceIn to evaluate the user button
void toggle_do_execute_main_fcn(); // custom function which is getting executed when user
                                   // button gets pressed, definition at the end

// main runs as an own thread
int main()
{
    // attach button fall function address to user button object
    user_button.fall(&toggle_do_execute_main_fcn);

    // while loop gets executed every main_task_period_ms milliseconds, this is a
    // simple approach to repeatedly execute main
    const int main_task_period_ms = 20; // define main task period time in ms e.g. 20 ms, therefore
                                        // the main task will run 50 times per second
    Timer main_task_timer;              // create Timer object which we use to run the main task
                                        // every main_task_period_ms

    // led on nucleo board
    DigitalOut user_led(LED1);

    // --- adding variables and objects and applying functions starts here ---

    // servo
    Servo servo_roll(PB_D0);
    Servo servo_pitch(PB_D1);

    // imu
    ImuData imu_data;
    IMU imu(PB_IMU_SDA, PB_IMU_SCL);
    Eigen::Vector2f rp(0.0f, 0.0f);

    // minimal pulse width and maximal pulse width obtained from the servo calibration process
    // careful, these values might differ from servo to servo
    // reely s-0090
    float servo_D0_ang_min = 0.0350f; 
    float servo_D0_ang_max = 0.1200f;
    // reely S3003
    float servo_D1_ang_min = 0.0250f;
    float servo_D1_ang_max = 0.1300f;

    // servo.setPulseWidth: before calibration (0,1) -> (min pwm, max pwm)
    // servo.setPulseWidth: after calibration (0,1) -> (servo_D0_ang_min, servo_D0_ang_max)
    servo_roll.calibratePulseMinMax(servo_D1_ang_min, servo_D1_ang_max);
    servo_pitch.calibratePulseMinMax(servo_D0_ang_min, servo_D0_ang_max);

    // pulse width
    static float roll_servo_width = 0.5f;
    static float pitch_servo_width = 0.5f;

    servo_roll.setPulseWidth(roll_servo_width);
    servo_pitch.setPulseWidth(pitch_servo_width);

    DigitalOut enable_motors(PB_ENABLE_DCMOTORS);

    const float voltage_max = 12.0f; // maximum voltage of battery packs, adjust this to
                                     // 6.0f V if you only use one battery pack
    const float gear_ratio = 100.00f;
    const float kn = 140.0f / 12.0f;

    // motor M1 and M2, do NOT enable motion planner when used with the LineFollower (disabled per default)
    
    DCMotor motor_M2(PB_PWM_M1, PB_ENC_A_M1, PB_ENC_B_M1, gear_ratio, kn, voltage_max);
    DCMotor motor_M1(PB_PWM_M2, PB_ENC_A_M2, PB_ENC_B_M2, gear_ratio, kn, voltage_max);

    // differential drive robot kinematics
    const float d_wheel = 0.100f; // wheel diameter in meters
    const float b_wheel = 0.172f;  // wheelbase, distance from wheel to wheel in meters
    const float bar_dist = 0.022f; // distance from wheel axis to leds on sensor bar / array in meters
    const float r1_wheel = d_wheel / 2.0f; // right wheel radius in meters
    const float r2_wheel = d_wheel / 2.0f; // left  wheel radius in meters
   
    // transforms wheel to robot velocities
    Eigen::Matrix2f Cwheel2robot;
    Cwheel2robot << r1_wheel / 2.0f   ,  r2_wheel / 2.0f   ,
                    r1_wheel / b_wheel, -r2_wheel / b_wheel;

    // sensor bar
    SensorBar sensor_bar(PB_9, PB_8, bar_dist);

    // angle measured from sensor bar (black line) relative to robot
    float angle{0.0f};

    // Intersection and end detection
    float leftMean = 0.0f;
    float centerMean = 0.0f;
    float rightMean = 0.0f;

    // rotational velocity controller
    const float Kp = 1.2f * 2.0f;
    const float Kp_nl = 1.2f * 17.0f;
    // const float wheel_vel_max = 2.0f * M_PIf * motor_M2.getMaxPhysicalVelocity();
    const float wheel_vel_max = 2.0f * M_PIf * motor_M2.getMaxPhysicalVelocity()* 0.1f;
    const float maximum_velocity = wheel_vel_max*r1_wheel; // maximum velocity

    // // set up states for state machine
    // enum NodeType
    // {
    //     NORMAL,
    //     DEAD_END,
    //     GOAL
    // } node_type = NodeType::NORMAL;

    enum Direction
    {
        LEFT,
        RIGHT,
        FOLLOWLINE,
        TURNAROUND,
    } direction = Direction::FOLLOWLINE;

    // set up states for state machine
    enum RobotState {
        INITIAL,
        STOP,
        LEARNING,
        EXECUTE
    } robot_state = RobotState::INITIAL;

    robot_state = RobotState::INITIAL;

    struct Node {
    int id;
    bool visited = false;
    bool leftTried = false;
    bool rightTried = false;
    bool straightTried = false;
    };

    std::vector<Node> pathStack;


    // navigation variables
    bool leftTurn = false, rightTurn = false, crossRoad = false, isIntersection = false, deadEnd = false;

     
    char maze_ram[3] = {};
    char maze_memory[50] = {};
    int store_counter = 0;
    int memory_counter = 0;
    int move_foward_time_counter = 0;
    int full_black_counter = 0;
    //const int full_black_threshold = 10;
    const int full_black_threshold = 6;  // Require multiple readings before stopping


    // start timer
    main_task_timer.start();

    // this loop will run forever
    while (true) {
        main_task_timer.reset();

        // --- code that runs every cycle at the start goes here ---

        if (do_execute_main_task) {

        // --- code that runs when the blue button was pressed goes here ---

            // only update sensor bar angle if an led is triggered
            if (sensor_bar.isAnyLedActive())
                angle = sensor_bar.getAvgAngleRad();
            
            // Intersection and end detection
            leftMean = sensor_bar.getMeanThreeAvgBitsLeft();
            centerMean = sensor_bar.getMeanFourAvgBitsCenter();
            rightMean = sensor_bar.getMeanThreeAvgBitsRight();

            // Detect if more than one sensor is on black
            leftTurn = (leftMean > 0.5f && centerMean > 0.3f && rightMean <= 0.5f);
            rightTurn = (rightMean > 0.5f && centerMean > 0.3f && leftMean <= 0.5f);
            crossRoad = (leftMean > 0.5f && centerMean > 0.5f && rightMean > 0.5f);
            deadEnd = (leftMean == 0.0f && centerMean == 0.0f && rightMean == 0.0f);
            isIntersection = (leftTurn || rightTurn || crossRoad);
        
             switch (robot_state) {
                case RobotState::INITIAL: {
                    // enable hardwaredriver dc motors: 0 -> disabled, 1 -> enabled
                    enable_motors = 1;
                    // robot_state = RobotState::FORWARD;
                    robot_state = RobotState::LEARNING;

                    break;
                }
                
                case RobotState::EXECUTE: {

                    // Code to run after the maze has been learned by the robot
                    break;
                }
                case RobotState::LEARNING: {

                    switch(direction){
                        case Direction::FOLLOWLINE: {
                            
                            //**Record the node details ***************/
                            
                            // Code to move the robot straight
                            Eigen::Vector2f robot_coord = {maximum_velocity/(1+(4*fabsf(angle)))*(1- tanh_controller(Kp_nl*fabsf(angle))),  // forward velocity changes with the error value
                                           Kp * angle +  tanh_controller(Kp_nl *angle)                 }; // simple proportional angle controller                 }; // simple proportional angle controller

                            // map robot velocities to wheel velocities in rad/sec
                            Eigen::Vector2f wheel_speed = Cwheel2robot.inverse() * robot_coord;

                            // setpoints for the dc motors in rps
                            motor_M1.setVelocity(wheel_speed(0) / (2.0f * M_PIf)); // set a desired speed for speed controlled dc motors M1
                            motor_M2.setVelocity(wheel_speed(1) / (2.0f * M_PIf)); // set a desired speed for speed controlled dc motors M2

                            // // Detecting the end goal
                            //  if (sensor_bar.getRaw() == 0b11111111 || (leftMean > 0.9f && centerMean > 0.9f && rightMean > 0.9f)) {
                            //     full_black_counter++;
                            //     if (full_black_counter > full_black_threshold) {
                            //     // Stop robot
                            //     motor_M1.setVelocity(0.0f);
                            //     motor_M2.setVelocity(0.0f);
                            //     enable_motors = 0;
                            //     // led1 = 0;
                            //     while (true) {
                            //     user_led = !user_led;  // Blink to indicate stop at endpoint
                            //     thread_sleep_for(300);
                            //     }
                            //     }
                            //     } else {
                            //     full_black_counter = 0; // Reset counter if black pattern not persistent
                            //     }


                            // Check for the next junction type
                            if ((sensor_bar.getRaw() & 0b11100000) == 0b11100000){
                                direction = Direction::LEFT;
                            }
                            if ((sensor_bar.getRaw() & 0b00000111) == 0b00000111){
                                
                                direction = Direction::RIGHT;                                
                            }
                            if ((sensor_bar.getRaw()) == 0b11111111){
                                // move foward for 1 second
                                motor_M1.setVelocity(0.9 / (2.0f * M_PIf)); // set a desired speed for speed controlled dc motors M1
                                motor_M2.setVelocity(0.9 / (2.0f * M_PIf)); // set a desired speed for speed controlled dc motors M2
                                wait_us(1000000);

                                // Check the sensor reading again if you have a reading at the center or still a crossroad reading.
                                leftMean = sensor_bar.getMeanThreeAvgBitsLeft();
                                centerMean = sensor_bar.getMeanFourAvgBitsCenter();
                                rightMean = sensor_bar.getMeanThreeAvgBitsRight();
                                crossRoad = (leftMean > 0.5f && centerMean > 0.5f && rightMean > 0.5f);

                                printf("leftMean: %f\n", leftMean);
                                printf("centerMean: %f\n", centerMean);
                                printf("rightMean: %f\n", rightMean);

                                // If the crossroad is still reading positive then we are at the end.
                                if (crossRoad){
                                     
                                    printf("End goal \n");
                                    printf("maze_memory %s\n", maze_memory);
                                    robot_state = RobotState::STOP;
                                    motor_M1.setVelocity(0);
                                    motor_M2.setVelocity(0);
                                    enable_motors = 0;
                                }

                               
                                // if we only have center mean, then we have a straight route to explore i.e @ '+' junction
                                if (!crossRoad && centerMean){
                                    // Prioritize turning to the left first, then straight, then right
                                }
                                // If we dont have a reading, we are at a T junction.
                                if (!(crossRoad && centerMean) && (leftMean > 0.0f)) {
                                    // move foward for 1 second
                                    motor_M1.setVelocity(-0.9 / (2.0f * M_PIf)); // set a desired speed for speed controlled dc motors M1
                                    motor_M2.setVelocity(-0.9 / (2.0f * M_PIf)); // set a desired speed for speed controlled dc motors M2
                                    wait_us(1000000);
                                    
                                    // Prioritize turning left
                                     Eigen::Vector2f robot_coord;
                                    robot_coord << 0.1f, 2.5f;
                                    Eigen::Vector2f wheel_speed = Cwheel2robot.inverse() * robot_coord; 
                                    motor_M1.setVelocity(wheel_speed(0) / (2.0f * M_PIf)); // set a desired speed for speed controlled dc motors M1
                                    motor_M2.setVelocity(wheel_speed(1) / (2.0f * M_PIf)); // set a desired speed for speed controlled dc motors M2
                                    wait_us(375000);

                                    // Record that we have turned left
                                    maze_ram[store_counter] = 'L'; 
                                    store_counter++;

                                    // Check if the stored 3 directions can be simplified
                                    if (store_counter == 2){
                                        // Simplify the turns
                                        if(maze_ram[0] == 'L' && maze_ram[1] == 'B' && maze_ram[2] == 'R' ){

                                            maze_memory[memory_counter] = 'B';
                                            memory_counter++;

                                        }
                                        if(maze_ram[0] == 'L' && maze_ram[1] == 'B' && maze_ram[2] == 'S' ){

                                            maze_memory[memory_counter] = 'R';
                                            memory_counter++;

                                        }
                                        if(maze_ram[0] == 'L' && maze_ram[1] == 'B' && maze_ram[2] == 'L' ){

                                            maze_memory[memory_counter] = 'S';
                                            memory_counter++;

                                        }
                                        if(maze_ram[0] == 'S' && maze_ram[1] == 'B' && maze_ram[2] == 'L' ){

                                            maze_memory[memory_counter] = 'R';
                                            memory_counter++;

                                        }
                                        if(maze_ram[0] == 'S' && maze_ram[1] == 'B' && maze_ram[2] == 'S' ){

                                            maze_memory[memory_counter] = 'B';
                                            memory_counter++;

                                        }
                                        if(maze_ram[0] == 'R' && maze_ram[1] == 'B' && maze_ram[2] == 'L' ){

                                            maze_memory[memory_counter] = 'B';
                                            memory_counter++;

                                        }
                                        store_counter = 0;
                                    }
                                    

                                    // Afterwards, change the direction to straight
                                    direction = Direction::FOLLOWLINE;
                                }

                                
                            }

                            // Detecting a deadend. Centermean reads all 0
                            // printf("Center mean: %f \n", centerMean);
                            if ((centerMean <= 0.0f)){

                                 
                                direction = Direction::TURNAROUND;
                            }

                            break;
                        }
                        case Direction::LEFT: {
                            printf("Turn Left \n"); 
                            //**Record the node details ***************/

                            motor_M1.setVelocity(0.9 / (2.0f * M_PIf)); // set a desired speed for speed controlled dc motors M1
                            motor_M2.setVelocity(0.9 / (2.0f * M_PIf)); // set a desired speed for speed controlled dc motors M2
                            wait_us(1000000);

                            // if (move_foward_time_counter > 10){
                                // Check the sensor reading again if you have a reading at the center or no reading.
                            centerMean = sensor_bar.getMeanFourAvgBitsCenter();

                            // Move back for 1 sec
                            motor_M1.setVelocity(-0.9 / (2.0f * M_PIf)); // set a desired speed for speed controlled dc motors M1
                            motor_M2.setVelocity(-0.9 / (2.0f * M_PIf)); // set a desired speed for speed controlled dc motors M2
                            wait_us(1000000);

                            // If no reading, turn left by 90 degrees
                            if (centerMean <= 0.0f){
                            
                            Eigen::Vector2f robot_coord;
                            robot_coord << 0.1f, 2.5f;
                            Eigen::Vector2f wheel_speed = Cwheel2robot.inverse() * robot_coord; 
                            motor_M1.setVelocity(wheel_speed(0) / (2.0f * M_PIf)); // set a desired speed for speed controlled dc motors M1
                            motor_M2.setVelocity(wheel_speed(1) / (2.0f * M_PIf)); // set a desired speed for speed controlled dc motors M2
                            wait_us(375000);
                            }
                            // If there is a reading then you can either turn left or go straight
                            else {
                            // Left hand wall follower. Will prioritize turning left over the straight direction
                            // while (!((sensor_bar.getRaw() & 0b00011000) == 0b00011000)){

                            // }
                            Eigen::Vector2f robot_coord;
                            robot_coord << 0.1f, 2.5f;
                            Eigen::Vector2f wheel_speed = Cwheel2robot.inverse() * robot_coord; 
                            motor_M1.setVelocity(wheel_speed(0) / (2.0f * M_PIf)); // set a desired speed for speed controlled dc motors M1
                            motor_M2.setVelocity(wheel_speed(1) / (2.0f * M_PIf)); // set a desired speed for speed controlled dc motors M2
                            wait_us(375000);
                            // Record the turn direction taken
                            maze_ram[store_counter] = 'L'; 
                            store_counter++;

                            // Check if the stored 3 directions can be simplified
                            if (store_counter == 2){
                                // Simplify the turns
                                if(maze_ram[0] == 'L' && maze_ram[1] == 'B' && maze_ram[2] == 'R' ){

                                    maze_memory[memory_counter] = 'B';
                                    memory_counter++;

                                }
                                if(maze_ram[0] == 'L' && maze_ram[1] == 'B' && maze_ram[2] == 'S' ){

                                    maze_memory[memory_counter] = 'R';
                                    memory_counter++;

                                }
                                if(maze_ram[0] == 'L' && maze_ram[1] == 'B' && maze_ram[2] == 'L' ){

                                    maze_memory[memory_counter] = 'S';
                                    memory_counter++;

                                }
                                if(maze_ram[0] == 'S' && maze_ram[1] == 'B' && maze_ram[2] == 'L' ){

                                    maze_memory[memory_counter] = 'R';
                                    memory_counter++;

                                }
                                if(maze_ram[0] == 'S' && maze_ram[1] == 'B' && maze_ram[2] == 'S' ){

                                    maze_memory[memory_counter] = 'B';
                                    memory_counter++;

                                }
                                if(maze_ram[0] == 'R' && maze_ram[1] == 'B' && maze_ram[2] == 'L' ){

                                    maze_memory[memory_counter] = 'B';
                                    memory_counter++;

                                }
                                store_counter = 0;
                            }
                            }

                            // Afterwards, change the direction to followline straight
                            direction = Direction::FOLLOWLINE;
                            move_foward_time_counter = 0;
                            printf("maze_ram %s\n", maze_ram);
                            // }

                            break;
                        }
                        case Direction::RIGHT: {
                            printf("Turn Right \n"); 
                            
                            motor_M1.setVelocity(0.9 / (2.0f * M_PIf)); // set a desired speed for speed controlled dc motors M1
                            motor_M2.setVelocity(0.9 / (2.0f * M_PIf)); // set a desired speed for speed controlled dc motors M2
                            wait_us(1000000);
                            

                            // if (move_foward_time_counter > 30){
                                // Check the sensor reading again if you have a reading at the center or no reading.
                            centerMean = sensor_bar.getMeanFourAvgBitsCenter();

                            // Move the robot back for 1 second
                            motor_M1.setVelocity(-0.9 / (2.0f * M_PIf)); // set a desired speed for speed controlled dc motors M1
                            motor_M2.setVelocity(-0.9 / (2.0f * M_PIf)); // set a desired speed for speed controlled dc motors M2
                            wait_us(1000000);

                            printf("centerMean: %f \n", centerMean); 

                            // If no reading, turn right by 90 degrees
                            if (centerMean <= 0.0f){
                            //  Code to turn the robot to the right  
                            Eigen::Vector2f robot_coord;
                            robot_coord << 0.1f, -2.5f;
                            Eigen::Vector2f wheel_speed = Cwheel2robot.inverse() * robot_coord; 
                            motor_M1.setVelocity(wheel_speed(0) / (2.0f * M_PIf)); // set a desired speed for speed controlled dc motors M1
                            motor_M2.setVelocity(wheel_speed(1) / (2.0f * M_PIf)); // set a desired speed for speed controlled dc motors M2
                            wait_us(675000);
                            }
                            // If there is a reading then you can either turn right or go straight
                            else {
                            // Left hand wall follower. Will prioritize going straigt over the Right turn
                            direction = Direction::FOLLOWLINE;

                            // Record the turn direction taken
                            maze_ram[store_counter] = 'S'; 
                            store_counter++;

                            // Check if the stored 3 directions can be simplified
                            if (store_counter == 2){
                                // Simplify the turns
                                if(maze_ram[0] == 'L' && maze_ram[1] == 'B' && maze_ram[2] == 'R' ){

                                    maze_memory[memory_counter] = 'B';
                                    memory_counter++;

                                }
                                if(maze_ram[0] == 'L' && maze_ram[1] == 'B' && maze_ram[2] == 'S' ){

                                    maze_memory[memory_counter] = 'R';
                                    memory_counter++;

                                }
                                if(maze_ram[0] == 'L' && maze_ram[1] == 'B' && maze_ram[2] == 'L' ){

                                    maze_memory[memory_counter] = 'S';
                                    memory_counter++;

                                }
                                if(maze_ram[0] == 'S' && maze_ram[1] == 'B' && maze_ram[2] == 'L' ){

                                    maze_memory[memory_counter] = 'R';
                                    memory_counter++;

                                }
                                if(maze_ram[0] == 'S' && maze_ram[1] == 'B' && maze_ram[2] == 'S' ){

                                    maze_memory[memory_counter] = 'B';
                                    memory_counter++;

                                }
                                if(maze_ram[0] == 'R' && maze_ram[1] == 'B' && maze_ram[2] == 'L' ){

                                    maze_memory[memory_counter] = 'B';
                                    memory_counter++;

                                }
                                store_counter = 0;
                            }
                            }

                            motor_M1.setVelocity(0.9 / (2.0f * M_PIf)); // set a desired speed for speed controlled dc motors M1
                            motor_M2.setVelocity(0.9 / (2.0f * M_PIf)); // set a desired speed for speed controlled dc motors M2
                            wait_us(1000000);

                            // Afterwards change the direction to straight
                            direction = Direction::FOLLOWLINE;
                            

                            move_foward_time_counter = 0;
                            printf("maze_ram %s\n", maze_ram);
                            

                            break;
                        }
                        case Direction::TURNAROUND: {
                            // node_type = NodeType::DEAD_END;
                            printf("Deadend \n"); 

                            // Code to run if we meet a deadend
                            // move foward for some time 
                            move_foward_time_counter++;

                            if (move_foward_time_counter > 10){
                                
                            if (centerMean <= 0.0f){

                                while (!((sensor_bar.getRaw() & 0b00011000) == 0b00011000)){
                                    Eigen::Vector2f robot_coord;
                                    robot_coord << 0.0f, 3.0f;
                                    Eigen::Vector2f wheel_speed = Cwheel2robot.inverse() * robot_coord; 
                                    motor_M1.setVelocity(wheel_speed(0) / (2.0f * M_PIf)); // set a desired speed for speed controlled dc motors M1
                                    motor_M2.setVelocity(wheel_speed(1) / (2.0f * M_PIf)); // set a desired speed for speed controlled dc motors M2
                                }
                                // motor_M1.setRotation(2.0f);
                                // motor_M2.setRotation(-2.0f);

                                //  store the direction taken &  simplify
                                // Record the turn direction taken
                                maze_ram[store_counter] = 'B'; 
                                store_counter++;

                                // Check if the stored 3 directions can be simplified
                                if (store_counter == 2){
                                    // Simplify the turns
                                    if(maze_ram[0] == 'L' && maze_ram[1] == 'B' && maze_ram[2] == 'R' ){

                                        maze_memory[memory_counter] = 'B';
                                        memory_counter++;

                                    }
                                    if(maze_ram[0] == 'L' && maze_ram[1] == 'B' && maze_ram[2] == 'S' ){

                                        maze_memory[memory_counter] = 'R';
                                        memory_counter++;

                                    }
                                    if(maze_ram[0] == 'L' && maze_ram[1] == 'B' && maze_ram[2] == 'L' ){

                                        maze_memory[memory_counter] = 'S';
                                        memory_counter++;

                                    }
                                    if(maze_ram[0] == 'S' && maze_ram[1] == 'B' && maze_ram[2] == 'L' ){

                                        maze_memory[memory_counter] = 'R';
                                        memory_counter++;

                                    }
                                    if(maze_ram[0] == 'S' && maze_ram[1] == 'B' && maze_ram[2] == 'S' ){

                                        maze_memory[memory_counter] = 'B';
                                        memory_counter++;

                                    }
                                    if(maze_ram[0] == 'R' && maze_ram[1] == 'B' && maze_ram[2] == 'L' ){

                                        maze_memory[memory_counter] = 'B';
                                        memory_counter++;

                                    }
                                    store_counter = 0;
                                }
                            }

                            // Aftewards move in the straight direction
                            direction = Direction::FOLLOWLINE;
                            move_foward_time_counter = 0;

                            }

                            break;
                        }
                        default: {
                            
                            break;
                        }
                    }

                    break;
                }
                
                default: {

                    // The robot will be in stop mode
                    motor_M1.setVelocity(0);
                    motor_M2.setVelocity(0);
                    break; // do nothing
                }
            }

        } else {
            // the following code block gets executed only once
            if (do_reset_all_once) {
                do_reset_all_once = false;

                // --- variables and objects that should be reset go here ---

                // reset variables and objects
                enable_motors = 0;
                // end_counter = 0;
                robot_state = RobotState::INITIAL;
                direction = Direction::FOLLOWLINE;
                angle = 0;
                store_counter = 0;
                move_foward_time_counter = 0;
                maze_ram[3] = {};
                maze_memory[50] = {}; 
            }
        }

        // toggling the user led
        user_led = !user_led;

        // --- code that runs every cycle at the end goes here ---

        // read timer and make the main thread sleep for the remaining time span (non blocking)
        int main_task_elapsed_time_ms = duration_cast<milliseconds>(main_task_timer.elapsed_time()).count();
        if (main_task_period_ms - main_task_elapsed_time_ms < 0)
            printf("Warning: Main task took longer than main_task_period_ms\n");
        else
            thread_sleep_for(main_task_period_ms - main_task_elapsed_time_ms);
    }
}

void toggle_do_execute_main_fcn()
{
    // toggle do_execute_main_task if the button was pressed
    do_execute_main_task = !do_execute_main_task;
    // set do_reset_all_once to true if do_execute_main_task changed from false to true
    if (do_execute_main_task)
        do_reset_all_once = true;
}

float tanh_controller(float e) {
    return e / (1.0f + fabsf(e));
}