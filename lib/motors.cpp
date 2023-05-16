#include "motors.hpp"
#include "units.hpp"
#include "odometry.hpp"
#include <Eigen/Dense>

MotorDataType MotorData;
static const int SPI_Channel = 1;

// Motor 1 = left
// Motor 2 = right

// Create variables for the encoder values as singed int 32 
int l_encoder;
int r_encoder;
int encoders_have_value;

int encoders_ready;

float dD;
float dT;

void init_motors(){
    wiringPiSetup(); 
	wiringPiSPISetup(SPI_Channel, 1000000);
    encoders_ready = 0;
    encoders_have_value = 0;
    dD = 0;
    dT = 0;
}

void call_motors(int l_speed, int r_speed){
    encoders_ready = 0; // Reset the encoders ready flag

    // Set the motor data
    MotorData.Set_Speed_M1 = -l_speed;
    MotorData.Set_Speed_M2 = -r_speed;

    // Send the motor data to the motors
    Send_Read_Motor_Data(&MotorData);

    // Get the new encoder values
    int new_l_encoder = MotorData.Encoder_M1;
    int new_r_encoder = MotorData.Encoder_M2;

    // Check if the encoder have value
    if(!encoders_have_value){
        l_encoder = new_l_encoder;
        r_encoder = new_r_encoder;
        encoders_have_value = 1;
    }

    // Calculate the change in distance and angle based on the new encoder values
    float ddr = (float)(new_r_encoder - r_encoder) * MM_PER_COUNT;
    float ddl = (float)(new_l_encoder - l_encoder) * MM_PER_COUNT;

    dD = (ddr + ddl) / 2.0 * -1;    // -1 since the encoder values are negative when the robot moves forward
    dT = -(ddr - ddl) / WHEEL_BASE;

    // Update the encoder values
    l_encoder = new_l_encoder;
    r_encoder = new_r_encoder;

    encoders_ready = 1;
    update_odometry_pose();
}


// Get functions...
float get_delta_D(){
    return dD;
}

float get_delta_theta(){
    return dT;
}

int get_l_encoder(){
    return l_encoder;
}

int get_r_encoder(){
    return r_encoder;
}

int get_l_motorSpeed(){
    return (int)MotorData.Act_Speed_M1 * -1;
}

int get_r_motorSpeed(){
    return (int)MotorData.Act_Speed_M2 * -1;
}


// Motor user functions
void stop_motors(){
    call_motors(0, 0);
}

void go_to((const Eigen::VectorXf& targetPosition)){
    Eigen::VectorXf currentPosition = get_odometry_pose();  // Get current position x, y, theta
    double distance_integral = 0.0;
    double theta_integral = 0.0;
    double prev_distance_error = 0.0;
    double prev_theta_error = 0.0;

    double tolerance = 10; //tolerance in mm

    while ((targetPosition - currentPosition).norm() > tolerance){
        currentPosition = get_odometry_pose();  // Update current position
        Eigen::VectorXf error = targetPosition - currentPosition;
        double distance_error = error.head<2>().norm();  // Distance error
        double theta_error = atan2(error[1], error[0]) - currentPosition[2];  // Angle error

        distance_integral += distance_error;
        theta_integral += theta_error;

        double distance_controlOutput = calculatePI(distance_error, distance_integral, Kp_distance, Ki_distance);
        double theta_controlOutput = calculatePI(theta_error, theta_integral, Kp_theta, Ki_theta);

        // Adjust motor speeds based on control output
        double leftWheelSpeed = std::max(std::min(distance_controlOutput - theta_controlOutput, 3000), -3000);
        double rightWheelSpeed = std::max(std::min(distance_controlOutput + theta_controlOutput, 3000), -3000);

        call_motors(leftWheelSpeed, rightWheelSpeed);

        prev_distance_error = distance_error;
        prev_theta_error = theta_error;
    }

    stop_motors();
}