#include "motors.hpp"
#include "units.hpp"
#include "odometry.hpp"
#include "pid.hpp"

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

void turn(Eigen::VectorXf& targetPosition){
    Eigen::VectorXf currentPosition = get_odometry_pose();  // Get current position x, y, theta
    Eigen::VectorXf error = targetPosition - currentPosition;
    float theta_integral = 0.0;
    float prev_theta_error = 0.0;

    float Kp_theta = 2000;
    float Ki_theta = 0;
    float Kd_theta = 0;

    float tolerance = 0.1; //tolerance in radians
    double theta_error = atan2(error[1], error[0]) - currentPosition[2];
    chrono::high_resolution_clock::time_point start = chrono::high_resolution_clock::now();
    while (abs(theta_error) > tolerance){
        chrono::high_resolution_clock::time_point currentTime = chrono::high_resolution_clock::now();
        chrono::duration<double> elapsedTime = chrono::duration_cast<chrono::duration<double>>(currentTime - start);

        // Check if 10ms have passed
        if (elapsedTime.count() < 0.01){
            continue;
        }
        
        // Update the start time for the next iteration
        start = chrono::high_resolution_clock::now();

        currentPosition = get_odometry_pose();  // Update current position
        error = targetPosition - currentPosition;
        theta_error = atan2(error[1], error[0]) - currentPosition[2];
        //theta_error = normalizeAngle(theta_error);

        cout << "currentPosition: " << currentPosition << endl;
        cout << "angle: " << (currentPosition[2]*180)/M_PI << endl;
        cout << "theta_error: " << theta_error << endl;

        theta_integral += theta_error;

        // P controller as in PID with Ki and Kd = 0
        double theta_controlOutput = calculatePID(theta_error, theta_integral, prev_theta_error, Kp_theta, Ki_theta, Kd_theta);

        // Adjust motor speeds based on control output
        double leftWheelSpeed = std::max(std::min(-theta_controlOutput, 350.0), -350.0);
        double rightWheelSpeed = std::max(std::min(theta_controlOutput, 350.0), -350.0);

        cout << "leftWheelSpeed: " << leftWheelSpeed << " rightWheelSpeed: " << rightWheelSpeed << endl;

        call_motors(leftWheelSpeed, rightWheelSpeed);

        prev_theta_error = theta_error;

    }

    stop_motors();
}

void go_to(Eigen::VectorXf& targetPosition){
    turn(targetPosition);
    Eigen::VectorXf currentPosition = get_odometry_pose();  // Get current position x, y, theta
    double distance_integral = 0.0;
    double theta_integral = 0.0;
    double prev_distance_error = 0.0;
    double prev_theta_error = 0.0;

    double Kp_distance = 50;
    double Kp_theta = 2000;
    double Ki_distance = 0;
    double Ki_theta = 0;
    double Kd_distance = 0;
    double Kd_theta = 0;

    double tolerance = 30; //tolerance in mm

    double initial_theta_error = atan2(targetPosition[1] - currentPosition[1], targetPosition[0] - currentPosition[0]) - currentPosition[2];
    chrono::high_resolution_clock::time_point start = chrono::high_resolution_clock::now();
    while ((targetPosition - currentPosition).norm() > tolerance){
        chrono::high_resolution_clock::time_point currentTime = chrono::high_resolution_clock::now();
        chrono::duration<double> elapsedTime = chrono::duration_cast<chrono::duration<double>>(currentTime - start);

        // Check if 10ms have passed
        if (elapsedTime.count() < 0.01){
            continue;
        }
        
        // Update the start time for the next iteration
        start = chrono::high_resolution_clock::now();

        currentPosition = get_odometry_pose();  // Update current position
        Eigen::VectorXf error = targetPosition - currentPosition;
        double distance_error = error.head<2>().norm();  // Distance error
        double theta_error = atan2(error[1], error[0]) - currentPosition[2];  // Angle error
        //theta_error = normalizeAngle(theta_error);

        //cout << "currentPosition: " << currentPosition << endl;
        //cout << "targetPosition: " << targetPosition << endl;
        //cout << "distance_error: " << distance_error << " theta_error: " << theta_error << endl;

        distance_integral += distance_error;
        theta_integral += theta_error;

        // P controller as in PID with Ki and Kd = 0
        double distance_controlOutput = calculatePID(distance_error, distance_integral, prev_distance_error, Kp_distance, Ki_distance, Kd_distance);
        double theta_controlOutput = calculatePID(theta_error, theta_integral, prev_theta_error, Kp_theta, Ki_theta, Kd_theta);

        // Adjust motor speeds based on control output
        double leftWheelSpeed = std::max(std::min(-theta_controlOutput, 3000.0), -3000.0);
        double rightWheelSpeed = std::max(std::min(theta_controlOutput, 3000.0), -3000.0);
        distance_controlOutput = distance_controlOutput - abs(theta_controlOutput);
        leftWheelSpeed += std::max(std::min(distance_controlOutput, 3000.0), -3000.0);
        rightWheelSpeed += std::max(std::min(distance_controlOutput, 3000.0), -3000.0);
        
        //cout << "leftWheelSpeed: " << leftWheelSpeed << " rightWheelSpeed: " << rightWheelSpeed << endl;

        call_motors(leftWheelSpeed, rightWheelSpeed);
        leftWheelSpeed -= std::max(std::min(theta_controlOutput, 3000.0), -3000.0);
        rightWheelSpeed += std::max(std::min(theta_controlOutput, 3000.0), -3000.0);

        prev_distance_error = distance_error;
        prev_theta_error = theta_error;
    }

    stop_motors();
}