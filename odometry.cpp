/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       odometry.cpp                                              */
/*    Author:       CSUN Matabots                                             */                                                   
/*    Description:  Contains odometry algorithms                              */
/*                                                                            */
/*----------------------------------------------------------------------------*/

#include "main.h"
#include "subsystems.hpp"
#include "odometry.hpp"
#include <cmath>
#include <cstdlib>

double TW_Dia = 2; //inches

double x = 0.0, y = 0.0, theta = 0.0;
double prev_y = 0, prev_x = 0, prev_heading_deg = 0;

double TW_offset_y = 0;
double TW_offset_x = -5.15; //correct value 

// Heading fusion weight (0..1). Closer to 1 → trust IMU more.
double wIMU = 0;

float cycle_time = 20; // in millisec


#pragma region Constants

double MoE_Drive = 0.75;

double MoE_Turn = 2;

double drive_kP = 3;
double drive_kD = 20;

double turn_kP = 2.5; //no larger than 1 
double turn_kD = 11;

// for heading correction in driveTo
double kp_heading = 0.5; 
double kd_heading = 5; 

// ========================== Helper Functions ==========================
void clearEncoders() {
  verticalEnc.reset();
  horizontalEnc.reset();
//   imu.reset();
}

//Angle wrapping to find the shortest turn path 
double reduceAngle(int angle_deg) {

    while(angle_deg <= -180 || angle_deg > 180) {

        if (angle_deg < -180) { angle_deg += 360; }
        if (angle_deg > 180) { angle_deg -= 360; } 
    }
 return angle_deg;
}   

/*
If v > 127, return 127 
If v < -127, reutrn -127
else (: v), v remains unchagned 
*/
double clamp127(double v){ 
    return v > 127 ? 127 : (v < -127 ? -127 : v); 
}

void Odom_initialize(double x, double y, double theta) {

    clearEncoders();

    prev_x = (horizontalEnc.get_position() / 36000.0) * M_PI * TW_Dia;
    prev_y = (verticalEnc.get_position() / 36000.0) * M_PI * TW_Dia;
    prev_heading_deg = imu.get_heading(); 

}



void setDriveVelocity(double LSpeed, double RSpeed) {
leftMotors.move(LSpeed);
rightMotors.move(RSpeed);

}

void stopDrive() {
leftMotors.move(0);
rightMotors.move(0);
}


// =============================================================

#pragma region Update Odometry  


int update_position() {

//Read encoder values
    double v_enc = verticalEnc.get_position(); 
    double h_enc = horizontalEnc.get_position();
    double imu_deg = imu.get_heading(); 

    double v_rev = v_enc / 36000;
    double h_rev = h_enc / 36000;
        
//Convert rotations to linear distance
    float dist_y = v_rev * M_PI * TW_Dia;
    float dist_x = h_rev * M_PI * TW_Dia;

//Change in pos for x and y (deltas)
    double dy = dist_y - (prev_y);
        prev_y = dist_y;

    double dx = dist_x - (prev_x);
        prev_x = dist_x;


//Change in pos for theta (deltas), imu and horizontal TW
    double dtheta_wheel_deg = 0.0;
        if (std::fabs(TW_offset_x) > 0) { //make sure that horiz tracking wheel offset not 0
            dtheta_wheel_deg = dx / TW_offset_x * (180.0 / M_PI);   // update horizontal wheel angle change, dtheta = arc length / TW wheel offset (radius)
        }

    double dtheta_imu_deg = (imu_deg - prev_heading_deg);
        prev_heading_deg = (prev_heading_deg + dtheta_imu_deg); //update imu heading by adding prev imu deg to delta to get new value 

//Combine dtheta values from imu and horizontal TW  
    double dTheta_deg = wIMU * dtheta_imu_deg + (1.0 - wIMU) * dtheta_wheel_deg; //sclaes the theta (wheel & imu) contribution
    
    double dTheta_rad = dTheta_deg * (M_PI / 180.0);

/*Subtract out rotational component 
  dxmeas​=dxtrans​+dθ⋅TW_offset_x
*/

// double dx_trans = dx - (dTheta_rad * TW_offset_x); //not reading correctly 
double dy_trans = dy - (-dTheta_rad * TW_offset_y); // dy_trans = dy since TW_offset_y = 0 in this case

double theta_mid_rad = theta + (dTheta_rad/2);

//Rotate to field coordinates not robot coor
// double dx_f = cos(theta_mid_rad) * dx_trans - sin(theta_mid_rad) * dy_trans;
// double dy_f = sin(theta_mid_rad) * dx_trans + cos(theta_mid_rad) * dy_trans;

// double dy_f = cos(theta_mid_rad) * dy_trans; //Projects into field coordinates


//update position
// x  += dx_f;                 // update field X position
y  += dy;                 // update field Y position
theta += dTheta_deg; // update heading (deg)

std::cout << "Y-value: %.2f" << y << std::endl; //prints to pros brain terminal 
std::cout << "Theta: %.2f" << theta << std::endl;

return 1; 

}

// float getXPos(){ return x; }
float getYPos(){ return y; }
float getTheta(){ return theta; } //in deg 


#pragma region Motion_Control 

//first attempt 9/26

void driveTo (double x_1, double y_1) {

    update_position();

    double target = sqrt(pow(x_1 - x, 2) + pow(y_1 - y, 2)); //dist to target, pos value


    double targetAngle = atan2(y_1, x_1) * 180.0 / M_PI; //determines drive direc
    double driveDirec = (targetAngle > 0) ? 1 : -1;

    const double y_start = y;
    const double holdHeading = theta; // intial hold condition 
    // target = target * driveDirec;

    double driveError = fabs(target - y_start); // calculates intial drive Err, dir dealt with separately

 
    // float headingDirec = std::fabs(getTheta()) / getTheta(); 

    float deltadriveError = 0;
    float deltaheadingError = 0;
    float prevDriveError = driveError;
    float prevHeadErr = holdHeading;
    
    while (driveError > MoE_Drive) {

        update_position(); 

        // remaining vector and distance
        double dist = std::fabs(target - y);

        float max_speed = 60;

        driveError = max_speed * (dist/target); //fraction 
        deltadriveError = driveError - prevDriveError;
        prevDriveError = driveError; 

        // headingError = max_speed - (max_speed*(reduceAngle(getTheta())/theta)); 
        int headErr = reduceAngle(holdHeading - getTheta()); 
        int dHeadErr = headErr - prevHeadErr;
        prevHeadErr = headErr;

        float driveSpeed = (driveError * drive_kP) + (deltadriveError * drive_kD);
        float turnSpeed = (headErr * kp_heading) + (dHeadErr * kd_heading) ;

        // correct turning w/ both sides
        // passed as an integer 

        float L = 0, R = 0; 

        // different heading correction for foward and back 
        if (driveDirec == 1) {
            L = clamp127((driveSpeed * driveDirec) - turnSpeed); 
            R = clamp127((driveSpeed * driveDirec) + turnSpeed);
        }
        
        else if (driveDirec == -1) {
            L = clamp127((driveSpeed * driveDirec) + turnSpeed); 
            R = clamp127((driveSpeed * driveDirec) - turnSpeed);
        }
        
        setDriveVelocity(std::round(L), std::round(R));
    
    pros::delay(cycle_time);
    
    }
    
stopDrive(); //hard stop

}


void pointTurn(float (angle_deg)) {

update_position();

const double target = reduceAngle(angle_deg);
// theta = 0;

double turnError = fabs(target - getTheta()); // initial difference

        if (turnError < 1e-6) return;   


    int turnDirec = (target - getTheta() > 0) ? 1 : -1;

    float deltaTurnError = 0;
    float prevTurnError = turnError;

while(turnError > MoE_Turn) {

        update_position(); 

        turnError = fabs((target - getTheta()));

        deltaTurnError = turnError - prevTurnError;
        prevTurnError = turnError;

        float turnSpeed = (turnError * turn_kP) + (deltaTurnError * turn_kD);

        float L = 0, R = 0;

        L = clamp127(turnDirec * turnSpeed); 
        R = clamp127(turnDirec * -turnSpeed); //make one side neg for direc

        setDriveVelocity(std::round(L), std::round(R)); 

    pros::delay(cycle_time);

    }

stopDrive();

}



void moveTo (double x_m, double y_m, double theta_end) {

    y_m = y_m - y;
    x_m = x_m - x; 

    double theta_1 = std::atan2(x_m, y_m) * 180.0 / M_PI; //calculates targetAngle again
    pointTurn(theta_1);

    driveTo(x_m, y_m);

    pointTurn(theta_end);

}
