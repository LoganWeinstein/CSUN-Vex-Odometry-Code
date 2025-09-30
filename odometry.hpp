# pragma once

/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       odometry.hpp                                              */
/*    Author:       CSUN Matabots                                             */                                                   
/*    Description:  Contains odometry definitions                             */
/*                                                                            */
/*----------------------------------------------------------------------------*/


float getXPos();
float getYPos();
float getTheta();

extern double x, y, theta;

void Odom_initialize(double x, double y, double theta);

int update_position();

void pointTurn (float angle_deg); //Turns to specified heading 

void driveTo(double x_1, double y_1); //Drives to specified position (only y-pos right now)

void moveTo (double x_m, double y_m, double theta_end);
