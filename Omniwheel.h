#ifndef OMNIWHEEL_H
#define OMNIWHEEL_H
#include <FEHIO.h>
#include <FEHMotor.h>

//Initialize motors
FEHMotor f_motor(FEHMotor::Motor0,9);
FEHMotor l_motor(FEHMotor::Motor1,9);
FEHMotor r_motor(FEHMotor::Motor2,9);

//moves the robot from it's current RPS location to the final RPS location
void absoluteMove(float final_x, float final_y, float final_heading, float power);

//Precisely translate and use the encoders
void relativeTranslateEncoder(float dx, float dy, float power);

//Precisely rotate using the encoders using angle in //!RADIANS
void relativeRotateEncoder(float angle, float power);

//Moves the robot a distance relative to it's current position, and a rotation relative to it's current rotation.
void relativeMove(float di, float dj, float dheading, float power, DigitalInputPin bump_switch);

//todo more functions to add for sure.

#endif