#include "Full_Run.h"
#include "Omniwheel.h"

#include <FEHLCD.h>
#include <FEHIO.h>
#include <FEHLCD.h>
#include <FEHMotor.h>
#include <FEHRPS.h>
#include <FEHUtility.h>
#include <FEHServo.h>
#include <FEHUtility.h>
#include <math.h>


DigitalEncoder r_encoder(FEHIO::P2_0);
DigitalEncoder l_encoder(FEHIO::P2_1);
AnalogInputPin CdS(FEHIO::P3_0);


/********************************
**Full code for running the course**
/********************************/

//This function is called from the main menu
//void FullRun(){


    //Function to determine color of jukebox
    //Unfiltered Cds cell value for red light is 0.291, 0.565 for blue light, and 3.075 for no light
    //todo Calibrate sensor values
    bool jukeboxColor() {
        while(true) {
        //Return true if red, return false if blue
            if (CdS.Value() > 0.2 && CdS.Value() < 0.35) {
                return true; //red
            }
            else if (CdS.Value() > 0.45 && CdS.Value() < 0.65) {
                return false; //blue
            }
        }
    }

    //Function to move robot in y direction
    void moveForward(float distance, int speed) {
    
        //318 / 2(pi)(1.25) = 40.4890175226 counts per inch
        const float countsperinch = 40.4890175226;
        if (distance < 0) {
            r_motor.SetPercent(-1 * speed);
            l_motor.SetPercent(-1 * speed);
        } else  {
            r_motor.SetPercent(speed);
            l_motor.SetPercent(speed);
        }

    
        r_encoder.ResetCounts();
        l_encoder.ResetCounts();

        while((r_encoder.Counts() < (fabs(distance) * countsperinch)) || l_encoder.Counts() < (fabs(distance) * countsperinch)) {
        //Stop the motors
            if (r_encoder.Counts() >= (fabs(distance) * countsperinch)) {
                r_motor.SetPercent(0);
            }
            if (l_encoder.Counts() >= (fabs(distance) * countsperinch)) {
                l_motor.SetPercent(0);
            }
        }
    } 

   

    //Function to turn the robot
    void turn(float angle, int speed) {
        //Reset encoders
        r_encoder.ResetCounts();
        l_encoder.ResetCounts();

        const float countsperinch = 40.4890175226; 
        //diameter = 7.54093496 inch
        //circumference of wheels rot. = 23.6905458715 inches 
        //counts for full rot. = 959.206926911
        const float countsPerDegree = 2.66446368586;
    
        if (angle < 0) {
            r_motor.SetPercent(-1 * speed);
            l_motor.SetPercent(-1 * speed);
            f_motor.SetPercent(-1 * speed);
        }
        else if {
            r_motor.SetPercent(speed);
            l_motor.SetPercent(speed);
            f_motor.SetPercent(speed);
        }

        while (r_encoder.Counts() < (countsPerDegree * fabs(angle))) {
            if (r_encoder.Counts() > (countsPerDegree * fabs(angle))) {
                r_motor.SetPercent(0);
                l_motor.SetPercent(0);
                f_motor.SetPercent(0);
            }

        }
  
    }

    int main(void)
    {
 
        //Call the function to orient the robot at the start of the run
        //initialize();
    
        //Call function to move forward 8 inches
        moveForward(8,25);
    

        //Turn left 45 degrees
        turn(-40, 25);
    

        //Call function to move left 11.5 inches
        moveForward(11.5, 25);
    

        //Determine color of the light sensor

        bool color = jukeboxColor();
        //Turn left 90 degrees
        turn(-90,25);
    

        //move to correct button
        if (color == 1) {
            //turn right 90 and move forward to red button
            turn(90,25);
            moveForward(0.5,18);
            turn(-90,25);
            moveForward(2,25);
            moveForward(-3,25);
        
        } 
        else {
            //turn left 90 and move to blue button
            turn(-90,25);
            moveForward(0.5,18);
            turn(90,25);
            moveForward(2,25);
            moveForward(-3,25);
        }


        //back up + turn 90 degrees

        //drive up ramp
    }



    
//}

