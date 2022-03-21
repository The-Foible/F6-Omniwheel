#include <FEHLCD.h>
#include <FEHIO.h>
#include <FEHMotor.h>
#include <FEHRPS.h>
#include <FEHUtility.h>
#include <FEHServo.h>
#include <FEHUtility.h>
#include <FEHSD.h>
#include <math.h>
#include <time.h>
#include <string.h>
#include <sstream>
#include <string>

#define _USE_MATH_DEFINES //Use M_PI

#define fivePIsix 2.61799387799
#define threePItwo 4.71238898038
#define onePIsix 0.52359877559

#define countsperinch 40.4890175226
 
DigitalEncoder r_encoder(FEHIO::P0_0);
DigitalEncoder l_encoder(FEHIO::P0_1);
DigitalEncoder b_encoder(FEHIO::P0_2);

AnalogInputPin CdS(FEHIO::P0_3);

FEHMotor r_motor(FEHMotor::Motor0, 9.0);
FEHMotor l_motor(FEHMotor::Motor1, 9.0);
FEHMotor b_motor(FEHMotor::Motor2, 9.0);

FEHServo arm_servo(FEHServo::Servo0);
FEHServo flip_servo(FEHServo::Servo1);

    /*
    The GetLightColor function determines the color of light detected by the CdS cell based on analog output values. It 
    returns true if the light is red and false if the light is either blue or undetected. 
    */ 
    bool GetLightColor() {
        while(true) {
            //Return true if light values indicate red
            if (CdS.Value() > 0 && CdS.Value() < 0.5) {
                return true;
                //Write color to the screen
                LCD.Clear();
                LCD.WriteAt("Red",0,0);
            }
            //Return false if light values indicate blue
            else if (CdS.Value() > 0.9 && CdS.Value() < 1.5) {
                return false;
                //Write color to the screen
                LCD.Clear();
                LCD.WriteAt("Blue",0,0);
            }
            //Default if light values do not indicate a color
            else {
                return false;
                //Write color to the screen
                LCD.Clear();
                LCD.WriteAt("Undetected",0,0);
            }
        }
    }

    /*
    The MoveArmServo function takes in an angle and rotates the servo on the robot's arm to that degree
    */
    void MoveArmServo(float angle) {
        //Move servo to entered position
        arm_servo.SetDegree(angle);
    }

    /*
    The MoveFlipServo function takes in an angle and rotates the servo on the robot's burger flipping mechanism to that 
    degree. 
    */
    void MoveBurgerServo(){
        flip_servo.SetDegree(160);
        Sleep(2.0);
        flip_servo.SetDegree(0);
    }

   /*
   The TranslateWithTime function takes in a time for the motors to be turned on, a power, and a coorinate point relative to the
   robot that is use to calculate an angle of motion. This function is used for aligning with walls when using encoders might
   not be the most efficient method. 
   */
   void TranslateWithTime (float time, float power, float x_pos, float y_pos) {
        //Reset encoder counts
        r_encoder.ResetCounts();
        l_encoder.ResetCounts();
        b_encoder.ResetCounts();

        //Calculate angle of translation
        float angle = atan2(y_pos, x_pos);

        //Set motor powers according to angle
        r_motor.SetPercent(-power * sin(onePIsix - angle));
        l_motor.SetPercent(-power * sin(fivePIsix - angle));
        b_motor.SetPercent(-power * sin(threePItwo - angle));

        Sleep(time);

        r_motor.Stop();
        l_motor.Stop();
        b_motor.Stop();

   }

    /*
    The TranslateWithEncoders function takes in a power along with a distance to be traveled in both the x and y directions,
    and then calculates an angle and exact distance for the robot to move. This will be used when RPS is not available to
    determine the current position of the robot.
    */
    void TranslateWithEncoders(float x_pos, float y_pos, int power) {
        //Reset encoders
        r_encoder.ResetCounts();
        l_encoder.ResetCounts();
        b_encoder.ResetCounts();

        //Calculate distance to a point
        float distance = hypot(x_pos, y_pos);
        
        //Calculate angle of translation
        float angle = atan2(y_pos, x_pos);

        //Set motor powers according to angle
        r_motor.SetPercent(-power * sin(onePIsix - angle));
        l_motor.SetPercent(-power * sin(fivePIsix - angle));
        b_motor.SetPercent(-power * sin(threePItwo - angle));

        //Initialize boolean values for encoders
        bool r_done = false;
        bool l_done = false;
        bool b_done = false;

        //Use Encoders to run the motors until reaching final point
        while (!r_done || !l_done || !b_done) {
            //Set states to stop the motors
            //const float countsperinch = 0.866 * 40.4890175226 * 0.9523; //0.9523 accounts for going too far
            r_done = (r_encoder.Counts() >= (countsperinch * distance * fabs(sin(onePIsix - angle))));
            l_done = (l_encoder.Counts() >= (countsperinch * distance * fabs(sin(fivePIsix - angle))));
            b_done = (b_encoder.Counts() >= (countsperinch * distance * fabs(sin(threePItwo - angle))));

            //Set boolean values to true if motors are going to get stuck
            
            if (fabs(-power * sin(onePIsix - angle)) < 7) {
                r_done = true;
            }
            if (fabs(-power * sin(fivePIsix - angle)) < 7) {
                l_done = true;
            }
            if (fabs(-power * sin(threePItwo - angle)) < 7) {
                b_done = true;
            }
    

            //Stop the motors when states are met
            if (r_done) {
                r_motor.Stop();
            }
            if (l_done) {
                l_motor.Stop();
            }
            if (b_done) {
                b_motor.Stop();
            }
        }
    }

    /*
    The TranslateWithRPS_X function is used to translate the robot to an exact horizontal position without changing its 
    current vertical position. 
    */
    void TranslateWithRPS_X (float end_x_pos, int power) {

       //Use RPS to calculate the distance to a point
       float current_x_pos = RPS.X();

       TranslateWithEncoders(end_x_pos - current_x_pos, 0, power);
    }

    /*
    The TranslateWithRPS_Y function is used to translate the robot to an exact vertical position without changing its 
    current horizontal position.
    */
    void TranslateWithRPS_Y (float end_y_pos, int power) {

       //Use RPS to calculate the distance to a point
       float current_y_pos = RPS.Y();

       TranslateWithEncoders(0, end_y_pos - current_y_pos, power);
    }

    /*
    The TranslateWithRPS function takes in a power along with a final position for the robot, as x and y coordinates, and 
    uses RPS to determine the angle and distance for which the robot needs to move. These calculations are used to set the 
    powers for each individual motor. 
    */
    void TranslateWithRPS(float x_pos, float y_pos, int power) {
        //Reset encoders
        r_encoder.ResetCounts();
        l_encoder.ResetCounts();
        b_encoder.ResetCounts();

        //Use RPS to calculate distance to a point
        float current_x_pos = RPS.X();
        float current_y_pos = RPS.Y();
        float distance = sqrt(pow(x_pos - current_x_pos,2) + pow(y_pos - current_y_pos, 2));

        //Calculate angle of translation
        float angle = atan2(y_pos - current_y_pos, x_pos - current_x_pos);

        //Set motor powers according to angle
        r_motor.SetPercent(-power * sin(onePIsix - angle));
        l_motor.SetPercent(-power * sin(fivePIsix - angle));
        b_motor.SetPercent(-power * sin(threePItwo - angle));

        //Initialize boolean values for encoders
        bool r_done = false;
        bool l_done = false;
        bool b_done = false;

        //Use Encoders to run the motors until reaching final point
        while (!r_done || !l_done || !b_done) {
            //Set states to stop the motors
            //const float countsperinch = 0.866 * 40.4890175226 * 0.9523; //0.9523 accounts for going too far
            r_done = (r_encoder.Counts() >= (countsperinch * distance * sin(onePIsix - angle)));
            l_done = (l_encoder.Counts() >= (countsperinch * distance * sin(fivePIsix - angle)));
            b_done = (b_encoder.Counts() >= (countsperinch * distance * sin(threePItwo - angle)));

            //Stop the motors when states are met
            if (r_done) {
                r_motor.Stop();
            }
            if (l_done) {
                l_motor.Stop();
            }
            if (b_done) {
                b_motor.Stop();
            }
        }
    }

    /*
    The moveForward function was used in the first performance test to move the robot either forward or backward only. This 
    function will not be used in the future. 
    */
    void moveForward(float distance, int speed) {

        r_encoder.ResetCounts();
        l_encoder.ResetCounts();

        //318 / 2(pi)(1.25) = 40.4890175226 counts per inch
        const float adjustedcountsperinch = 0.866 * countsperinch * 0.9523; //0.9523 accounts for going too far
        if (distance < 0) {
            r_motor.SetPercent(-1 * speed);
            l_motor.SetPercent(1 * speed);
        } else  {
            r_motor.SetPercent(speed);
            l_motor.SetPercent(-1 * speed);
        }

        bool r_done = false;
        bool l_done = false;

        while(!r_done || !l_done) {
        //Stop the motors
            r_done = (r_encoder.Counts() >= (fabs(distance) * adjustedcountsperinch));
            l_done = (l_encoder.Counts() >= (fabs(distance) * adjustedcountsperinch));

            if (r_done) {
                r_motor.SetPercent(0);
            }
            if (l_done) {
                l_motor.SetPercent(0);
            }
        }
    }

    /*
    The TurnWithEncoders function takes in an angle in degrees and a motor power, and then uses the encoders to determine when
    the robot has turned the correct amount. This function does the turn relative to the robot's previous position. 
    */
    void TurnWithEncoders(float angle, int power) {
        //Reset encoders
        r_encoder.ResetCounts();
        l_encoder.ResetCounts();
        b_encoder.ResetCounts();
 
        //const float countsperinch = 40.4890175226;
        //diameter of robot = 7.54093496 inch
        //circumference of robot = 23.6905458715 inches
        //counts for a full rotation = 959.206926911
        const float countsPerDegree = 2.66446368586 * (0.9703504);//0.9703504 accounts for overturning
       
        //Turn clockwise if angle is negative
        if (angle < 0) {
            r_motor.SetPercent(-1 * power);
            l_motor.SetPercent(-1 * power);
            b_motor.SetPercent(-1 * power);
        }
        //Turn counterclockwise if angle is positive
        else {
            r_motor.SetPercent(power);
            l_motor.SetPercent(power);
            b_motor.SetPercent(power);
        }
       
        //Use r_encoder to determine when to stop turning (the encoder counts were experimentally tested to be extremely similar)
        while (r_encoder.Counts() < (countsPerDegree * fabs(angle)));

        //Stop the motors
        r_motor.SetPercent(0);
        l_motor.SetPercent(0);
        b_motor.SetPercent(0);
    }

    /*
    The TurnWithRPS function takes in an angle in degrees, relative to the course, and a motor power. It then uses RPS to 
    determine the current heading of the robot, which is then used to find how huch farther the robot must turn to reach the 
    input angle. 
    */
    void TurnWithRPS(float CourseAngle, int power) {
        //Reset encoders
        r_encoder.ResetCounts();
        l_encoder.ResetCounts();
        b_encoder.ResetCounts();
 
        const float countsPerDegree = 2.66446368586 * (0.9703504);//0.9703504 accounts for overturning

        float angle = (CourseAngle - RPS.Heading());
       
        //Turn clockwise if angle is negative
        if (angle < 0) {
            r_motor.SetPercent(-1 * power);
            l_motor.SetPercent(-1 * power);
            b_motor.SetPercent(-1 * power);
        }
        //Turn counterclockwise if angle is positive
        else {
            r_motor.SetPercent(power);
            l_motor.SetPercent(power);
            b_motor.SetPercent(power);
        }
       
        //Use encoders to determine when to stop turning
        while (r_encoder.Counts() < (countsPerDegree * fabs(angle)));
        r_motor.SetPercent(0);
        l_motor.SetPercent(0);
        b_motor.SetPercent(0);
    }

/*
* Takes a coordinate on the robot course and moves the robot there in a straight line
* Turns at 65% of the way there
*/
void RpsGoto(float x, float y, float heading){

    const float sleepTime = 0.2; //Time to sleep between movements so that RPS can update
    const float turnAtPercent = 0.65; //What percentage completion to stop and turn at

    float currentHeading = RPS.Heading();
    //Travel 65% to the point //!This assumes that rps.heading() = 0 when the robot is facing towards +y (upwards)
    TranslateWithEncoders((x-RPS.X()) * turnAtPercent * cos(currentHeading), (y-RPS.Y()) * turnAtPercent * sin(currentHeading), 35);
    Sleep(sleepTime); //Wait so that RPS can update
    
    //Turn roughly to the angle
    currentHeading = RPS.Heading(); //update heading
    TurnWithEncoders(heading-currentHeading, 35);
    Sleep(sleepTime);

    //Travel the rest of the way to the point at a slower speed 
    currentHeading = RPS.Heading(); //update heading
    TranslateWithEncoders((x-RPS.X()) * cos(currentHeading), (y-RPS.Y()) * sin(currentHeading), 25);
    Sleep(sleepTime);

    //Rotate again at a lower speed 
    currentHeading = RPS.Heading(); //update heading
    TurnWithEncoders(heading-currentHeading, 15);
    Sleep(sleepTime);

    //Translate again at a lower seed
    currentHeading = RPS.Heading(); //update heading
    TranslateWithEncoders((x-RPS.X()) * cos(currentHeading), (y-RPS.Y()) * sin(currentHeading), 15);
}

int main(void)
{   
    

/* THIRD PERFRMANCE TEST
    //initialize RPS
    RPS.InitializeTouchMenu();

    //Calibrate the servo
    flip_servo.SetMax(2405);
    flip_servo.SetMin(520);

    //Set the burger flipping servo to 0 at the beginning of run
    flip_servo.SetDegree(0);
    arm_servo.SetDegree(0);

    //Call function to wait for the red light to turn on
    while(!GetLightColor());
    Sleep(0.5);

    //Move to the base of the ramp
    TranslateWithEncoders(0,14.5,25);
    Sleep(0.5);

    //Turn towards the ramp
    TurnWithEncoders(-220,25);

    //Move up the ramp
    TranslateWithEncoders(0,-33,80);
    Sleep(0.5);
    TurnWithRPS(90, 25);
    Sleep(0.5);

    //Move to the right wall
    TranslateWithTime(4.0, 25, 4, 0);
    Sleep(0.5);

    //Move to Burger Plate
    TranslateWithEncoders(0,11,25);
    Sleep(0.5);
    TranslateWithEncoders(-3.5,0,25);
    Sleep(0.5);
    TranslateWithTime(1.0,20, 0, 1);
    Sleep(0.5);

    //back up
    TranslateWithEncoders(0, -4, 25);
    Sleep(0.5);

    //turn
    TurnWithEncoders(180, 25);
    Sleep(0.5);

    //move forward
    TranslateWithEncoders(1.0, 0, 25);
    Sleep(0.5);
    MoveArmServo(0);
    Sleep(0.5);
    TranslateWithEncoders(0, 1.0, 25);
    Sleep(0.5);

    //Flip the burger with servo
    MoveBurgerServo();
*/

/* SECOND PERFORMACE TEST
    //Calibrate the servo
    arm_servo.SetMax(2407);
    arm_servo.SetMin(571);

    //Start servo angle
    MoveArmServo(180);

    //Call function to wait for the red light to turn on
    while(!GetLightColor());
    Sleep(0.5);

    //Move to the base of the ramp
    TranslateWithEncoders(0,14.5,25);
    Sleep(0.5);

    //Turn towards the ramp
    TurnWithEncoders(-220,25);

    //Move up the ramp
    TranslateWithEncoders(0,-33,80);
    Sleep(0.5);
    TurnWithEncoders(180, 25);
    Sleep(0.5);

    //Move sideways to wall
    TranslateWithTime(3.0, 25, -11.5, 0);
    Sleep(0.5);

    //Back into the sink
    TranslateWithTime(2.0, 20, 0, -2); 
    Sleep(0.5);   

    //Use servos to dump the tray
    MoveArmServo(60);
    Sleep(1.0);

    //Move towards the ticket slider
    TranslateWithEncoders(24.5,0,25);
    Sleep(0.5);
    MoveArmServo(0);

    //Back up towards ticket
    TranslateWithEncoders(0,-7.25,25);
    Sleep(0.5);

    //Move ticket forward
    TranslateWithEncoders(-5.5,0,25);
    Sleep(0.5);

    //Back away from ticket
    TranslateWithEncoders(1,0,25);
    Sleep(0.5);

    //Move to burger plate
    TranslateWithEncoders(0,24,25);
    Sleep(0.5);
*/

}







