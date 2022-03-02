#include <FEHLCD.h>
#include <FEHIO.h>
#include <FEHLCD.h>
#include <FEHMotor.h>
#include <FEHRPS.h>
#include <FEHUtility.h>
#include <FEHServo.h>
#include <FEHUtility.h>
#include <math.h>
#include <time.h>

#define _USE_MATH_DEFINES //Use M_PI

#define fivePIsix 2.61799387799
#define threePItwo 4.71238898038
#define onePIsix 0.52359877559
 
DigitalEncoder r_encoder(FEHIO::P0_2);
DigitalEncoder l_encoder(FEHIO::P0_0);
DigitalEncoder b_encoder(FEHIO::P0_1);

AnalogInputPin CdS(FEHIO::P3_0);

FEHMotor r_motor(FEHMotor::Motor0, 9.0);
FEHMotor l_motor(FEHMotor::Motor1, 9.0);
FEHMotor b_motor(FEHMotor::Motor2, 9.0);

FEHServo arm_servo(FEHServo::Servo0);

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
    The MoveArmServo function takes in an angle to rotate the servo to  
    */
    void MoveArmServo(float angle, float time) {
        //USE FOR CALIBRATION BEFORE SERVO USE
        /*
        arm_servo.Touchalibrate();
        arm_servo.SetMax();
        arm_servo.SetMin();
        */
        
        //Move servo to entered position
        arm_servo.SetDegree(angle);
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

        const float countsperinch = 40.4890175226;

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
        float distance = sqrt(pow(x_pos,2) + pow(y_pos,2));
        
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

        const float countsperinch = 40.4890175226;

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

    //Function to move robot in y direction
    void moveForward(float distance, int speed) {

        r_encoder.ResetCounts();
        l_encoder.ResetCounts();

        //318 / 2(pi)(1.25) = 40.4890175226 counts per inch
        const float countsperinch = 0.866 * 40.4890175226 * 0.9523; //0.9523 accounts for going too far
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
            r_done = (r_encoder.Counts() >= (fabs(distance) * countsperinch));
            l_done = (l_encoder.Counts() >= (fabs(distance) * countsperinch));

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
       
        //Use encoders to determine when to stop turning
        while (r_encoder.Counts() < (countsPerDegree * fabs(angle)));
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

int main(void)
{
    //Set all motors to zero at the start of the run
    /*
    l_motor.SetPercent(0);
    r_motor.SetPercent(0);
    b_motor.SetPercent(0);
    */


    //Call function to wait for the red light to turn on
    while(!GetLightColor());
    Sleep(0.5);

    

   
    

/*
        //Call function to move forward 8 inches
        moveForward(9.5,25);

   

        Sleep(0.5);

        //Turn left 45 degrees

        turn(41.5, 25);

        Sleep(0.5);
 
        LCD.WriteAt("end",0,50);
        //Call function to move left 11.5 inches

        moveForward(10.75, 25);
 

        //Determine color of the light sensor

 

        bool color = GetLightColor();
        if (color) {
            LCD.WriteAt("red",0,0);
        }
        else {
            LCD.WriteAt("blue",0,0);
        }
        //LCD.WriteAt(color, 0,0);
       

   
        Sleep(0.5);
 

        //move to correct button

        if (color == 1) {

            //turn right 90 and move forward to red button
            moveForward(3.5, 25);
            Sleep(0.5);
            turn(90,25);
            Sleep(0.5);
            moveForward(2,18);
            Sleep(0.5);
            moveForward(2,25);
            Sleep(0.5);




            //go back to original spot
            //moveForward(-0.5,25);
            //Sleep(0.5);
            moveForward(-2.5,18);
            Sleep(0.5);
            turn(85,25);
            Sleep(0.5);
            moveForward(3.5,25);



            //change after performance test one

            Sleep(0.5);
            moveForward(9.5,25);
            Sleep(0.5);
            turn(80,25);
            Sleep(0.5);
            moveForward(40,60);
            Sleep(1.0);
            moveForward(-40,40);
            LCD.WriteAt("done",50,50);





        }

        else {


            turn(80,25);
            Sleep(0.5);
            moveForward(2,18);
            Sleep(0.5);
            turn(5,25);
            Sleep(0.5);
            moveForward(2,25);
            Sleep(0.5);



            //reverse movements

            moveForward(-0.5,25);
            Sleep(0.5);
            turn(5,25);
            Sleep(0.5);
            moveForward(-2,18);
            Sleep(0.5);
            turn(85,25);
            Sleep(0.5);



            //change after performance test one



            Sleep(0.5);
            moveForward(8.5,25);
            Sleep(0.5);
            turn(70,25);
            Sleep(0.5);
            moveForward(40,60);
            Sleep(1.0);
            moveForward(-40,40);
            LCD.WriteAt("done",50,50);


        }

 

    return(0);

    */

}

