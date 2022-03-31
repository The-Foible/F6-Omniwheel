#include <FEHLCD.h>
#include <FEHIO.h>
#include <FEHMotor.h>
#include <FEHRPS.h>
#include <FEHUtility.h>
#include <FEHServo.h>
#include <FEHUtility.h>
#include <FEHSD.h>
#define _USE_MATH_DEFINES //Use M_PI
#include <math.h>

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
                LCD.WriteAt(RPS.Heading(),5,20);
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
    degree. The function also implements a code to slow down the motion of the servo. 
    */
    void MoveBurgerServo(){
        //Initialize a variable for degree
        float degree = 0;

        //Loop to move the servo one degree each hundredth of a second
        while (degree < 160) {
            degree = degree + 1;
            flip_servo.SetDegree(degree);
            Sleep(10);
        }

        //Sleep for two seconds when servo reaches its highest position
        Sleep(1.0);

        //Loop to move the servo negative one degree eah hundredth of a second
        while (degree > 0) {
            degree = degree - 1;
            flip_servo.SetDegree(degree);
            Sleep(10);
        }
    }

    //The GetRPS function populates the value pointed to by each POINTER argument with the relevant RPS value
    //For any values you don't need for a use, populate the parameters with 0
    //GetRPS(0, 0, &current_heading); //Gets heading and ignores X and Y positions
    int GetRPS(float *x = 0, float *y = 0, float *heading = 0){
        float start_time = TimeNow();
        //Loop until RPS returns a non-error value
        while(RPS.Heading() < 0){
            //Specify the error value
            if(RPS.Heading() == -1){
                LCD.WriteLine("QR CODE NOT FOUND");
            } else {
                LCD.WriteLine("DEADZONE");
            }

            //Check how long it's been looping and break if it's more than 5s
            if((TimeNow() - start_time) > 5.0 ){
                return 1;
            }
            Sleep(10);
        }

        //Populate the value of any fields given
        if(x != 0){
            *x = RPS.X();
        }
        if(x != 0){
            *y = RPS.Y();
        }
        if(x != 0){
            *heading = RPS.Heading();
        }

        return 0;
    }

   /*
   The TranslateWithTime function takes in a time for the motors to be turned on, a power, and a coorinate point relative to the
   robot that is use to calculate an angle of motion. This function is used for aligning with walls when using encoders might
   not be the most efficient method. 
   */
   void TranslateWithTime (float time, float power, float x_pos, float y_pos) {
        //Calculate angle of translation
        float angle = atan2(y_pos, x_pos);

        //Initialize variables for motor power
        float r_pow, l_pow, b_pow;
        r_pow = -power * sin(onePIsix - angle);
        l_pow = -power * sin(fivePIsix - angle);
        b_pow = -power * sin(threePItwo - angle);

        //Adjust right motor power
        if (r_pow > 0) {
            r_pow = r_pow + 5.58;
        }
        else if (r_pow < 0) {
            r_pow = r_pow - 8.03;
        }
        //Adjust left motor power
        if (l_pow > 0) {
            l_pow = l_pow + 5.74;
        }
        else if (l_pow < 0) {
            l_pow = l_pow - 6.56;
        }
        //Adjust back motor power
        if (b_pow > 0) {
            b_pow = b_pow + 6.5;
        }
        else if (b_pow < 0) {
            b_pow = b_pow - 8.9;
        }

        //Set motor powers according to angle
        r_motor.SetPercent(r_pow);
        l_motor.SetPercent(l_pow);
        b_motor.SetPercent(b_pow);

        //Sleep for input time
        Sleep(time);

        //Stop the motors when time is reached
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
        const float rotation_correction_factor = -0.07; //Tuning constant to make the robot translate straight. larger is stronger
        //const float rotation_correction_factor = 0.0; //Tuning constant to make the robot translate straight. larger is stronger

        //Reset encoders
        r_encoder.ResetCounts();
        l_encoder.ResetCounts();
        b_encoder.ResetCounts();

        //Calculate distance to a point
        float distance = hypot(x_pos, y_pos);
        
        //Calculate angle of translation
        float angle = atan2(y_pos, x_pos);

        //Initialize and calculate variables for motor power
        float rotation_correction_power = rotation_correction_factor * cos(3 * angle);

        float r_pow = -power * (sin(onePIsix   - angle) + rotation_correction_power);
        float l_pow = -power * (sin(fivePIsix  - angle) + rotation_correction_power);
        float b_pow = -power * (sin(threePItwo - angle) + rotation_correction_power);

        //Adjust right motor power
        if (r_pow > 0) {
            r_pow = r_pow + 5.58;
        }
        else if (r_pow < 0) {
            r_pow = r_pow - 8.03;
        }
        //Adjust left motor power
        if (l_pow > 0) {
            l_pow = l_pow + 5.74;
        }
        else if (l_pow < 0) {
            l_pow = l_pow - 6.56;
        }
        //Adjust back motor power
        if (b_pow > 0) {
            b_pow = b_pow + 6.5;
        }
        else if (b_pow < 0) {
            b_pow = b_pow - 8.90;
        }

        //Set motor powers according to angle
        r_motor.SetPercent(r_pow);
        l_motor.SetPercent(l_pow);
        b_motor.SetPercent(b_pow);

        //Initialize boolean values for encoders
        bool r_done = false;
        bool l_done = false;
        bool b_done = false;

        //Initalize start time
        float start_time = TimeNow();

        //Use Encoders to run the motors until reaching final point
        while (!r_done || !l_done || !b_done) {
            //Set states to stop the motors
            //const float countsperinch = 0.866 * 40.4890175226 * 0.9523; //0.9523 accounts for going too far
            r_done = (r_encoder.Counts() >= (countsperinch * distance * fabs(sin(onePIsix - angle))));
            l_done = (l_encoder.Counts() >= (countsperinch * distance * fabs(sin(fivePIsix - angle))));
            b_done = (b_encoder.Counts() >= (countsperinch * distance * fabs(sin(threePItwo - angle))));
    
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

            //If the robot takes longer than three seconds to translate, give it a temporary boost of power
            if(TimeNow() - start_time > 3.0){
                //Boost power
                r_motor.SetPercent(r_pow * 1.5);
                l_motor.SetPercent(l_pow * 1.5);
                b_motor.SetPercent(b_pow * 1.5);
                //For 100 ms
                Sleep(0.1);
                //Return power to normal value
                r_motor.SetPercent(r_pow);
                l_motor.SetPercent(l_pow);
                b_motor.SetPercent(b_pow);
                //Reset time so this doesn't loop
                start_time = TimeNow();
            }
        }
    }

    /*
    The TranslateWithRPS_X function is used to translate the robot to an exact horizontal position without changing its 
    current vertical position. This should not need to be used in the final run. 
    */
    void TranslateWithRPS_X (float end_x_pos, int power) {

       //Use RPS to calculate the distance to a point
       float current_x_pos = RPS.X();

       TranslateWithEncoders(end_x_pos - current_x_pos, 0, power);
    }

    /*
    The TranslateWithRPS_Y function is used to translate the robot to an exact vertical position without changing its 
    current horizontal position. This should not need to be used in the final run. 
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
        float current_x_pos = -3;
        float current_y_pos = -3;
        float current_heading = -3;

        float start_time = TimeNow();
        //Loops until the robot is less than half an inch from the goal or 5 seconds have past
        while((hypot(x_pos - RPS.X(), y_pos - RPS.Y()) > 0.5) && (start_time-TimeNow() < 5.0 )){
            //GetRPS(&current_x_pos, &current_y_pos, &current_heading);
            current_x_pos = RPS.X();
            current_y_pos = RPS.Y();
            current_heading = (RPS.Heading()-90)*M_PI/180; //Convert to radians
        
            //calculate the distance to travel in x and y
            float x_dif = x_pos - current_x_pos;
            float y_dif = y_pos - current_y_pos;

            //Rotate the translation vector based on the angle of the robot
            float x_adjusted = x_dif * cos(current_heading) - y_dif * sin(current_heading);
            float y_adjusted = x_dif * sin(current_heading) + y_dif * cos(current_heading);

            LCD.Clear();
            LCD.WriteAt("heading (rad):",0,0);
            LCD.WriteAt(current_heading,180,0);
            LCD.WriteAt("x:",0,20);
            LCD.WriteAt(current_x_pos,180,20);
            LCD.WriteAt("y:",0,40);
            LCD.WriteAt(current_y_pos,180,40);
            LCD.WriteAt("x_dif:",0,60);
            LCD.WriteAt(x_dif,180,60);
            LCD.WriteAt("y_dif:",0,80);
            LCD.WriteAt(y_dif,180,80);
            LCD.WriteAt("x_adjusted:",0,100);
            LCD.WriteAt(x_adjusted,120,100);
            LCD.WriteAt("y_adjusted:",0,120);
            LCD.WriteAt(y_adjusted,120,120);
            LCD.WriteAt("heading (deg):",0,140);
            LCD.WriteAt(current_heading*180/M_PI,180,140);
            LCD.WriteAt("distance from goal:",0,160);
            LCD.WriteAt(current_heading*180/M_PI,200,160);

            //Call the encoder movement function using adjusted RPS values
            TranslateWithEncoders(x_adjusted, y_adjusted, power);

            Sleep(0.25);
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

        angle = fmod(angle + 180.0, 360.0) - 180;
 
        const float constant_distance_mod = 0.9; //How many more degrees to turn for a given angle //0.0 is no change
        const float proportional_power_mod = -0.27; //How much more or less to turn based on the power (linear) //0.0 is no change
        const float quadratic_power_mod = 0.0; //A quadratic term affecting more or less degrees to turn based on power (due to rotational kinetic energy increasing at the square of velocity) //0.0 is no change
        //const float countsperinch = 40.4890175226;
        //diameter of robot = 7.54093496 inch
        //circumference of robot = 23.6905458715 inches
        //counts for a full rotation = 959.206926911
        //const float countsPerDegree = 2.66446368586;
        const float countsPerDegree = 2.66446368586;

        int target_encoder_counts = ((fabs(angle) + power * power * quadratic_power_mod + power * proportional_power_mod + constant_distance_mod) * countsPerDegree);
       
        //Turn clockwise if angle is negative
        if (angle < 0) {
            r_motor.SetPercent(-power);
            l_motor.SetPercent(-power);
            b_motor.SetPercent(-power);
        }
        //Turn counterclockwise if angle is positive
        else {
            r_motor.SetPercent(power);
            l_motor.SetPercent(power);
            b_motor.SetPercent(power);
        }
       
        //Use r_encoder to determine when to stop turning (the encoder counts were experimentally tested to be extremely similar)
        while (r_encoder.Counts() <= target_encoder_counts);

        //Stop the motors
        r_motor.SetPercent(0);
        l_motor.SetPercent(0);
        b_motor.SetPercent(0);
    }

    /*
    The TurnCalibrate function logs relevant data about robot turns so that the motor power can be adjusted for accurate turning.
    */
    void TurnCalibrate(float angle, float power, FEHFile *sd) {

        //Reset encoders
        r_encoder.ResetCounts();
        l_encoder.ResetCounts();
        b_encoder.ResetCounts();

        float initial_RPS_heading = RPS.Heading();

        //Tuning constants (experimentally calculated)
        // const float constant_distance_mod = 1.57; //How many more degrees to turn for a given angle //0.0 is no change
        // const float proportional_power_mod = -0.1555; //How much more or less to turn based on the power (linear) //0.0 is no change
        // const float quadratic_power_mod = -0.0027; //A quadratic term affecting more or less degrees to turn based on power (due to rotational kinetic energy increasing at the square of velocity) //0.0 is no change
        const float constant_distance_mod = 0.9; //How many more degrees to turn for a given angle //0.0 is no change
        const float proportional_power_mod = -0.27; //How much more or less to turn based on the power (linear) //0.0 is no change
        const float quadratic_power_mod = 0.0; //A quadratic term affecting more or less degrees to turn based on power (due to rotational kinetic energy increasing at the square of velocity) //0.0 is no change
        //const float countsperinch = 40.4890175226;
        //diameter of robot = 7.54093496 inch
        //circumference of robot = 23.6905458715 inches
        //counts for a full rotation = 959.206926911
        //const float countsPerDegree = 2.66446368586;
        const float countsPerDegree = 2.66446368586;
        
        //Calculate goal number of encoder steps
        int target_encoder_counts = ((fabs(angle) + power * power * quadratic_power_mod + power * proportional_power_mod + constant_distance_mod) * countsPerDegree);

        //Turn clockwise if angle is negative
        if (angle < 0) {
            r_motor.SetPercent(-power);
            l_motor.SetPercent(-power);
            b_motor.SetPercent(-power);
        }
        //Turn counterclockwise if angle is positive
        else {
            r_motor.SetPercent(power);
            l_motor.SetPercent(power);
            b_motor.SetPercent(power);
        }

        //Use b_encoder to determine when to stop turning (the encoder counts were experimentally tested to be extremely similar)
        while (b_encoder.Counts() <= target_encoder_counts);


        //Stop the motors
        r_motor.SetPercent(0);
        l_motor.SetPercent(0);
        b_motor.SetPercent(0);

        Sleep(0.5);

        SD.FPrintf(sd, "%f, %f, %d, %d, %d, %d, %d, %f, %f, %f\n", power, angle, (int) (fabs(angle) * countsPerDegree), target_encoder_counts, b_encoder.Counts(), r_encoder.Counts(), l_encoder.Counts(), initial_RPS_heading, RPS.Heading(), fmod(RPS.Heading()-initial_RPS_heading+360, 360));
    }

    /*
    The TurnWithRPS function takes in an angle in degrees, relative to the course, and a motor power. It then uses RPS to 
    determine the current heading of the robot, which is then used to find how huch farther the robot must turn to reach the 
    input angle. 
    */
    void TurnWithRPS(float angle, int power) {
        float start_time = TimeNow();
        float current_heading;
        //Loops until the robot is less than two degrees from the goal or 5 seconds have past
        while((fabs(angle - RPS.Heading()) > 2 ) && (start_time-TimeNow() < 5.0 )){
            //GetRPS(0, 0, &current_heading);
            current_heading = RPS.Heading();
            TurnWithEncoders(angle - current_heading, power);
            //Use a lower power on all subsequent runs
            power = 10;
            //Wait for RPS to update
            Sleep(0.25);
        }
    }

    /*
    The PrintRPS function prints all RPS outputs to the screen so that they can be called in other functions.
    */
    void PrintRPS () {
        LCD.Clear();
        LCD.WriteAt("x = ", 0, 0);
        LCD.WriteAt(RPS.X(), 30, 0);
        LCD.WriteAt("y = ", 0, 20);
        LCD.WriteAt(RPS.Y(), 30, 20);
        LCD.WriteAt("angle = ", 0, 40);
        LCD.WriteAt(RPS.Heading(), 50, 40);
    }


void MotorCalibration(FEHMotor m, DigitalEncoder e, char *filename){
    FEHFile *sd = SD.FOpen(filename, "w");
    SD.FPrintf(sd, "power, counts/s\n");
    for(int i = -100; i <= 100; i+=5){
        m.SetPercent(i);
        Sleep(0.2);
        e.ResetCounts();
        Sleep(1.0);
        SD.FPrintf(sd, "%d, %d\n", i, e.Counts());
    }
    m.SetPercent(0);
    SD.FClose(sd);
}

int main(void)
{   
    
    //Calibrate the servos
    flip_servo.SetMax(2350);
    flip_servo.SetMin(520);
    arm_servo.SetMax(2370);
    arm_servo.SetMin(571);

    //Set the servos to 0 at the beginning of run
    flip_servo.SetDegree(0);
    flip_servo.Off(); //Turn the servo off until it's needed
    arm_servo.SetDegree(180);
    
    //initialize RPS
    RPS.InitializeTouchMenu();
    while(1){
        LCD.WriteLine("TRANSLATING(18,18)");
        TranslateWithRPS(18,18,25);
        // Sleep(1.0);
        //TurnWithRPS(90, 25);
        Sleep(5.0);
    }

    //* Encoder turn calibration start

    // FEHFile *sd = SD.FOpen("turn.csv", "w");
    // SD.FPrintf(sd, "power, target angle, target eSteps, modified eSteps, experimental eSteps B, experimental eSteps R, experimental eSteps L, start angle, end angle, calculated angle (aprox)\n");

    // LCD.SetFontColor(BLACK);
    // for(int i = 10; i <= 40; i+=5){
    //     for(int j = -135; j<= 135; j+= 45){
    //             LCD.Clear(WHITE);
    //             LCD.WriteAt("pow:",0,0);
    //             LCD.WriteAt("ang:",0,20);
    //             LCD.WriteAt(i,50,0);
    //             LCD.WriteAt(j,50,20);
    //         if(j!=0){
    //             TurnWithRPS(180, 25);
    //             Sleep(0.5);
    //             TurnCalibrate(j, i, sd);
    //         }
    //     }
    // }

    // SD.FClose(sd);

    // LCD.Clear(WHITE);
    // LCD.WriteLine("DONE");

    // return 0;
    

    //* Encoder turn calibration end

    /*
    int timeCount = 0;
    
    while (timeCount < 5) {
        PrintRPS();
        Sleep(1.0);
        timeCount++;
    }
    */

    //Get RPS values for jukebox light
    float xpos,ypos;
    while (!LCD.Touch(&xpos, &ypos)) {
        PrintRPS();
        Sleep(0.5);
    }

    float jukeboxX, jukeboxY;
    jukeboxX = RPS.X();
    jukeboxY = RPS.Y();
    LCD.Clear();
    LCD.WriteAt(jukeboxX, 5, 5);
    LCD.WriteAt(jukeboxY, 5, 25);
    Sleep(2.0);

    //Get RPS values for burger wheel
    while (!LCD.Touch(&xpos, &ypos)) {
        PrintRPS();
        Sleep(0.5);
    }

    float burgerX, burgerY;
    burgerX = RPS.X();
    burgerY = RPS.Y();
    LCD.Clear();
    LCD.WriteAt(burgerX, 5, 5);
    LCD.WriteAt(burgerY, 5, 25);

    //Wait for the starting light
    while(!GetLightColor());
    Sleep(0.5);

    //Move towards jukebox light
    TranslateWithRPS(16,15.2,25);
    Sleep(0.25);

    //Move to jukebox light
    TranslateWithRPS(jukeboxX,jukeboxY,25);
    Sleep(0.5);
    TranslateWithRPS(jukeboxX,jukeboxY,15);
    Sleep(0.5);
    
    //Get the light color and press correct button
    if(GetLightColor()) {
        //RED
        TurnWithRPS(180,25);
        Sleep(0.25);
        TranslateWithRPS(7.4,12,25);
        Sleep(0.25);
        TranslateWithTime(0.25,15,-1,0);

    } else {
        //BLUE
        TurnWithRPS(180,25);
        Sleep(0.25);
        TranslateWithRPS(10.6,12,25);
        Sleep(0.25);
        TranslateWithTime(0.25,15,-1,0);
    }
    Sleep(0.5);

    //Move to the base of the ramp
    TranslateWithRPS(18, 18, 25);
    Sleep(0.25);

    //Turn towards the ramp
    TurnWithRPS(270, 25);
    Sleep(0.25);

    //Move up the ramp and turn to face forward   
    TranslateWithEncoders(0,-33,70);
    Sleep(0.25);
    TurnWithRPS(90, 25);         
    Sleep(0.25);

    //Determine correct ice cream lever
    LCD.Clear();
    if (RPS.GetIceCream() == 0) {
        //Move to the vanilla lever path
        LCD.Write("Vanilla");
        TranslateWithRPS(15.3, 52.9,40);
        Sleep(0.25);
    }
    else if (RPS.GetIceCream() == 1) {
        //Move to the twist lever path
        LCD.Write("Twist");
        TranslateWithRPS(18.3, 56, 40);
        Sleep(0.25);
    }
    else if (RPS.GetIceCream() == 2){
        //Move to the chocolate lever path
        LCD.Write("Chocolate");
        TranslateWithRPS(20.5, 59, 40);
        Sleep(0.25);
    }

    //Turn towards lever
    TurnWithRPS(315, 25);
    Sleep(0.25);

    //Move forward to the levers
    TranslateWithEncoders(0,-7,20);
    Sleep(0.5);
    
    //Flip the correct lever down
    MoveArmServo(75);
    Sleep(0.5);

    //Back off lever
    TranslateWithEncoders(0,7.5,25);

    //Move servo back up
    MoveArmServo(170);

    //Go to sink
    TurnWithRPS(90,25);
    Sleep(0.25);
    TranslateWithRPS(9 ,49 ,40);
    Sleep(0.25);
    TranslateWithTime(0.5, 25, -1, 0);
    TranslateWithTime(1.0, 20, 0, -1);

    //Dump tray in sink
    MoveArmServo(40);
    Sleep(0.5);

    //Move back to correct lever
    LCD.Clear();
    if (RPS.GetIceCream() == 0) {
        //Move to the vanilla lever path
        LCD.Write("Vanilla");
        TranslateWithRPS(15.3, 52.9, 40);
        Sleep(0.25);
    }
    else if (RPS.GetIceCream() == 1) {
        //Move to the twist lever path
        LCD.Write("Twist");
        TranslateWithRPS(18.3, 56, 40);
        Sleep(0.25);
    }
    else if (RPS.GetIceCream() == 2){
        //Move to the chocolate lever path
        LCD.Write("Chocolate");
        TranslateWithRPS(20.5, 59, 40);
        Sleep(0.25);
    }

    //Turn towards lever
    TurnWithRPS(315, 25);
    Sleep(0.25);
    
    //Move forward to the lever
    Sleep(0.25);
    TranslateWithEncoders(0,-7,20);
    Sleep(0.25);

    //Flip ice cream lever up 
    MoveArmServo(85);
    Sleep(0.25);

    //Move away from ice cream
    TranslateWithEncoders(0,8,25);
    Sleep(0.25);
    TurnWithRPS(90,25);
    MoveArmServo(0);
    Sleep(0.25);

    //Move to ticket
    TranslateWithRPS(28,50,40);
    Sleep(0.25);
    TranslateWithRPS(31.5,43.2,40);
    Sleep(0.25);
    TranslateWithTime(1.0,25,1,-1);
    Sleep(0.25);
    
    //Move ticket over
    TranslateWithEncoders(-5,0,25);
    Sleep(0.25);

    //Translate away from ticket
    TranslateWithRPS(28.6,54.8,25);
    Sleep(0.25);

    //Move to burge wheel (close to (27.9,62.7))
    TranslateWithRPS(burgerX,burgerY,25);
    Sleep(0.25);

    //Use servo to flip the burger
    MoveBurgerServo();

    //Back off the burger wheel
    TranslateWithEncoders(0,-5,25);
    Sleep(0.25);

    //Move to the top of the ramp
    TranslateWithRPS(18,46,25);
    Sleep(0.25);

    //Go down the ramp
    TranslateWithRPS(18,20,25);
    Sleep(0.25);

    //Move to final button
    TurnWithRPS(135,25);
    Sleep(0.25);
    TranslateWithTime(10.0,25,0,-1);

    Sleep(3.0);
    //Get RPS Values
    while (1) {
        PrintRPS();
        Sleep(1.0);
    }


/* FOURTH PERFORMANCE TEST
    //Move to the base of the ramp
    TranslateWithEncoders(0,13.5,25);
    //TranslateWithRPS(18, 18, 25);
    Sleep(0.5);

    //Turn towards the ramp
    TurnWithRPS(270, 25);
    Sleep(0.5);

    //Move up the ramp
    TranslateWithEncoders(0,-33,70);
    Sleep(0.5);
    TurnWithRPS(90, 25);
    Sleep(0.5);

    //Determine which ice cream lever needs to be flipped
    LCD.Clear();
    LCD.Write(RPS.GetIceCream());
    if (RPS.GetIceCream() == 0) {
        //Move to the vanilla lever
        TranslateWithRPS(15.3, 52.9, 25);
        Sleep(0.5);
        TurnWithRPS(315, 25);
    }
    else if (RPS.GetIceCream() == 1) {
        //Move to the twist lever
        TranslateWithRPS(18.3, 56, 25);
        Sleep(0.5);
        TurnWithRPS(315, 25);
    }
    else if (RPS.GetIceCream() == 2){
        //Move to the chocolate lever
        TranslateWithRPS(20.5, 59, 25);
        Sleep(0.5);
        TurnWithRPS(315, 25);
    }

    //Move forward to the levers
    Sleep(0.5);
    TranslateWithEncoders(0,-6.5,20);
    Sleep(0.5);
    
    //Flip the correct lever down
    MoveArmServo(80);
    Sleep(0.5);

    //Back away from the lever
    TranslateWithEncoders(0, 3, 20);
    Sleep(7.0);

    //Move Servo down
    MoveArmServo(45);
    Sleep(0.5);

    //Move towards lever
    TranslateWithEncoders(0, -3, 20);
    Sleep(0.5);

    //Push lever up
    MoveArmServo(85);
    Sleep(0.5);
    
    //Move to the ramp
    TranslateWithEncoders(0,7,25);
    Sleep(0.5);
    TurnWithRPS(90,25);
    Sleep(0.5);
    TranslateWithRPS(17, 42, 25);
    Sleep(0.5);
    TurnWithRPS(90,25);
    Sleep(0.5);
    MoveArmServo(180);
    Sleep(0.5);

    //Go down the ramp
    TranslateWithEncoders(0, -12, 25);
    Sleep(0.5);

    //Travel to starting position
    TranslateWithRPS(18, 18, 25);
    Sleep(0.5);
    TurnWithRPS(135, 25);
    Sleep(0.5);

    //Press final button
    TranslateWithTime(5.0, 15, 0, -1);
*/
    
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

    //* Encoder turn calibration start

    // FEHFile *sd = SD.FOpen("turn.csv", "w");
    // SD.FPrintf(sd, "power, target angle, target eSteps, modified eSteps, experimental eSteps B, experimental eSteps R, experimental eSteps L, calculated angle (aprox)\n");

    // LCD.SetFontColor(BLACK);
    // for(int i = 10; i <= 40; i+=5){
    //     for(int j = -180; j<= 180; j+= 90){
    //             LCD.Clear(WHITE);
    //             LCD.WriteAt("pow:",0,0);
    //             LCD.WriteAt("ang:",0,20);
    //             LCD.WriteAt(i,50,0);
    //             LCD.WriteAt(j,50,20);
    //         if(j!=0){
    //             TurnCalibrate(j, i, sd);
    //         }
    //     }
    // }

    // SD.FClose(sd);

    // LCD.Clear(WHITE);
    // LCD.WriteLine("DONE");

    // return 0;

    //* Encoder turn calibration end

    //* Encoder translation calibration start
    // TranslateWithEncoders(-2.7, -17, 25);
    // TranslateWithEncoders(20*cos(0), 20*sin(0), 25);
    // TranslateWithEncoders(20*cos(M_PI/12), 20*sin(M_PI/12), 25);
    // //Figure 8 code
    // while(1){
    // for(float a = 0; a < M_PI; a+=M_PI/24){
    //     TranslateWithEncoders(cos(a), sin(a), 20);
    // }
    // for(float a = M_PI; a > -M_PI; a-=M_PI/24){
    //     TranslateWithEncoders(cos(a), sin(a), 20);
    // }
    //     for(float a = M_PI; a < 2*M_PI; a+=M_PI/24){
    //     TranslateWithEncoders(cos(a), sin(a), 20);
    // }
    // }
    // return 0;
    //* Encoder translation calibration end

}





