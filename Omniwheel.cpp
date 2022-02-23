#include "Omniwheel.h"

#include <FEHMotor.h>

//Define the number of encoder steps per linear inch traveled by the wheels
//todo calculate this, then measure it at whatever power we decide to use to fine-tune the accuracy
#define STEPS_PER_INCH = 40

void absoluteMove(float final_x, float final_y, float final_heading, float power){

}
//moves the robot from it's current RPS location to the final RPS location
//Wrapper for relativeMove
//Recomend multiple iterations at decreasing power levels for more accuracy (probably 1 at 100%, 1 at 50%, and one at 20% would be good. requires testing though)

/*ex:
relativeMove(final_x-RPS.get_x(), final_y-RPS.get_y(), final_heading-RPS.get_heading(), 100);
relativeMove(final_x-RPS.get_x(), final_y-RPS.get_y(), final_heading-RPS.get_heading(), 50);
relativeMove(final_x-RPS.get_x(), final_y-RPS.get_y(), final_heading-RPS.get_heading(), 20);
*/

//Precisely translate and use the encoders
void relativeTranslateEncoder(float dx, float dy, float power){

}
//Slow down motors near the final position

//Precisely rotate using the encoders using angle in //!RADIANS
void relativeRotateEncoder(float angle, float power){

}
//Slow down motors near the final position


//i and j instead of x and y since the movement is relative to the robot's orientation
void relativeMove(float di, float dj, float dheading, float power, DigitalInputPin bump_switch){

}
//moves the robot a certain distance just by controlling the power of the motors
//Does not use encoders. (Might calculate speed from the distance needed to travel and then go for that long, idk)
//Can be used with bump switches by giving a non-void bump_switch argument
//Probably the harest to work mathematically, but by far the most important to smooth opperation
//If this never works, it can kinda be substituted by chaining encoder steps, it'll just be slower and way less cool
//todo do this one last if we have time