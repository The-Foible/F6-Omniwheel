#include "Full_Run.h"

#include <FEHRPS.h>
#include <FEHLCD.h>
#include <FEHUtility.h>

/********************************
**Full code for running the course
/********************************/

//This function is called from the main menu
void FullRun(){
    LCD.Clear();
    float i = 3;
    while(i > 0){
        LCD.WriteAt(i,0,0);
        Sleep(0.01);
        i-=0.01;
    }
}