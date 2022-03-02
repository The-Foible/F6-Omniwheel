#include "Menu.h"
#include "Full_Run.h"
#include "Utility.h"
#include <FEHLCD.h>
#include <FEHBattery.h>


//Define which program is run by which menu
#define RUN_FULL_FUNCTION FullRun()

//Based on Proteus Test Code

//Main Menu
int MenuMain()
{   
    LCD.Clear(BACKGROUND_C);

    /* Create icons for main menu */
    FEHIcon::Icon MAIN_T[1];
    char main_t_label[1][20] = {"TEAM F6: GRUBBOTS"};
    FEHIcon::DrawIconArray(MAIN_T, 1, 1, 1, 201, 1, 1, main_t_label, HI_C, TEXT_C);
    MAIN_T[0].Select();

    FEHIcon::Icon MAIN[8];
    //Label for each button
    char main_label[8][20] = {"Run Main", "Program 2", "Program 3", "Program 4", "Function 5", "Function 6", "Function 7", "Function 8"};
    FEHIcon::DrawIconArray(MAIN, 4, 2, 40, 20, 1, 1, main_label, MENU_C, TEXT_C);

    LCD.SetFontColor(TEXT_C);
    LCD.WriteAt("BATT:        V", 0, 222);

    //Buzzer.Buzz(beep_t);

    int menu=MENU_MAIN, n;
    float x, y;
    float m = 0, bat_v = 0;

    while(menu==MENU_MAIN)
    {
        /* Display average battery voltage to screen */
        bat_v = ((bat_v*m)+Battery.Voltage());
        bat_v = bat_v/(++m);
        LCD.WriteAt(bat_v, 72, 222);
        if (LCD.Touch(&x, &y))
        {
            /* Check to see if a main menu icon has been touched */
            for (n=0; n<=7; n++)
            {
                if (MAIN[n].Pressed(x, y, 0))
                {
                    menu = n+1;
                    MAIN[n].WhilePressed(x, y);
                    break;
                }
            }
        }
    }
    return menu;
}

//Run full program as soon as the box is clicked (since this program is used so often)
int MenuProg1(){
    LCD.Clear(BACKGROUND_C);

    RUN_FULL_FUNCTION;

    return MENU_MAIN;
}

int MenuProg2() { //! Currently does the same thing as MenuRunFull, but with a run/back menu (for testing)
    LCD.Clear(BACKGROUND_C);

    //Create back button
    FEHIcon::Icon Back[1];
    char back_label[1][20] = {"<-"};
    FEHIcon::DrawIconArray(Back, 1, 1, 1, 200, 1, 260, back_label, MENU_C, TEXT_C);

    //Create run button
    FEHIcon::Icon Run[1];
    char run_label[1][20] = {"Run"};
    FEHIcon::DrawIconArray(Run, 1, 1, 70, 70, 110, 110, run_label, MENU_C, FORESTGREEN);

    int menu=MENU_PROG1, n, m;
    float x, y;
    int run[4] = {0, 0, 0, 0};

    while(menu==MENU_PROG1) {
            if (LCD.Touch(&x, &y))
            {   
                //If run button has been touched, run the program
                if (Run[0].Pressed(x, y, 0))
                {
                    Run[0].WhilePressed(x, y);
                    RUN_FULL_FUNCTION; //todo Change this to the function you want to execute on "run" pressed
                    menu = MENU_MAIN;
                }

                //If back button has been touched, go to main menu
                if (Back[0].Pressed(x, y, 0))
                {
                    Back[0].WhilePressed(x, y);
                    menu = MENU_MAIN;
                }
            }
        }
    return menu;
}

//todo Blank functions for future expansion (currently just return MENU_MAIN so the menu ignores the button)
int MenuProg3(){return MENU_MAIN;} //todo Create a program that allows for manual translation/rotation inputs
int MenuProg4(){return MENU_MAIN;}  
int MenuFunc5(){return MENU_MAIN;} //todo Create a function that displays RPS values
int MenuFunc6(){return MENU_MAIN;} //todo Create a function that displays optosensor values (and what color it thinks it sees)
int MenuFunc7(){return MENU_MAIN;} //todo Create a function that displays encoder counts
int MenuFunc8(){
    ShowEncoders();
    return MENU_MAIN;
    } 

/*
int Menu0() { //!Template function for a back button and a run button

    //Create back button
    FEHIcon::Icon Back[1];
    char back_label[1][20] = {"<-"};
    FEHIcon::DrawIconArray(Back, 0, 1, 1, 200, 1, 260, back_label, MENU_C, TEXT_C);

    //Create run button
    FEHIcon::Icon Run[1];
    char run_label[1][20] = {"Run"};
    FEHIcon::DrawIconArray(Run, 1, 1, 20, 20, 60, 60, run_label, MENU_C, FORESTGREEN);

    int menu=MENU_RUN_FULL, n, m;
    float x, y;
    int run[4] = {0, 0, 0, 0};

    while(menu==MENU_RUN_FULL) {
            if (LCD.Touch(&x, &y))
            {   
                //If run button has been touched, run the program
                if (Run[0].Pressed(x, y, 0))
                {
                    Run[0].WhilePressed(x, y);
                    runFull(); //todo Change this to the function you want to execute on "run" pressed
                }

                //If back button has been touched, go to main menu
                if (Back[0].Pressed(x, y, 0))
                {
                    Back[0].WhilePressed(x, y);
                    menu = MENU_MAIN;
                }
            }
        }
    return menu;
}
*/

//!Menu stuff from main
// #include "menu.h"

// int main(void)
// {   
//     //Main menu
//     /*
//     The main menu is displayed, which allows the user to select a submenu.
//     The function of each submenu can be edited in menu.cpp
//     MENU_RUN_FULL (the first box) is configured to call FullRun()
//     Main menu also displays voltage
//     */

//     int menu=MENU_MAIN;
//     while (true)
//     {
//         switch (menu)
//         {
//         case MENU_MAIN:
//             menu = MenuMain();
//             break;
//         case MENU_PROG1: //This is the main robot program
//             menu = MenuProg1();
//             break;
//         case MENU_PROG2: //Auxilary testing programs
//             menu = MenuProg2();
//             break;
//         case MENU_PROG3:
//             menu = MenuProg3();
//             break;
//         case MENU_PROG4:
//             menu = MenuProg4();
//             break;
//         case MENU_FUNC5:
//             menu = MenuFunc5();
//             break;
//         case MENU_FUNC6:
//             menu = MenuFunc6();
//             break;
//         case MENU_FUNC7:
//             menu = MenuFunc7();
//             break;
//         case MENU_FUNC8:
//             menu = MenuFunc8();
//             break;
//         }
//     }
//     return(0);
// }
