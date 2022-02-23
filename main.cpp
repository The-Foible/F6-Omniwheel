#include "menu.h"

int main(void)
{   
    //Main menu
    /*
    The main menu is displayed, which allows the user to select a submenu.
    The function of each submenu can be edited in menu.cpp
    MENU_RUN_FULL (the first box) is configured to call FullRun()
    Main menu also displays voltage
    */

    int menu=MENU_MAIN;
    while (true)
    {
        switch (menu)
        {
        case MENU_MAIN:
            menu = MenuMain();
            break;
        case MENU_PROG1: //This is the main robot program
            menu = MenuProg1();
            break;
        case MENU_PROG2: //Auxilary testing programs
            menu = MenuProg2();
            break;
        case MENU_PROG3:
            menu = MenuProg3();
            break;
        case MENU_PROG4:
            menu = MenuProg4();
            break;
        case MENU_FUNC5:
            menu = MenuFunc5();
            break;
        case MENU_FUNC6:
            menu = MenuFunc6();
            break;
        case MENU_FUNC7:
            menu = MenuFunc7();
            break;
        case MENU_FUNC8:
            menu = MenuFunc8();
            break;
        }
    }
    return(0);
}
