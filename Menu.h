#ifndef MENU_H
#define MENU_H

#define MENU_MAIN 0
//These should get better names as they become things
#define MENU_PROG1 1
#define MENU_PROG2 2
#define MENU_PROG3 3
#define MENU_PROG4 4
#define MENU_FUNC5 5
#define MENU_FUNC6 6
#define MENU_FUNC7 7
#define MENU_FUNC8 8

/* Define colors for parts of menus */
#define BACKGROUND_C WHITE
#define MENU_C 0xe22b00
#define TEXT_C BLACK
#define HI_C BLACK

//Displays main menu
int MenuMain();

//todo Honestly these should probably be renamed once they have discreet functions
//Prog1 is assumed to be the main program
int MenuProg1();

//TBD
int MenuProg2();
int MenuProg3();
int MenuProg4();
int MenuFunc5();
int MenuFunc6();
int MenuFunc7();
int MenuFunc8();

#endif