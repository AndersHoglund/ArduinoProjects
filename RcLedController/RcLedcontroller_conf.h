#if !defined(ARDUINO_AVR_NANO) && !defined(ARDUINO_GENERIC_STM32F103C)
#error Bord type not supported

// Please select one of the supported/tested Boards and board managers:
// * Arduino Nano from the standard "Arduino AVR boards" list
// * Generic STM32F103C from "STM32F1xx boards" list ( Install Board Manager http://dan.drown.org/stm32duino/package_STM32duino_index.json )
#endif

// TODO
// I have not managed to get "STM32 MCU based boards" to compile OK. (Board Manager https://github.com/stm32duino/BoardManagerFiles/raw/main/package_stmicroelectronics_index.json  )
// This is however needed to get STM32 internal halfduplex serial to work, not supported by the Dan Brown BM. So for now we need external diod same as eith aa Nano even on the STN32.
//


// Spektrum channel order
#define THRO 0
#define AILE 1
#define ELEV 2
#define ROLL 3
#define GEAR 4
#define AUX1 5
#define AUX2 6
#define AUX3 7
#define AUX4 8
#define AUX5 9

// Only available at 22ms frame rate, not at 11ms.
#define AUX6 10
#define AUX7 11

#define LED_CONTROL_CHANNEL AUX7

