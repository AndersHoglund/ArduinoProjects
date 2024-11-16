#if !defined(ARDUINO_AVR_NANO) && !defined(ARDUINO_GENERIC_STM32F103C) && !defined(ARDUINO_BLUEPILL_F103C8)
#error Bord type not supported

// Please select one of the upported/tested Board types and Board Banagers /Board Support Packages:
// * Arduino Nano from the standard "Arduino AVR boards" list
// * STM32F103C8 "Bluepill" using one of the two BMs/BSPs:
// ** "Generic STM32F1 series" from "STM32 MCU based boards" list and  select Board part no "BluePill F103C8"
//    ( Install Board Manager https://github.com/stm32duino/BoardManagerFiles/raw/main/package_stmicroelectronics_index.json )
// ** "Generic STM32F103C series" from "STM32F1xx/STM32GDxx" list and select variant "STM32F103C8 20k RAM 64k Flash"
//    ( Install Board Manager http://dan.drown.org/stm32duino/package_STM32duino_index.json )
//   

#endif

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

