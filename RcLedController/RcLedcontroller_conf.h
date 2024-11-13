#if !defined(ARDUINO_AVR_NANO) && !defined(ARDUINO_BLUEPILL_F103C8)
#error Bord type not supported

// Please select one of the two supported/tested Boards and board managers:
// * Arduino Nano from the standard "Arduino AVR boards" list
// * Generic STM32F1 series from "STM32 MCU based boards" list and  select Board part no "BluePill F103C8"
//   ( Install Board Manager https://github.com/stm32duino/BoardManagerFiles/raw/main/package_stmicroelectronics_index.json )
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

