#pragma once

#ifdef __cplusplus
extern "C" {
#endif

// A few basic types Spektrum depends on
#define INT8 char
#define INT16 short int
#define INT32 long int
#define INT64 long long int

#define UINT8 unsigned char
#define UINT16 unsigned short int
#define UINT32 unsigned long int
#define UINT64 unsigned long long int

#define NO_DATA 0xff
#define UINT_NO_DATA 0xffff
#define INT_NO_DATA_BE 0x7fff
#define INT_NO_DATA_LE 0xff7f

#ifdef __cplusplus
} // extern "C"
#endif
