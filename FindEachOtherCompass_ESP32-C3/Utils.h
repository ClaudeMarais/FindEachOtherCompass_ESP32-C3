#ifndef _UTILS
#define _UTILS

#ifdef DEBUG
#define DebugPrintf(...) Serial.printf(__VA_ARGS__)
#define DebugPrintln(...) Serial.println(__VA_ARGS__)
#else
#define DebugPrintln(...)
#define DebugPrintf(...)
#endif

#endif