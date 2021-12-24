#ifndef _DEBUG_H_
#define _DEBUG_H_

#define DEBUG_LEVEL_NONE  0 // Nu Debugging
#define DEBUG_LEVEL_1     1 // Own debug messages
#define DEBUG_LEVEL_2     2 // State debug messages
#define DEBUG_LEVEL_3     3 // All state debug messages (not implemented yet!)

#define DEBUG_NONE        0x00
#define DEBUG_STATES      0x01
#define DEBUG_CUSTOM      0x02


#define DEBUG_PRINT(enable, fmt, ...) \
  do { if(enable) fprintf(stderr, fmt, __VA_ARGS__); } while (0)

#define DEBUG_PRINT_EX(enable, fmt, ...) \
  do { if(enable)fprintf(stderr, "file: %s\nline: %d\nfunction :%s(): \n" fmt, __FILE__, \
    __LINE__, __func__, __VA_ARGS__); } while (0)

#endif // _DEBUG_H_
