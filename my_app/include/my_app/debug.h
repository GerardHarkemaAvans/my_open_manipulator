#ifndef _DEBUG_H_
#define _DEBUG_H_

#define DEBUG_NONE               0x0000
#define DEBUG_STATES             0x0001
#define DEBUG_BEHAVIORS          0x0002
#define DEBUG_BEHAVIORS_STATES   0x0004
// defin other dembuf functions here
#define DEBUG_CUSTOM             0x0080


#define DEBUG_PRINT(enable, fmt, ...) \
  do { if(enable) fprintf(stderr, fmt, __VA_ARGS__); } while (0)
//  do { if(enable) fprintf(stderr, fmt __VA_OPT__(,) __VA_ARGS__); } while (0)

#define DEBUG_PRINT_EX(enable, fmt, ...) \
  do { if(enable)fprintf(stderr, "file: %s\nline: %d\nfunction :%s(): \n" fmt, __FILE__, \
    __LINE__, __func__, __VA_ARGS__); } while (0)

#endif // _DEBUG_H_
