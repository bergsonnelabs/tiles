/**
 * log_module.h -- Stub logging macros
 */

#ifndef LOG_MODULE_H
#define LOG_MODULE_H

#define LOG_INFO_APP(...)     ((void)0)
#define LOG_ERROR_APP(...)    ((void)0)
#define LOG_WARNING_APP(...)  ((void)0)
#define LOG_DEBUG_APP(...)    ((void)0)
#define LOG_INFO_SYSTEM(...)    ((void)0)   /* flash_manager.c */
#define LOG_ERROR_SYSTEM(...)   ((void)0)
#define LOG_DEBUG_SYSTEM(...)   ((void)0)

/* Log regions and levels */
typedef enum {
  LOG_REGION_BLE = 0,
  LOG_REGION_APP,
  LOG_REGION_ALL_REGIONS = 0xFFFF
} Log_Region_t;

typedef enum {
  LOG_VERBOSE_INFO = 0,
  LOG_VERBOSE_DEBUG,
  LOG_VERBOSE_WARNING,
  LOG_VERBOSE_ERROR
} Log_Verbose_Level_t;

typedef struct {
  Log_Verbose_Level_t verbose_level;
  uint32_t region_mask;
} Log_Module_t;

#endif /* LOG_MODULE_H */
