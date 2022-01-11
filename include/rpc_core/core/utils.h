#ifndef UTILS_H_
#define UTILS_H_

#include <stdint.h>
#include <pthread.h>
#include <time.h>

#ifdef __cplusplus
namespace core {
extern "C" {
#endif

int exec_cmd(const char *cmd);

int popen_cmd(const char *cmd);

int64_t get_unix_timestamp_ms();

uint32_t get_uptime();

uint64_t get_uptime_ms();

int msleep(int ms);

struct tm *get_now_time(void);

uint32_t get_current_time(void);

#ifdef __cplusplus
}
}
#endif

#endif
