#ifndef LOG_H__
#define LOG_H__

#ifdef __cplusplus
namespace core {
extern "C" {
#endif

int log_init_default();

int log_init(const char *name, const char *path);

void log_enable();

void log_disable();

void log_debug(const char *tag, const char* format, ...);

void log_info(const char *tag, const char* format, ...);

void log_warn(const char *tag, const char* format, ...);

void log_error(const char *tag, const char* format, ...);

void log_flush();

void log_hexdump(void *buf, int size);

void log_hexdump_with_hint(const char *hint, void *buf, int size);

#ifdef __cplusplus
}
}
#endif

#endif
