#ifndef __LOGGER_H__
#define __LOGGER_H__

/**
 * Unified debug log specification,
 * different modules to print a different label
 * and will print func name and line
 * Instructions, such as airkiss module want add a print
 * LOGE(DEBUG_TEST,"hello workld ...\n")
 * you can set debug_mask value
 * to choose which module need to print
 * if you need to add more modules,
 * please refer to the original way
 */

#define LOG_MOD_NAME_LEN    64

typedef unsigned int ModMask_t;
typedef unsigned char ModName_t;

typedef struct {
    ModMask_t mod_mask;
    ModName_t mod_name[LOG_MOD_NAME_LEN];
} ModLogDef_t;

/* 定义日志等级 */
typedef enum {
    ELEVEL_TRACE = 0,
    ELEVEL_DEBUG,
    ELEVEL_INFO,
    ELEVEL_ERROR,
} TLogLevel;

#define TAG modid
#define LOG_MOD_DECLARE(tag, id) static ModMask_t (tag) = (id)

#define LOG_SHOW_FILE    (0)
#define LOG_SHOW_FUNC    (1)
#define LOG_SHOW_LINE    (1)

#define LOG_STR_FILE     __FILE__
#define LOG_STR_FUNC     __FUNCTION__
#define LOG_STR_LINE     __LINE__

#define LOGT(mod_id,...) logger_printf(mod_id, ELEVEL_TRACE, LOG_STR_FILE, LOG_STR_FUNC, LOG_STR_LINE, __VA_ARGS__)
#define LOGD(mod_id,...) logger_printf(mod_id, ELEVEL_DEBUG, LOG_STR_FILE, LOG_STR_FUNC, LOG_STR_LINE, __VA_ARGS__)
#define LOGI(mod_id,...) logger_printf(mod_id, ELEVEL_INFO,  LOG_STR_FILE, LOG_STR_FUNC, LOG_STR_LINE, __VA_ARGS__)
#define LOGE(mod_id,...) logger_printf(mod_id, ELEVEL_ERROR, LOG_STR_FILE, LOG_STR_FUNC, LOG_STR_LINE, __VA_ARGS__)

#ifdef __cplusplus
extern "C" {
#endif

void logger_printf(int mod_mask, int level, const char* filename, const char* func, unsigned int line, const char* format, ...);

int logger_init(const ModLogDef_t *mod_array, unsigned int mod_num, const char *log_dir);
int logger_level(TLogLevel level);
int logger_module_enable(ModMask_t mod_mask);
int logger_module_disable(ModMask_t mod_mask);

#ifdef __cplusplus
}
#endif

#endif /* __LOGGER_H__ */
