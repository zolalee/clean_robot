#ifndef __LOGGER_RC_H__
#define __LOGGER_RC_H__

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

enum {
    DEBUG_TRANSCEIVER       = 0,
    DEBUG_PROTOCOL          = 1,
    DEBUG_ASYNC_INVOKE      = 2,
    DEBUG_IPC               = 3,
    DEBUG_KEY               = 4,
    DEBUG_UART              = 5,
    DEBUG_CLIFF             = 6,
    DEBUG_OBS               = 7,
    DEBUG_DPM               = 8,
    DEBUG_DPM_IPC           = 9,
};


#define DEBUG_DESC_TRANSCEIVER      "Transceiver" 
#define DEBUG_DESC_PROTOCOL         "Protocol"
#define DEBUG_DESC_ASYNC_INVOKE     "AsyncInvoke"
#define DEBUG_DESC_IPC              "IPC"
#define DEBUG_DESC_KEY              "Key"
#define DEBUG_DESC_UART             "Uart"
#define DEBUG_DESC_CLIFF            "Hello"
#define DEBUG_DESC_OBS              "Obs"
#define DEBUG_DESC_DPM              "Dpm"
#define DEBUG_DESC_DPM_IPC          "DmpIpc"


#endif /* __LOGGER_RC_H__ */
