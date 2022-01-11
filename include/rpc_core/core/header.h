#ifndef MSG_HEADER_H
#define MSG_HEADER_H

#include <stdint.h>

#ifdef __cplusplus
namespace core {
extern "C" {
#endif

enum {
    kHeaderRequest = 'Q',
    kHeaderResponse = 'S'
};

struct header {
    int type;
    uint32_t seq;
    int data_type;
    int data_len;
    uint32_t timestamp;
};

#ifdef __cplusplus
}
}
#endif

#endif
