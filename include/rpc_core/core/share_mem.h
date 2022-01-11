#ifndef SHARE_MEM_MANAGER_H_
#define SHARE_MEM_MANAGER_H_

#include <sys/types.h>

#ifdef __cplusplus
namespace core {
extern "C" {
#endif

/*
 * API is not thread safe yet. please alloc memory when program init
 */

void *share_mem_alloc(const char *name, size_t size);
void share_mem_free(void *p);

int share_mem_lock(void *p);
int share_mem_unlock(void *p);

// int share_mem_copyto(void *p, void *dst, size_t size);
// int share_mem_copyfrom(void *p, void *src, size_t size);

#ifdef __cplusplus

class share_mem_sync
{
public:
    share_mem_sync() = delete;

    share_mem_sync(void *p)
    {
        p_mem = p;
        // FIXME: do error check
        share_mem_lock(p_mem);
    }

    ~share_mem_sync() {
        // FIXME: do error check
        share_mem_unlock(p_mem);
    }
private:
    void *p_mem;
};

#endif

#ifdef __cplusplus
}
}
#endif

#endif
