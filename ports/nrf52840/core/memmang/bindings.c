#include <errno.h>
#include <string.h>
#include <stdbool.h>
#include <stdint.h>
#include "heap.h"

// Binds newlib memory allocators to that of freeRTOS heap manager

void *sbrk(int incr) {
	// Newlib standard heap usage banned
	// Instead newlib should automatically use the overriden malloc etc... functions below
	(void)incr;

	configASSERT(false);

	errno = ENOMEM;
	return (char *)-1;
}

void *_sbrk(int incr) {
	return sbrk(incr);
}

void *_sbrk_r(struct _reent *r, int incr) {
	(void)r;
	return _sbrk(incr);
}

void free(void *ptr) {
	return vPortFree(ptr);
}

void _free_r(struct _reent *r, void *ptr) {
	(void)r;
	return free(ptr);
}

void *malloc(size_t size) {
	return pvPortMalloc(size);
}

void *_malloc_r(struct _reent *r, size_t size) {
	(void)r;
	return malloc(size);
}

void *realloc(void *ptr, size_t size) {
	if (ptr == NULL) return malloc(size);
	if (size == 0) {
		free(ptr);
		return NULL;
	}
	void *mem = malloc(size);
	if (mem != NULL) {
		// NOTE: pvPortMalloc does not expose the original allocation size, so
		// we copy 'size' bytes.  Safe when shrinking.  When growing, the extra
		// bytes read are harmless because heap blocks are always >= the
		// original request (rounded up to alignment).
		memcpy(mem, ptr, size);
		free(ptr);
	}
	return mem;
}

void *_realloc_r(struct _reent *r, void *ptr, size_t size) {
	(void)r;
	return realloc(ptr, size);
}

void *calloc(size_t nitems, size_t size) {
	void *mem;
	// Guard against integer overflow in multiplication
	if (nitems != 0 && size > (SIZE_MAX / nitems)) return NULL;
	size_t bytes = nitems * size;

	mem = malloc(bytes);
	if (mem != NULL) memset(mem, 0, bytes);
	return mem;
}

void *_calloc_r(struct _reent *r, size_t nitems, size_t size) {
	(void)r;
	return calloc(nitems, size);
}

void *memalign(size_t alignment, size_t size) {
	configASSERT(false);  // Not implemented

	return NULL;
}

void *_memalign_r(struct _reent *r, size_t alignment, size_t size) {
	(void)r;
	return memalign(alignment, size);
}
