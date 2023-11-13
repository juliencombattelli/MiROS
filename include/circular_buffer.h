#ifndef CIRCULAR_BUFFER_H_
#define CIRCULAR_BUFFER_H_

#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>

// The hidden definition of our circular buffer structure
typedef struct {
    uint8_t * buffer;
    size_t head;
    size_t tail;
    size_t max; // of the buffer
    bool full;
} circular_buf_t;

/// Pass in a storage buffer and size 
/// Returns a circular buffer handle
void circular_buf_init(circular_buf_t* buf, uint8_t* buffer, size_t size);

/// Free a circular buffer structure.
/// Does not free data buffer; owner is responsible for that
void circular_buf_free(circular_buf_t* me);

/// Reset the circular buffer to empty, head == tail
void circular_buf_reset(circular_buf_t* me);

/// Put version 1 continues to add data if the buffer is full
/// Old data is overwritten
void circular_buf_put(circular_buf_t* me, uint8_t data);

/// Put Version 2 rejects new data if the buffer is full
/// Returns 0 on success, -1 if buffer is full
int circular_buf_put2(circular_buf_t* me, uint8_t data);

/// Retrieve a value from the buffer
/// Returns 0 on success, -1 if the buffer is empty
int circular_buf_get(circular_buf_t* me, uint8_t * data);

/// Returns true if the buffer is empty
bool circular_buf_empty(circular_buf_t* me);

/// Returns true if the buffer is full
bool circular_buf_full(circular_buf_t* me);

/// Returns the maximum capacity of the buffer
size_t circular_buf_capacity(circular_buf_t* me);

/// Returns the current number of elements in the buffer
size_t circular_buf_size(circular_buf_t* me);

#endif
