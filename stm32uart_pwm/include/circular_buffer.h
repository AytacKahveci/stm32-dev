#ifndef CIRCULAR_BUFFER_H_
#define CIRCULAR_BUFFER_H_

#include <stdlib.h>
#include <stdint.h>
#include <math.h>

class CircularBuffer 
{
public:
    CircularBuffer()
    {}

    CircularBuffer(int buff_size);

    ~CircularBuffer();

    int readBuffer(uint8_t* data);

    int writeBuffer(uint8_t* data);

private:
    uint8_t read_index_, write_index_;
    uint8_t *buffer_;
    uint8_t buffer_size_;

    int bufferEmpty();

    int bufferFull();
};

#endif