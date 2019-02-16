#include "circular_buffer.h"

CircularBuffer::CircularBuffer(int buffer_size)
{
    buffer_ = new uint8_t (buffer_size * sizeof(uint8_t));

    read_index_ = 0;
    write_index_ = 0;
    buffer_size_ = buffer_size;
}

CircularBuffer::~CircularBuffer()
{
    delete [] buffer_;
}

int CircularBuffer::bufferEmpty()
{
    return (read_index_ == write_index_);
}

int CircularBuffer::bufferFull()
{
    return ((write_index_ + 1) % buffer_size_ == read_index_);
}

int CircularBuffer::readBuffer(uint8_t* data)
{
    if(bufferEmpty())
    {
        return -1;
    }
    *data = buffer_[read_index_];
    read_index_++;
    if(read_index_ == buffer_size_)
        read_index_ = 0;
    
    return 1;
}

int CircularBuffer::writeBuffer(uint8_t* data)
{
    if(bufferFull())
    {
        return -1;
    }
    buffer_[write_index_] = *data;
    write_index_++;
    if(write_index_ == buffer_size_)
        write_index_ = 0;

    return 1;
}