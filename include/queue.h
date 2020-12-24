//
// Created by cda on 2020/9/29.
//

#ifndef UNTITLED15_QUEUE_H
#define UNTITLED15_QUEUE_H
#include <vector>

template<typename ValueType>
class Queue{
public:
    Queue(int size):array_(nullptr), size_(size), current_(0), size_used_(0)
    {
        array_ = new ValueType[size];
    }
    virtual ~Queue()
    {
        delete [] this->array_;
    }

    void push(const ValueType & value)
    {
        this->array_[current_] = value;
        if (size_used_ < size_)
        {
            size_used_ ++;
        }
        if(current_ < size_ - 1)
        {
            current_ ++;
        }
        else if(current_ == size_ - 1)
        {
            current_ = 0;
        }

    }

    const ValueType & operator[](int index)
    {
        if(size_used_ < size_)
        {
            return this->array_[index];
        }
        else if (current_ + index < size_)
        {
            return this->array_[current_ + index];
        } else
        {
            return this->array_[current_ + index - size_];
        }
    }

    ValueType mean()
    {
        ValueType result{};
        for(int i=0;i<size_used_;i++)
        {
            // std::cout << (*this)[i] << std::endl;
            result += (*this)[i]/size_used_;
        }
        return result;
    }


    void clear()
    {
        size_used_ = 0;
        current_ = 0;
    }


// protected:
    ValueType * array_;
    int size_;
    int current_;
    int size_used_;

};


#endif //UNTITLED15_QUEUE_H
