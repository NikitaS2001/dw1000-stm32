#pragma once

#include <FreeRTOS.h>
#include <assert.h>
#include <semphr.h>

#include <queue>
#include <utility>

namespace std
{

template <typename T>
class concurrent_queue
{
public:
    concurrent_queue()
    {
        mutex_ = xSemaphoreCreateMutex();
        assert(mutex_ != NULL);
    }

    T pop()
    {
        xSemaphoreTake(mutex_, portMAX_DELAY);

        T item = queue_.front();
        queue_.pop();

        xSemaphoreGive(mutex_);

        return item;
    }

    void push(const T& item)
    {
        xSemaphoreTake(mutex_, portMAX_DELAY);

        queue_.push(item);

        xSemaphoreGive(mutex_);
    }

    size_t size()
    {
        xSemaphoreTake(mutex_, portMAX_DELAY);

        size_t res = queue_.size();

        xSemaphoreGive(mutex_);

        return res;
    }

private:
    std::queue<T> queue_;
    xSemaphoreHandle mutex_;
};

} // namespace std
