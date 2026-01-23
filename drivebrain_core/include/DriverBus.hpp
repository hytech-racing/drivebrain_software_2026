#pragma once

#include <deque>
#include <mutex>
#include <condition_variable>


// what this should do:
// should contain the instances of the thread-safe deques, contain the thread(s) 
// for communication and manage the routing of messages

// threading paradigm (for now):
// we will still need to have threads within the drivers themselves since otherwise we wont be able to easily wait on 

namespace core
{
    template <typename MsgType>
    struct ThreadSafeDeque
    {
        std::deque<MsgType> deque;
        std::mutex mtx;
        std::condition_variable cv;
    };
}
