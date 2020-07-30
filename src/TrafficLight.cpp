#include <iostream>
#include <random>
#include <thread>
#include <chrono>
#include <future>
#include "TrafficLight.h"

/* Implementation of class "MessageQueue" */

 
template <typename T>
T MessageQueue<T>::receive()
{ 
    std::unique_lock<std::mutex> lck(_mtx);
    _cond.wait(lck, [this] { return !_queue.empty();});

    T msg = std::move(_queue.front());
    _queue.pop_front();
    return msg;
}

template <typename T>
void MessageQueue<T>::send(T &&msg)
{
    /* Using Lock_guard to prevent a data race */
    std::lock_guard<std::mutex> lck(_mtx);
    /* Move data into queue and notify client */
    _queue.emplace_back(msg);
    _cond.notify_one();
}


/* Implementation of class "TrafficLight" */


TrafficLight::TrafficLight()
{
    _currentPhase = TrafficLightPhase::red;
}

void TrafficLight::waitForGreen()
{
    while (true){
        std::this_thread::sleep_for(std::chrono::milliseconds(1));

        auto current_phase = _queue.receive();
        if(current_phase == green){
            return;
        }
    }
}

TrafficLightPhase TrafficLight::getCurrentPhase()
{
    return _currentPhase;
}

void TrafficLight::simulate()
{
    threads.emplace_back(std::thread(&TrafficLight::cycleThroughPhases, this));
}

// virtual function which is executed in a thread
void TrafficLight::cycleThroughPhases()
{
    /* Initialize our random generation between 4 and 6 */
    std::random_device rn;
    std::mt19937 gen(rn());
    std::uniform_int_distribution<> dist(4,6);

    /* Initialize Variables */
    int cycle_dur = dist(gen);

    /* Initialize a Stop Watch */
    auto prev_update = std::chrono::system_clock::now();

    while (true){
        /* Sleep every 1 millisecond */
        std::this_thread::sleep_for(std::chrono::milliseconds(1));

        auto time_since_last_update = std::chrono::duration_cast<std::chrono::seconds>(std::chrono::system_clock::now() - prev_update).count();

        /* Toggling between traffic lights */
        if(time_since_last_update >= cycle_dur){
            if(_currentPhase == red){
                _currentPhase = green;
            }
            else if (_currentPhase == green){
                _currentPhase = red;
            }

            /* Send an update to the message queue */
            auto is_sent = std::async(std::launch::async, &MessageQueue<TrafficLightPhase>::send, &_queue, std::move(_currentPhase));
            is_sent.wait();

            /* Reset the StopWatch */
            prev_update = std::chrono::system_clock::now();

            /* Randomly choose the cycle duration for the next cycle */
            cycle_dur = dist(gen);
        }
    }
}

