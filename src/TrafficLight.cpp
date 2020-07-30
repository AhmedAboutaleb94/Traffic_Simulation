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
    // FP.5a : The method receive should use std::unique_lock<std::mutex> and _condition.wait() 
    // to wait for and receive new messages and pull them from the queue using move semantics. 
    // The received object should then be returned by the receive function. 
}

template <typename T>
void MessageQueue<T>::send(T &&msg)
{
    // FP.4a : The method send should use the mechanisms std::lock_guard<std::mutex> 
    // as well as _condition.notify_one() to add a new message to the queue and afterwards send a notification.
}


/* Implementation of class "TrafficLight" */


TrafficLight::TrafficLight()
{
    _currentPhase = TrafficLightPhase::red;
}

void TrafficLight::waitForGreen()
{
    // FP.5b : add the implementation of the method waitForGreen, in which an infinite while-loop 
    // runs and repeatedly calls the receive function on the message queue. 
    // Once it receives TrafficLightPhase::green, the method returns.
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
    std::mt19937 eng(rn());
    std::uniform_int_distribution<float> distr(4,6);

    /* Print the ID of the Current thread */
    std::unique_lock<std::mutex> lck(_mutex);
    // std::cout << "Traffic Light #" << TrafficObject::getID() << "Cycle_Through_Pha"
    lck.unlock();

    /* Initialize Variables */
    int cycle_dur = distr(eng);

    /* Initialize a Stop Watch */
    auto prev_update = std::chrono::system_clock::now();
    while (true){
        long time_since_last_update = std::chrono::duration_cast<std::chrono::seconds>(std::chrono::system_clock::now() - prev_update).count();

        /* Sleep every 1 millisecond */
        std::this_thread::sleep_for(std::chrono::milliseconds(1));

        /* Toggling between traffic lights */
        if(time_since_last_update >= cycle_dur){
            if(_currentPhase == red){
                _currentPhase = green;
            }
            else if (_currentPhase == green){
                _currentPhase = red;
            }
        }

        /* Send an update to the message queue */
        auto msg = _currentPhase;
        auto is_sent = std::async(std::launch::async, &MessageQueue<TrafficLightPhase>::send, msg_queue, std::move(msg));
        is_sent.wait();

        /* Reset the StopWatch */
        prev_update = std::chrono::system_clock::now();

        /* Randomly choose the cycle duration for the next cycle */
        cycle_dur = distr(eng);
    }
}

