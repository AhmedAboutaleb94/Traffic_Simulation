#ifndef TRAFFICLIGHT_H
#define TRAFFICLIGHT_H

#include <mutex>
#include <deque>
#include <condition_variable>
#include "TrafficObject.h"

// forward declarations to avoid include cycle
class Vehicle;

enum TrafficLightPhase
{
    red,
    green,
};

template <class T>
class MessageQueue
{
public:
    T receive();
    void send(T &&msg);
private:
    std::deque<TrafficLightPhase> _queue;
    std::condition_variable _cond;
    std::mutex _mtx;
    
};

class TrafficLight : public TrafficObject
{
public:
    // constructor / desctructor

    // getters / setters
    TrafficLightPhase getCurrentPhase();
    // typical behaviour methods
    void waitForGreen();
    void simulate();
private:
    // typical behaviour methods
    void cycleThroughPhases();
    
    /* Member Variables */
    std::shared_ptr<MessageQueue<TrafficLightPhase>> msg_queue;
    TrafficLightPhase _currentPhase;
    std::condition_variable _condition;
    std::mutex _mutex;
};

#endif