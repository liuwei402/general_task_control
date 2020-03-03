
#include "general_task_control/MotionControl.h"
#include "general_task_control/MotionGeneral.h"

namespace GeneralTaskControl
{

MotionControl::MotionControl(ros::NodeHandle *_nh)
    : nh(_nh),
      currentState(MotionState::STOP),
      actualMotion(nullptr),
      failedRetryCount(0)
{
    init();
}

MotionControl::~MotionControl()
{
    release();
}

int MotionControl::exec(const Task &task)
{
    while (ros::ok())
    {
        switch (currentState)
        {
        case MotionState::STOP:
            stopHandler();
            break;
        case MotionState::MOVING:
            movingHandler(task);
            break;
        case MotionState::FAILED:
            failedHandler();
            return 0;
        case MotionState::ARRIVED:
            arrivedHandler();
            return 1;
        default:
            break;
        }
    }
    return 0;
}

void MotionControl::currentStateReset()
{
    currentState = MotionState::STOP;
    failedRetryCount = 0;
}

void MotionControl::init()
{
    actualMotion =  new MotionGeneral(nh);
}

void MotionControl::release()
{
    if(actualMotion != nullptr)
    {
        delete actualMotion;
        actualMotion = nullptr;
    }
}

void MotionControl::stopHandler()
{
    ROS_INFO("[MotionControl::stopHandler] start motion control");

    currentState = MotionState::MOVING;
}

void MotionControl::movingHandler(const Task &task)
{
    ROS_INFO("[MotionControl::movingHandler] moving...");

    if(actualMotion != nullptr)
    {
        if (actualMotion->motion(task))
        {
            currentState = MotionState::ARRIVED;
        }
        else
        {
            currentState = MotionState::FAILED;
        }
    }
    else
    {
        ROS_ERROR("[MotionControl::movingHandler] actualMotion is a nullptr , please check ! ");
        currentState = MotionState::FAILED;
    }
}

void MotionControl::failedHandler()
{
    ROS_ERROR("[MotionControl::failedHandler] motion failed !");

    if(failedRetryCount < 3)
    {
        currentState = MotionState::MOVING;
    }
    failedRetryCount++;
}

void MotionControl::arrivedHandler()
{
    ROS_INFO("[MotionControl::arrivedHandler] motion success !");

    currentState = MotionState::STOP;
}


} /* namespace GeneralTaskControl */