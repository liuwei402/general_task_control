
#include "general_task_control/WorkControl.h"
#include "general_task_control/WorkGeneral.h"

namespace GeneralTaskControl
{

WorkControl::WorkControl(ros::NodeHandle *_nh)
    : nh(_nh),
      currentState(WorkState::STOP),
      actualWork(nullptr),
      failedRetryCount(0)
{
    init();
}

WorkControl::~WorkControl()
{
    release();
}

int WorkControl::exec(const std::string &workData)
{
    while (ros::ok())
    {
        switch (currentState)
        {
        case WorkState::STOP:
            stopHandler();
            break;
        case WorkState::WORKING:
            workingHandler(workData);
            break;
        case WorkState::FAILED:
            failedHandler();
            return 0;
        case WorkState::FINISHED:
            finishedHandler();
            return 1;
        default:
            break;
        }
    }

    return 0;
}

void WorkControl::currentStateReset()
{
    currentState = WorkState::STOP;
    failedRetryCount = 0;
}

void WorkControl::init()
{
    actualWork = new WorkGeneral(nh);
}

void WorkControl::release()
{
    if(actualWork != nullptr)
    {
        delete actualWork;
        actualWork = nullptr;
    }
}

void WorkControl::stopHandler()
{
    currentState = WorkState::WORKING;
}

void WorkControl::workingHandler(const std::string &workData)
{
    ROS_INFO("[WorkControl::workingHandler] start work control");
    ROS_INFO("[WorkControl::workingHandler] working...");

    if(actualWork != nullptr)
    {
        if (actualWork->work(workData))
        {
            currentState = WorkState::FINISHED;
        }
        else
        {
            currentState = WorkState::FAILED;
        }
    }
    else
    {
        ROS_ERROR("[WorkControl::workingHandler] actualWork is a nullptr , please check ! ");
        currentState = WorkState::FAILED;
    }
}

void WorkControl::failedHandler()
{
    ROS_ERROR("[WorkControl::failedHandler] work failed !");

    if(failedRetryCount < 3)
    {
        currentState = WorkState::WORKING;
    }
    failedRetryCount++;
}

void WorkControl::finishedHandler()
{
    ROS_INFO("[WorkControl::finishedHandler] work finished !");

    currentState = WorkState::STOP;
}

} /* namespace GeneralTaskControl */