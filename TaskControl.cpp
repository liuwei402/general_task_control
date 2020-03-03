
#include "general_task_control/TaskControl.h"

namespace GeneralTaskControl
{

#define TASK_CONTROL_LOOP_FREQUENCE (100)

TaskControl::TaskControl(ros::NodeHandle *_nh,
                         IotCommunicate *_iotCommunicate)
    : loopFrequence(TASK_CONTROL_LOOP_FREQUENCE),
      currentState(TaskState::TASK_DATA_EMPTY),
      iotCommunicate(_iotCommunicate),
      motionControl(_nh),
      motionCount(0),
      workControl(_nh),
      workCount(0)

{
    init();
}

TaskControl::~TaskControl()
{
    release();
}

void TaskControl::exec()
{
    while (ros::ok())
    {
        switch (currentState)
        {
        case TaskState::TASK_DATA_EMPTY:
            taskDataEmptyHandler();
            break;
        case TaskState::MOTION_CONTROL:
            motionControlHandler();
            break;
        case TaskState::WORK_CONTROL:
            workControlHandler();
            break;
        default:
            break;
        }

        loopFrequence.sleep();
    }
}

void TaskControl::init()
{
}

void TaskControl::release()
{
}

void TaskControl::clearTaseData()
{
    while(taskData.size())
    {
        taskData.pop();
    }
}

void TaskControl::stateReset()
{
    clearTaseData();
    currentState = TaskState::TASK_DATA_EMPTY;
    motionControl.currentStateReset();
    workControl.currentStateReset();
}

void TaskControl::taskDataEmptyHandler()
{
    if (!taskData.size())
    {
        if (iotCommunicate->taskDataQueueSize())
        {
            taskData = iotCommunicate->taskDataQueueFront();
            iotCommunicate->taskDataQueuePop();
            ROS_INFO("[TaskControl::taskDataEmptyHandler] get taskData success , task nums is : %d",
                     taskData.size());
            currentState = TaskState::MOTION_CONTROL;
        }
    }
}

void TaskControl::motionControlHandler()
{
    if (!taskData.size())
    {
        currentState = TaskState::TASK_DATA_EMPTY;
        return;
    }

    /* motion control execute */
    Task task = taskData.front();
    int motionStatus = motionControl.exec(task);
    motionCount++;
    /* motion control result handle */
    switch (motionStatus)
    {
    case 0:
        motionFailedHandler();
        break;
    case 1:
        motionSuccessHandler();
        break;
    default:
        break;
    }
}

void TaskControl::motionFailedHandler()
{
    /* motion failed , try again */
    if (motionCount < 4)
    {
        ROS_WARN("[TaskControl::motionControlHandler] motion control failed , try again. %d times",
                 motionCount);
    }
    else
    {
        ROS_ERROR("[TaskControl::motionControlHandler] motion control still failed.");
        motionCount = 0;

        stateReset();
    }
}

void TaskControl::motionSuccessHandler()
{
    /* motion sucess */
    currentState = TaskState::WORK_CONTROL;

    motionCount = 0;
}

void TaskControl::workControlHandler()
{
    if (!taskData.size())
    {
        currentState = TaskState::TASK_DATA_EMPTY;
        return;
    }

    /* work control execute */
    Task task = taskData.front();
    /* work control result handle */
    int workStatus = 0;
    if(task.workState)
    {
        ROS_INFO("[TaskControl::workControlHandler] workState is true , begin to work.");
        workStatus = workControl.exec(task.workData);
    }
    else
    {
        workStatus =  1;
        ROS_INFO("[TaskControl::workControlHandler] workState is false , do not need to work.");
    }
    workCount++;
    switch (workStatus)
    {
    case 0:
        workFailedHandler();
        break;
    case 1:
        workSuccessHandler();
        break;
    default:
        break;
    }
}

void TaskControl::workFailedHandler()
{
    /* work failed , try again */
    if (workCount < 4)
    {
        ROS_WARN("[TaskControl::workControlHandler] work control failed , try again. %d times",
                    workCount);
    }
    else
    {
        ROS_ERROR("[TaskControl::workControlHandler] work control still failed.");
        workCount = 0;
        
        stateReset();
    }
}

void TaskControl::workSuccessHandler()
{
    /* work success */
    currentState = TaskState::MOTION_CONTROL;

    workCount = 0;

    /* always pop taskData here */
    taskData.pop();
}

} /* namespace GeneralTaskControl */