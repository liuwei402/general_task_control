
#ifndef _TASK_CONTROL_H_
#define _TASK_CONTROL_H_

#include <iostream>

#include <ros/ros.h>
#include "general_task_control/IotCommunicate.h"
#include "general_task_control/MotionControl.h"
#include "general_task_control/WorkControl.h"

namespace GeneralTaskControl
{

class TaskControl
{
public:
    enum TaskState
    {
        TASK_DATA_EMPTY,
        MOTION_CONTROL,
        WORK_CONTROL
    };

public:
    TaskControl(ros::NodeHandle *_nh,
                IotCommunicate *_iotCommunicate);
    ~TaskControl();

    /**
     * @brief   task control execute
     */
    void exec();

private:
    void init();
    void release();

    /* clear taskData */
    void clearTaseData();

    /* reset all state */
    void stateReset();

    /* taskData empty handler */
    void taskDataEmptyHandler();

    /* motion control handler */
    void motionControlHandler();

    /* motion failed handler */
    void motionFailedHandler();

    /* motion success handler */
    void motionSuccessHandler();

    /* work control handler */
    void workControlHandler();

    /* work failed handler */
    void workFailedHandler();

    /* work success handler */
    void workSuccessHandler();

private:
    ros::Rate loopFrequence;

    TaskState currentState;

    IotCommunicate *iotCommunicate;
    TaskData taskData;

    MotionControl motionControl;
    int motionCount;

    WorkControl workControl;
    int workCount;
};

} /* namespace GeneralTaskControl */

#endif