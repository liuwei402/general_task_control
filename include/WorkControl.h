
#ifndef _WORK_CONTROL_H_
#define _WORK_CONTROL_H_

#include <iostream>
#include "json/json.h"

#include <ros/ros.h>

#include "general_task_control/WorkBase.h"

namespace GeneralTaskControl
{

class WorkControl
{
public:
    enum WorkState
    {
        STOP,
        WORKING,
        FAILED,
        FINISHED
    };

public:
    WorkControl(ros::NodeHandle *_nh);
    ~WorkControl();

    /**
     * @brief   work control execute
     * @return  work control status
     *          success 1
     *          failed 0
     */
    int exec(const std::string &workData);

    /* currentState reset */
    void currentStateReset();

private:
    void init();
    void release();

    /* stop handler */
    void stopHandler();

    /* working handler */
    void workingHandler(const std::string &workData);

    /* failed handler */
    void failedHandler();

    /* finished handler */
    void finishedHandler();

private:
    ros::NodeHandle *nh;

    WorkState currentState;

    WorkBase *actualWork;

    int failedRetryCount;
};

} /* namespace GeneralTaskControl */

#endif