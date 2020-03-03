
#include "general_task_control/WorkGeneral.h"

namespace GeneralTaskControl
{

WorkGeneral::WorkGeneral(ros::NodeHandle *_nh)
    : nh(_nh)
{
}

WorkGeneral::~WorkGeneral()
{
}

bool WorkGeneral::work(const std::string &workData)
{
    /* TODO: need to implement actual work */
    return false;
}

} /* namespace GeneralTaskControl */