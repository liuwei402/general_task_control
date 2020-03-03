
#include "general_task_control/IotCommunicate.h"

#include "general_task_control/Global.h"

#include "json/json.h"

namespace GeneralTaskControl
{

/* invalid task id */
#define INVALID_TASK_ID (-1)

IotCommunicate::IotCommunicate(ros::NodeHandle *_nh)
    : nh(_nh),
      lastTaskId(INVALID_TASK_ID)
{
    init();
}

IotCommunicate::~IotCommunicate()
{
    release();
}

void IotCommunicate::taskDataCallBack(const std_msgs::String &taskData)
{
    ROS_INFO("[IotCommunicate::taskDataCallBack] Task data is :\n%s",
             taskData.data.c_str());

    this->parseTaskData(taskData);
}

void IotCommunicate::basicPublish(const general_task_control::device_info_basic &basic)
{
    basicPublisher.publish(basic);
}

void IotCommunicate::taskPublish(const general_task_control::device_info_task &task)
{
    taskPublisher.publish(task);
}

void IotCommunicate::exceptionPublish(const general_task_control::device_info_exception &exception)
{
    exceptionPublisher.publish(exception);
}

int IotCommunicate::taskDataQueueSize()
{
    int size = 0;

    taskDataQueueMutex.lock();
    size = taskDataQueue.size();
    taskDataQueueMutex.unlock();

    return size;
}

TaskData IotCommunicate::taskDataQueueFront()
{
    TaskData taskData;

    taskDataQueueMutex.lock();
    if (taskDataQueue.size())
    {
        taskData = taskDataQueue.front();
    }
    taskDataQueueMutex.unlock();

    return taskData;
}

void IotCommunicate::taskDataQueuePop()
{
    taskDataQueueMutex.lock();
    if (taskDataQueue.size())
    {
        taskDataQueue.pop();
    }
    taskDataQueueMutex.unlock();
}

void IotCommunicate::init()
{
    /* subscriber */
    taskDataSubcriber = nh->subscribe(ROS_SUB_TOPIC_TASK_DATA, 1000,
                                      &IotCommunicate::taskDataCallBack, this);
    /* publisher */
    basicPublisher = nh->advertise<general_task_control::device_info_basic>(ROS_PUB_TOPIC_DIVICE_INFO_BASIC, 5);
    taskPublisher = nh->advertise<general_task_control::device_info_task>(ROS_PUB_TOPIC_DIVICE_INFO_TASK, 5);
    exceptionPublisher = nh->advertise<general_task_control::device_info_exception>(ROS_PUB_TOPIC_DIVICE_INFO_EXCEPTION, 5);
}

void IotCommunicate::release()
{
}

void IotCommunicate::parseTaskData(const std_msgs::String &taskData)
{
    Json::Reader reader;
    Json::Value root;

    if (reader.parse(taskData.data, root) && root.isObject())
    {
        ROS_INFO("[IotCommunicate::parseTaskData] Task data parse success.");

        /* current task id */
        int currentTaskId = INVALID_TASK_ID;
        if (root["TaskID"].isInt())
        {
            currentTaskId = root["TaskID"].asInt();
        }

        /* avoid the same task deal multiple times */
        if (lastTaskId != currentTaskId)
        {
            lastTaskId = currentTaskId;

            /* pase task data to taskDataQueue */
            TaskData taskData;
            Task task;
            /* origin point task */
            task.clear();
            task.x = root["RouteData"]["OrgPt"]["X"].asDouble();
            task.y = root["RouteData"]["OrgPt"]["Y"].asDouble();
            task.z = root["RouteData"]["OrgPt"]["Z"].asDouble();
            task.angle = root["RouteData"]["OrgPt"]["Angle"].asDouble();
            task.workState = root["RouteData"]["OrgPt"]["WorkState"].asBool();
            if (root["RouteData"]["OrgPt"]["WorkData"].isObject() ||
                root["RouteData"]["OrgPt"]["WorkData"].isArray())
            {
                task.workData = root["RouteData"]["OrgPt"]["WorkData"].toStyledString();
            }
            taskData.push(task);
            /* other route point task */
            task.clear();
            for (int i = 0; i < root["RouteData"]["RoutePts"].size(); i++)
            {
                for (int j = 0; j < root["RouteData"]["RoutePts"].size(); j++)
                {
                    if (i + 1 == root["RouteData"]["RoutePts"][j]["Order"].asInt())
                    {
                        task.x = root["RouteData"]["RoutePts"][j]["X"].asDouble();
                        task.y = root["RouteData"]["RoutePts"][j]["Y"].asDouble();
                        task.z = root["RouteData"]["RoutePts"][j]["Z"].asDouble();
                        task.angle = root["RouteData"]["RoutePts"][j]["Angle"].asDouble();
                        task.workState = root["RouteData"]["RoutePts"][j]["WorkState"].asBool();
                        if (root["RouteData"]["RoutePts"][j]["WorkData"].isObject() ||
                            root["RouteData"]["RoutePts"][j]["WorkData"].isArray())
                        {
                            task.workData = root["RouteData"]["RoutePts"][j]["WorkData"].toStyledString();
                        }
                        taskData.push(task);
                    }
                }
            }

            /* check route point nums */
            if (taskData.size() == root["RouteData"]["NumOfPoints"].asInt() + 1)
            {
                taskDataQueueMutex.lock();
                taskDataQueue.push(taskData);
                taskDataQueueMutex.unlock();
            }
            else
            {
                ROS_ERROR("[IotCommunicate::parseTaskData] route point nums may be incorrect , please check!");
            }
        }
        else
        {
            ROS_WARN("[IotCommunicate::parseTaskData] TaskID has sent . It is equal to last task id . ");
        }
    }
    else
    {
        ROS_ERROR("[IotCommunicate::parseTaskData] Task data parse failed , \
                    may be task data isn't a complete json.");
    }
}

} /* namespace GeneralTaskControl */