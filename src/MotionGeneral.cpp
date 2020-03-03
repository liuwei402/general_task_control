
#include "general_task_control/MotionGeneral.h"
#include "tf/tf.h"

namespace GeneralTaskControl
{

#define MOTION_CONTROL_ACTION "motion_control"
#define MOTION_CONTROL_ACTION_WAIT_DURATION (5)
#define MOTION_CONTROL_GOAL_LINEAR_VEL_MAX (5000)
#define MOTION_CONTROL_GOAL_ANGULAR_VEL_MAX (100)
#define MOTION_CONTROL_GOAL_LINEAR_TRAJ (1)

MotionGeneral::MotionGeneral(ros::NodeHandle *_nh)
    : nh(_nh),
      motionControlActionClient(MOTION_CONTROL_ACTION, true),
      currentPosition(_nh),
      motionArrived(false)
{
}

MotionGeneral::~MotionGeneral()
{
}

bool MotionGeneral::motion(const Task &task)
{
    ROS_INFO("[MotionGeneral::motion] connect to motion control action server for %ds.",
             MOTION_CONTROL_ACTION_WAIT_DURATION);
    bool ok = motionControlActionClient.waitForServer(ros::Duration(MOTION_CONTROL_ACTION_WAIT_DURATION, 0));
    if (ok)
    {
        ROS_INFO("[MotionGeneral::motion] connect success...");
        ROS_INFO("[MotionGeneral::motion] motion action goal had sent...");

        MotionControlActionGoal goal;

        geometry_msgs::Quaternion orientation = currentPosition.getPosition().pose.orientation;
        tf::Matrix3x3 mat(tf::Quaternion(orientation.x, orientation.y, orientation.z, orientation.w));
        double yaw, pitch, roll;
        mat.getEulerYPR(yaw, pitch, roll);

        goal.ctrl_cmd.start_pose_x = currentPosition.getPosition().pose.position.x * 1000.;
        goal.ctrl_cmd.start_pose_y = currentPosition.getPosition().pose.position.y * 1000.;
        goal.ctrl_cmd.start_atti_yaw = yaw;

        goal.ctrl_cmd.aim_pose_x = task.x;
        goal.ctrl_cmd.aim_pose_y = task.y;
        goal.ctrl_cmd.aim_atti_yaw = task.angle;

        goal.ctrl_cmd.linear_vel_max = MOTION_CONTROL_GOAL_LINEAR_VEL_MAX;
        goal.ctrl_cmd.angular_rate_max = MOTION_CONTROL_GOAL_ANGULAR_VEL_MAX;
        goal.ctrl_cmd.traj_mode = MOTION_CONTROL_GOAL_LINEAR_TRAJ;

        sendMotionControlActionGoal(goal);

        bool motionArrivedCopy = false;

        while (ros::ok())
        {
            std::this_thread::sleep_for(std::chrono::milliseconds(10));

            motionArrivedMutex.lock();
            motionArrivedCopy = motionArrived;
            motionArrivedMutex.unlock();

            if(motionArrivedCopy)
            {
                break;
            }
        }

        motionArrivedMutex.lock();
        motionArrived = false;
        motionArrivedMutex.unlock();

        return true;
    }
    else
    {
        ROS_ERROR("[MotionGeneral::motion] connect failed , motion control module will not work ! please check action server's status.");
        return false;
    }
}

void MotionGeneral::sendMotionControlActionGoal(const MotionControlActionGoal &goal)
{
    motionControlActionClient.sendGoal(goal,
                                       boost::bind(&MotionGeneral::done, this, _1, _2),
                                       boost::bind(&MotionGeneral::active, this),
                                       boost::bind(&MotionGeneral::feedback, this, _1));
}

void MotionGeneral::cancelMotionControlActionGoal()
{
    motionControlActionClient.cancelGoal();
}

void MotionGeneral::done(const actionlib::SimpleClientGoalState &state,
                         const MotionControlResultConstPtr &result)
{
    ROS_INFO("[MotionGeneral::done] motion action done...");

    motionArrivedMutex.lock();
    motionArrived = true;
    motionArrivedMutex.unlock();
}

void MotionGeneral::active()
{
    ROS_INFO("[MotionGeneral::active] motion action active...");
}

void MotionGeneral::feedback(const MotionControlFeedbackConstPtr &feedback)
{
    ROS_INFO("[MotionGeneral::feedback] motion action feedback.progress is %f", feedback->progress);
}

} /* namespace GeneralTaskControl */