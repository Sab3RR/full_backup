//
// Created by sab3r on 25.03.20.
//

#include "PointArray.h"



void PointArray::handPoint(const std_msgs::Float64::ConstPtr &msg)
{
    if (secondpoint)
    {
        hendpoint.y = msg->data;
        secondpoint = false;
        bhendpoint = true;
    }
    else
    {
        hendpoint.x = msg->data;
        secondpoint = true;
        bhendpoint = false;
    }
}

void PointArray::PointTransmitter(const nav_msgs::Odometry::ConstPtr &msg)
{
    float lenght;
    std_msgs::Bool stop;

    stop.data = true;
    tf::Quaternion quat(msg->pose.pose.orientation.x, msg->pose.pose.orientation.y, msg->pose.pose.orientation.z, msg->pose.pose.orientation.w);
    tf::Vector3 vecto(msg->pose.pose.position.x, msg->pose.pose.position.y, 0);
    tf::Vector3 r = tf::quatRotate(quat, vecto);
    if (bhendpoint)
    {
        pubdest.publish(hendpoint);
        return;
    }
    if (points.size() == 0)
        return ;
    lenght = sqrtf(pow(vecto.x() - points[pointI].x, 2) + pow(vecto.y() - points[pointI].y, 2));
    //std::cout << points[pointI] << std::endl;
    if (lenght < 0.15)
    {
        pointI++;
//        if (points.size() <= pointI)
//        {
//            pubstop.publish(stop);
//            pointI--;
//        }
        if (points.size() <= pointI)
        {
            pointI = 0;
            loop++;
            std::cout << loop << std::endl;
        }
    }
    pubdest.publish(points[pointI]);
    if (pointI != 0)
        pubprevdest.publish(points[pointI - 1]);
}

void    PointArray::createArray(const algo::vector_array::ConstPtr &msg)
{
    algo::vector_msg v;
    tf2::Quaternion quat;
    tf2::Vector3 basic(1,0,0);
    tf2::Vector3 dest;
    tf2::Vector3 prevdest;
    tf2::Vector3 pDir;
    tf2::Vector3 buff;
    double angle;
    move_base_msgs::MoveBaseGoal goal;
    int i = 0;
    for (auto j = msg->vec.begin(); j != msg->vec.end(); j++)
    {


        if (i != 0)
        {
            dest.setX(j->x);
            dest.setY(j->y);
            dest.setZ(0);
            pDir = (dest - prevdest).normalized();
            angle = pDir.angle(basic);
            if (tf2::tf2Cross(pDir, basic).z() > 0) {
                angle *= -1;
            }
            buff = prevdest + pDir * 0.2;
            while ((buff - prevdest).length() < (dest - prevdest).length())
            {
                goal.target_pose.pose.position.x = buff.x();
                goal.target_pose.pose.position.y = buff.y();
                goal.target_pose.pose.position.z = 0;
                quat.setRPY(0, 0, angle);
                goal.target_pose.pose.orientation = tf2::toMsg(quat);
                buff += pDir * 0.2;
                goals.push_back(goal);
            }
            goal.target_pose.pose.position.x = dest.x();
            goal.target_pose.pose.position.y = dest.y();
            goal.target_pose.pose.position.z = 0;
            quat.setRPY(0, 0, angle);
        }
        else
        {
            dest.setX(j->x);
            dest.setY(j->y);
            dest.setZ(0);
            goal.target_pose.pose.position.x = dest.x();
            goal.target_pose.pose.position.y = dest.y();
            goal.target_pose.pose.position.z = 0;
            quat.setRPY(0, 0, 0);
        }
        v.x = j->x;
        v.y = j->y;
        prevdest = dest;
        points.push_back(v);
        goals.push_back(goal);
        i++;
    }
    havegoals = true;
}

PointArray::PointArray(ros::NodeHandle *n)
{
    algo::vector_msg point;

    subpos = n->subscribe("odom", 0, &PointArray::PointTransmitter, this);
    subpoints = n->subscribe("handpoint", 0, &PointArray::handPoint, this);
    subdots = n->subscribe("dots", 0, &PointArray::createArray, this);
    pubdest = n->advertise<algo::vector_msg>("destination", 1);
    pubprevdest = n->advertise<algo::vector_msg>("prevdestination", 1);

    pubstop = n->advertise<std_msgs::Bool>("stop", 1);
//    ac = &acc;
//    while(!ac->waitForServer(ros::Duration(5.0))){
//        ROS_INFO("Waiting for the move_base action server to come up");
//    }

    /* point.x = 1;
     point.y = 0;
     points.push_back(point);
     point.y = 0.25;
     points.push_back(point);
     point.x = 0;
     points.push_back(point);
     point.y = 0.5;
     points.push_back(point);
     point.x = 1;
     points.push_back(point);
     point.y = 0.75;
     points.push_back(point);
     point.x = 0;
     points.push_back(point);
     point.y = 1;
     points.push_back(point);
     point.x = 1;
     points.push_back(point);
     point.y = 1.25;
     points.push_back(point);
     point.x = 0;
     points.push_back(point);
     point.y = 1.5;
     points.push_back(point);
     point.x = 1;
     points.push_back(point);
     point.y = 1.75;
     points.push_back(point);
     point.x = 0;
     points.push_back(point);
     point.y = 2;
     points.push_back(point);
     point.x = 1;
     points.push_back(point);
     point.x = 0;
     point.y = 0;
     points.push_back(point);*/

}

int main(int acc, char** av)
{
    PointArray *pointarray;
    ros::init(acc, av, "PointArray");
    ros::NodeHandle n;

//    MoveBaseClient acc("move_base", true);
    pointarray = new PointArray(&n);
    MoveBaseClient ac("move_base", true);
    while(!ac.waitForServer(ros::Duration(5.0))){
        ROS_INFO("Waiting for the move_base action server to come up");
    }
    tf2::Quaternion quat;
    tf2::Vector3 basic(1,0,0);
    tf2::Vector3 dest;
    tf2::Vector3 prevdest;
    tf2::Vector3 pDir;
    move_base_msgs::MoveBaseGoal goal;
    int i = 0;
    auto igoals = pointarray->goals.begin();
    actionlib::SimpleClientGoalState lastgoal(actionlib::SimpleClientGoalState::LOST);
    bool sending = true;
    while (ros::ok())
    {

       double angle;
       if (pointarray->havegoals)
       {
           igoals = pointarray->goals.begin();
           pointarray->havegoals = false;
       }
        if (ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED) {
            if (sending == false)
            {
                igoals++;
                sending = true;
            }
            ROS_INFO("Hooray, the base moved 1 meter forward");
        } else if (ac.getState() == actionlib::SimpleClientGoalState::ABORTED)
        {
            if (sending == false)
            {
                igoals++;
                sending = true;

            }
            ROS_INFO("The base failed to move forward 1 meter for some reason");
        }
        else if (ac.getState() == actionlib::SimpleClientGoalState::ACTIVE)
            ROS_INFO("Base moving");
        else if (ac.getState() == actionlib::SimpleClientGoalState::LOST)
            ROS_INFO("Base lost");
        if (igoals == pointarray->goals.end())
            igoals = pointarray->goals.begin();
        if (sending && igoals != pointarray->goals.end()) {
            //we'll send a goal to the robot to move 1 meter forward
//            lastgoal = ac.getState();
            igoals->target_pose.header.frame_id = "map";
            igoals->target_pose.header.stamp = ros::Time::now();
//            if (pointarray->points.size() != 0) {
//                goal.target_pose.pose.position.x = pointarray->points[i].x;
//                goal.target_pose.pose.position.y = pointarray->points[i].y;
//                goal.target_pose.pose.position.z = 0;
//                if (i == 0)
//                    quat.setRPY(0, 0, 0);
//                else {
//                    dest.setX(pointarray->points[i].x);
//                    dest.setY(pointarray->points[i].y);
//                    dest.setZ(0);
//                    prevdest.setX(pointarray->points[i - 1].x);
//                    prevdest.setY(pointarray->points[i - 1].y);
//                    prevdest.setZ(0);
//                    pDir = (dest - prevdest).normalized();
//                    angle = pDir.angle(basic);
//                    if (tf2::tf2Cross(pDir, basic).z() > 0) {
//                        angle *= -1;
//                    }
//                    quat.setRPY(0, 0, angle);
//                }
//                goal.target_pose.pose.orientation = tf2::toMsg(quat);
            ac.sendGoal(*igoals);
            ROS_INFO("Sending goal");
            sending = false;
            }



//    ac.waitForResult();


        ros::spinOnce();
    }
    return 0;
}