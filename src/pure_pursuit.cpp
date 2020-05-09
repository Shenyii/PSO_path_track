#include <iostream>
#include <ros/ros.h>
#include "geometry_msgs/Twist.h"
#include "tf_listerner.h"
#include "nav_msgs/Path.h"
#include "geometry_msgs/Pose2D.h"

using namespace std;

#define pi 3.14159265

ros::Publisher pub_vel_g;
ros::Publisher pub_trajectory_g;
Tf_Listerner* car_in_map_g;
float now_v_g = 0;
float now_w_g = 0;
ros::Subscriber sub_vel_;
float e_d_g;
float e_ang_g;

float v_g, w_g;
sensor_msgs::PointCloud trajectory;
void addTrajectory(float x, float y)
{
    trajectory.header.frame_id = "map";
    geometry_msgs::Point32 point;
    point.x = x;
    point.y = y;
    trajectory.points.push_back(point);
    pub_trajectory_g.publish(trajectory);
}

geometry_msgs::Twist pidControl(geometry_msgs::Pose2D now_pose, geometry_msgs::Pose2D ref_point)
{
    geometry_msgs::Twist ans;
    float ave_vel = 1;
    float x0;
    float y0;
    x0 = cos(now_pose.theta) * (ref_point.x - now_pose.x) + sin(now_pose.theta) * (ref_point.y - now_pose.y);
    y0 = cos(now_pose.theta) * (ref_point.y - now_pose.y) - sin(now_pose.theta) * (ref_point.x - now_pose.x);
    float r = (x0 * x0 + y0 * y0) / (2 * y0);
    ans.linear.x = ave_vel;
    ans.angular.z = ave_vel / r;
    if(ans.angular.z > 1000) 
    {
        ans.angular.z = 0;
        cout << "error!!!" << endl;
    }
    cout << ans.angular.z << endl;
    v_g = ans.linear.x;
    w_g = ans.angular.z;
    return ans;
}

void SPath2()
{
    geometry_msgs::Twist vel;
    geometry_msgs::Pose2D ref_path;
    geometry_msgs::Pose2D now_pose;
    double last_time = ros::Time::now().toSec();
    double det_t = 0.1;
    float ave_vel = 1;
    float theta0;
    vector<geometry_msgs::Pose2D> ref;
    ref.resize(1000);
    theta0 = pi;
    for(int i = 0; i < 500; i++)
    {
        ref[i].x = cos(theta0) + 1;
        ref[i].y = sin(theta0);
        ref[i].theta = theta0 - pi / 2;
        theta0 -= pi / 500;
    }
    theta0 = -pi;
    for(int i = 500; i < 1000; i++)
    {
        ref[i].x = cos(theta0) + 3;
        ref[i].y = sin(theta0);
        ref[i].theta = theta0 + pi / 2;
        theta0 += pi / 500;
    }

    while(ros::ok())
    {
        ros::spinOnce();
        
        now_pose.x = car_in_map_g->x();
        now_pose.y = car_in_map_g->y();
        float theta = acos(2 * car_in_map_g->ow() * car_in_map_g->ow() - 1);
        now_pose.theta = car_in_map_g->oz() * car_in_map_g->ow() < 0 ? (0 - theta) : theta;
        theta0 = atan(car_in_map_g->y() / (car_in_map_g->x() - 1));
        if((car_in_map_g->x() < 1) && (car_in_map_g->y() > 0))
        {
            theta0 = theta0 + pi;
        }
        else if((car_in_map_g->x() < 1) && (car_in_map_g->y() < 0))
        {
            theta0 = theta0 - pi;
        }
        if(theta0 > 0)
        {
            ref_path = ref[500 * (pi - theta0) / pi + 31];
        }
        else
        {
            theta0 = atan(car_in_map_g->y() / (car_in_map_g->x() - 3));
            if((car_in_map_g->x() < 3) && (car_in_map_g->y() > 0))
            {
                theta0 = theta0 + pi;
            }
            else if((car_in_map_g->x() < 3) && (car_in_map_g->y() < 0))
            {
                theta0 = theta0 - pi;
            }
            ref_path = ref[500 + 500 * (pi + theta0) / pi + 31];
        }
        
        addTrajectory(ref_path.x, ref_path.y);
        addTrajectory(now_pose.x, now_pose.y);
        vel = pidControl(now_pose, ref_path);
        pub_vel_g.publish(vel);
        det_t = ros::Time::now().toSec() - last_time;
        cout << "The PSO running time is: " << det_t << endl << endl;
        last_time = ros::Time::now().toSec();
        ros::Duration(0.05).sleep();
    }
}

void subCmdVel(geometry_msgs::Twist msg)
{
    now_v_g = msg.linear.x;
    now_w_g = msg.angular.z;
}

int main(int argc, char** argv)
{
    cout << "Program of PSO path tracking is beginning!" << endl;
    ros::init(argc, argv, "PSO_path_tracking");
    ros::NodeHandle nh;
    car_in_map_g = new Tf_Listerner("map", "base_footprint");
    ros::Duration(2).sleep();
    pub_vel_g = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 2);
    pub_trajectory_g = nh.advertise<sensor_msgs::PointCloud>("/trajectory", 2);
    sub_vel_ = nh.subscribe("/actural_velocity_of_car", 1, subCmdVel);
    double last_time = ros::Time::now().toSec();
    SPath2();
    cout << "The PSO running time is: " << ros::Time::now().toSec() - last_time << endl;
    
    return 0;
}