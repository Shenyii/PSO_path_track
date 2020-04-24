#include <iostream>
#include <ros/ros.h>
#include "pso_algorithm.h"
#include "geometry_msgs/Twist.h"
#include "tf_listerner.h"
#include "nav_msgs/Path.h"

using namespace std;

ros::Publisher pub_vel_g;
ros::Publisher pub_trajectory_g;
Tf_Listerner* car_in_map_g;
float now_v_g = 0;
float now_w_g = 0;
ros::Subscriber sub_vel_;

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

void LPath(particle_swarm_opt& test)
{
    geometry_msgs::Twist vel;
    vector<geometry_msgs::Pose2D> ref_path;
    geometry_msgs::Pose2D now_pose;
    ref_path.resize(4);
    double last_time = ros::Time::now().toSec();
    double det_t = 0.15;
    float x = 0;
    float y = -0.1;
    float theta = 1;
    float v;
    float w;
    while(ros::ok())
    {
        last_time = ros::Time::now().toSec();
        ros::spinOnce();
        ref_path[0].x = car_in_map_g->x() + 0.1; 
        ref_path[0].y = 0; 
        ref_path[0].theta = 0;
        //addTrajectory(ref_path[0].x, ref_path[0].y);
        for(int i = 1; i < 4; i++)
        {
            ref_path[i].x = ref_path[i - 1].x + 0.1;
            ref_path[i].y = 0;
            ref_path[i].theta = 0;
        }
        now_pose.x = car_in_map_g->x();
        now_pose.y = car_in_map_g->y();
        theta = acos(2 * car_in_map_g->ow() * car_in_map_g->ow() - 1);
        now_pose.theta = car_in_map_g->oz() * car_in_map_g->ow() < 0 ? (0 - theta) : theta;
        vel = test.beginParticleSwarmOpt(ref_path, now_pose, now_v_g, now_w_g);
        pub_vel_g.publish(vel);
        addTrajectory(now_pose.x, now_pose.y);
        cout << "The PSO running time is: " << ros::Time::now().toSec() - last_time << endl;
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
    pub_vel_g = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 2);
    pub_trajectory_g = nh.advertise<sensor_msgs::PointCloud>("/trajectory", 2);
    sub_vel_ = nh.subscribe("/actural_velocity_of_car", 1, subCmdVel);
    particle_swarm_opt test1;
    double last_time = ros::Time::now().toSec();
    LPath(test1);
    cout << "The PSO running time is: " << ros::Time::now().toSec() - last_time << endl;
    
    return 0;
}