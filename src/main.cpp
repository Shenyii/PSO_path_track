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
    float now_v = 0.1;
    float now_w = 0;
    ref_path.resize(1);
    // vel = test.beginParticleSwarmOpt(ref_path, now_pose, now_v, now_w);
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
        ref_path[0].x = x + 0.02; 
        ref_path[0].y = 0; 
        ref_path[0].theta = 0;
        addTrajectory(ref_path[0].x, ref_path[0].y);
        // for(int i = 1; i < 4; i++)
        // {
        //     ref_path[i].x = ref_path[i - 1].x + 0.2;
        //     ref_path[i].y = 0;
        //     ref_path[i].theta = 0;
        // }
        now_pose.x = x;
        now_pose.y = y;
        now_pose.theta = theta;
        vel = test.beginParticleSwarmOpt(ref_path, now_pose, now_v, now_w);
        now_v = vel.linear.x;
        now_w = vel.angular.z;
        pub_vel_g.publish(vel);

        det_t = ros::Time::now().toSec() - last_time;
        theta = theta + now_w * det_t;
        x = x + now_v * det_t * cos(theta);
        y = y + now_v * det_t * sin(theta);
        addTrajectory(x, y);
        cout << "The PSO running time is: " << ros::Time::now().toSec() - last_time << endl;
    }
}

int main(int argc, char** argv)
{
    cout << "Program of PSO path tracking is beginning!" << endl;
    ros::init(argc, argv, "PSO_path_tracking");
    ros::NodeHandle nh;
    //car_in_map_g = new Tf_Listerner("map", "base_footprint");
    pub_vel_g = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 2);
    pub_trajectory_g = nh.advertise<sensor_msgs::PointCloud>("/trajectory", 2);
    particle_swarm_opt test1;
    double last_time = ros::Time::now().toSec();
    LPath(test1);
    cout << "The PSO running time is: " << ros::Time::now().toSec() - last_time << endl;
    
    return 0;
}