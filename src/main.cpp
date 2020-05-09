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

float data_g[10];

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
    vector<geometry_msgs::Pose2D> ref_path2;
    geometry_msgs::Pose2D now_pose;
    ref_path.resize(4);
    double last_time = ros::Time::now().toSec();
    double det_t;
    float slope_anglar = 0;
    float slope = tan(slope_anglar);
    float ave_vel = 0.5;
    while(ros::ok())
    {
        ros::spinOnce();
        ref_path[0].x = (car_in_map_g->x() + slope * car_in_map_g->y()) / (slope * slope +1) + 0.2 * ave_vel * cos(slope_anglar); 
        ref_path[0].y = slope * ref_path[0].x + 0.2 * ave_vel * sin(slope_anglar); 
        ref_path[0].theta = slope_anglar;
        addTrajectory(ref_path[0].x, ref_path[0].y);
        for(int i = 1; i < 4; i++)
        {
            ref_path[i].x = ref_path[i - 1].x + 0.2 * ave_vel * cos(slope_anglar);
            ref_path[i].y = ref_path[i - 1].y + 0.2 * ave_vel * sin(slope_anglar);
            ref_path[i].theta = slope_anglar;
        }
        now_pose.x = car_in_map_g->x();
        now_pose.y = car_in_map_g->y();
        float theta = acos(2 * car_in_map_g->ow() * car_in_map_g->ow() - 1);
        now_pose.theta = car_in_map_g->oz() * car_in_map_g->ow() < 0 ? (0 - theta) : theta;
        vel = test.beginParticleSwarmOpt(ref_path, ref_path2, now_pose, now_v_g, now_w_g, det_t, ave_vel);
        pub_vel_g.publish(vel);
        addTrajectory(now_pose.x, now_pose.y);
        det_t = ros::Time::now().toSec() - last_time;
        cout << "The PSO running time is: " << det_t << endl << endl;
        last_time = ros::Time::now().toSec();
    }
}

void SPath(particle_swarm_opt& test)
{
    geometry_msgs::Twist vel;
    vector<geometry_msgs::Pose2D> ref_path;
    vector<geometry_msgs::Pose2D> ref_path2;
    geometry_msgs::Pose2D now_pose;
    ref_path.resize(4);
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
            for(int i = 0; i < ref_path.size(); i++)
            {
                ref_path[i] = ref[500 * (pi - theta0) / pi + (i + 1) * (500 * 0.2 * ave_vel / pi)];
            }
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
            for(int i = 0; i < ref_path.size(); i++)
            {
                ref_path[i] = ref[500 + 500 * (pi + theta0) / pi + (i + 1) * (500 * 0.2 * ave_vel / pi)];
            }
        }
        
        addTrajectory(ref_path[0].x, ref_path[0].y);
        now_pose.x = car_in_map_g->x();
        now_pose.y = car_in_map_g->y();
        float theta = acos(2 * car_in_map_g->ow() * car_in_map_g->ow() - 1);
        now_pose.theta = car_in_map_g->oz() * car_in_map_g->ow() < 0 ? (0 - theta) : theta;
        // //////////////////////////////
        // cout << now_pose.x << ", " << now_pose.y << ", " << now_pose.theta << endl;
        // cout << ref_path[0].x << ", " << ref_path[0].y << ", " << ref_path[0].theta << endl;
        // cout << ref_path[1].x << ", " << ref_path[1].y << ", " << ref_path[1].theta << endl;
        // cout << ref_path[2].x << ", " << ref_path[2].y << ", " << ref_path[2].theta << endl;
        // cout << ref_path[3].x << ", " << ref_path[3].y << ", " << ref_path[3].theta << endl;
        // //////////////////////////////
        vel = test.beginParticleSwarmOpt(ref_path, ref_path2, now_pose, now_v_g, now_w_g, det_t, ave_vel);
        pub_vel_g.publish(vel);
        addTrajectory(now_pose.x, now_pose.y);
        det_t = ros::Time::now().toSec() - last_time;
        cout << "The PSO running time is: " << det_t << endl << endl;
        last_time = ros::Time::now().toSec();
    }
}

void SPath2(particle_swarm_opt& test)
{
    geometry_msgs::Twist vel;
    vector<geometry_msgs::Pose2D> ref_path;
    vector<geometry_msgs::Pose2D> ref_path2;
    geometry_msgs::Pose2D now_pose;
    ref_path.resize(4);
    ref_path2.resize(150);
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
            for(int i = 0; i < ref_path.size(); i++)
            {
                ref_path[i] = ref[500 * (pi - theta0) / pi + (i + 1) * (500 * 0.2 * ave_vel / pi)];
            }
            for(int i = 0; i < ref_path2.size(); i++)
            {
                ref_path2[i] = ref[500 * (pi - theta0) / pi + i];
            }
            data_g[0] = ref[500 * (pi - theta0) / pi].x;
            data_g[1] = ref[500 * (pi - theta0) / pi].y;
            data_g[2] = ref[500 * (pi - theta0) / pi].theta;
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
            for(int i = 0; i < ref_path.size(); i++)
            {
                ref_path[i] = ref[500 + 500 * (pi + theta0) / pi + (i + 1) * (500 * 0.2 * ave_vel / pi)];
            }
            for(int i = 0; i < ref_path2.size(); i++)
            {
                ref_path2[i] = ref[500 + 500 * (pi + theta0) / pi + i];
            }
            data_g[0] = ref[500 + 500 * (pi + theta0) / pi].x;
            data_g[1] = ref[500 + 500 * (pi + theta0) / pi].y;
            data_g[2] = ref[500 + 500 * (pi + theta0) / pi].theta;
        }
        
        addTrajectory(ref_path[0].x, ref_path[0].y);
        now_pose.x = car_in_map_g->x();
        now_pose.y = car_in_map_g->y();
        float theta = acos(2 * car_in_map_g->ow() * car_in_map_g->ow() - 1);
        now_pose.theta = car_in_map_g->oz() * car_in_map_g->ow() < 0 ? (0 - theta) : theta;
        data_g[3] = now_pose.x;
        data_g[4] = now_pose.y;
        data_g[5] = now_pose.theta;
        // //////////////////////////////
        // cout << now_pose.x << ", " << now_pose.y << ", " << now_pose.theta << endl;
        // cout << ref_path[0].x << ", " << ref_path[0].y << ", " << ref_path[0].theta << endl;
        // cout << ref_path[1].x << ", " << ref_path[1].y << ", " << ref_path[1].theta << endl;
        // cout << ref_path[2].x << ", " << ref_path[2].y << ", " << ref_path[2].theta << endl;
        // cout << ref_path[3].x << ", " << ref_path[3].y << ", " << ref_path[3].theta << endl;
        // //////////////////////////////
        vel = test.beginParticleSwarmOpt(ref_path, ref_path2, now_pose, now_v_g, now_w_g, det_t, ave_vel);
        pub_vel_g.publish(vel);
        addTrajectory(now_pose.x, now_pose.y);
        det_t = ros::Time::now().toSec() - last_time;
        //cout << "The PSO running time is: " << det_t << endl << endl;
        cout << data_g[0] << ", " << data_g[1] << ", " << data_g[2] << ", "
             << data_g[3] << ", " << data_g[4] << ", " << data_g[5] << ", "
             << data_g[6] << ", " << data_g[7] << endl;
        last_time = ros::Time::now().toSec();
    }
}

void OPath(particle_swarm_opt& test)
{
    geometry_msgs::Twist vel;
    vector<geometry_msgs::Pose2D> ref_path;
    vector<geometry_msgs::Pose2D> ref_path2;
    geometry_msgs::Pose2D now_pose;
    ref_path.resize(4);
    double last_time = ros::Time::now().toSec();
    double det_t = 0.1;
    float ave_vel = 0.5;
    float theta0;
    float theta1;
    while(ros::ok())
    {
        ros::spinOnce();
        theta0 = atan(car_in_map_g->y() / (car_in_map_g->x() - 1));
        if((car_in_map_g->x() < 1) && (car_in_map_g->y() > 0))
        {
            theta0 = theta0 + pi;
        }
        else if((car_in_map_g->x() < 1) && (car_in_map_g->y() < 0))
        {
            theta0 = theta0 - pi;
        }
        theta1 = theta0 - pi / 2;
        for(int i = 0; i < ref_path.size(); i++)
        {
            ref_path[i].x = cos(theta0 - 0.2 * ave_vel * (i + 1)) + 1;
            ref_path[i].y = sin(theta0 - 0.2 * ave_vel * (i + 1));
            ref_path[i].theta = theta1 - 0.2 * ave_vel * (i + 1);
            ref_path[i].theta = ref_path[i].theta > pi ? ref_path[i].theta - 2 * pi : ref_path[i].theta;
            ref_path[i].theta = ref_path[i].theta < -pi ? ref_path[i].theta + 2 * pi : ref_path[i].theta;
        }
        addTrajectory(ref_path[0].x, ref_path[0].y);
        now_pose.x = car_in_map_g->x();
        now_pose.y = car_in_map_g->y();
        float theta = acos(2 * car_in_map_g->ow() * car_in_map_g->ow() - 1);
        now_pose.theta = car_in_map_g->oz() * car_in_map_g->ow() < 0 ? (0 - theta) : theta;
        //////////////////////////////
        cout << now_pose.x << ", " << now_pose.y << ", " << now_pose.theta << endl;
        cout << ref_path[0].x << ", " << ref_path[0].y << ", " << ref_path[0].theta << endl;
        cout << ref_path[1].x << ", " << ref_path[1].y << ", " << ref_path[1].theta << endl;
        cout << ref_path[2].x << ", " << ref_path[2].y << ", " << ref_path[2].theta << endl;
        cout << ref_path[3].x << ", " << ref_path[3].y << ", " << ref_path[3].theta << endl;
        //////////////////////////////
        vel = test.beginParticleSwarmOpt(ref_path, ref_path2, now_pose, now_v_g, now_w_g, det_t, ave_vel);
        pub_vel_g.publish(vel);
        addTrajectory(now_pose.x, now_pose.y);
        det_t = ros::Time::now().toSec() - last_time;
        cout << "The PSO running time is: " << det_t << endl << endl;
        last_time = ros::Time::now().toSec();
    }
}

void subCmdVel(geometry_msgs::Twist msg)
{
    now_v_g = msg.linear.x;
    now_w_g = msg.angular.z;
    data_g[6] = msg.linear.x;
    data_g[7] = msg.angular.z;
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
    SPath2(test1);
    cout << "The PSO running time is: " << ros::Time::now().toSec() - last_time << endl;
    
    return 0;
}