#ifndef PSO_ALGORITHM_H
#define PSO_ALGORITHM_H

#include <iostream>
#include <ros/ros.h>
#include <vector>
#include "geometry_msgs/Pose2D.h"
#include <time.h>
#include <algorithm>
#include "geometry_msgs/Twist.h"
#include "sensor_msgs/PointCloud.h"
#include <string>

using namespace std;

#define pi 3.14159265

class one_particle
{
public:
    float fit_value_;
    vector<float> v_;
    vector<float> w_;
};

class particle_swarm_opt
{
public:
    particle_swarm_opt();
    ~particle_swarm_opt();
    geometry_msgs::Twist beginParticleSwarmOpt(vector<geometry_msgs::Pose2D> ref_path, vector<geometry_msgs::Pose2D> ref_path2, geometry_msgs::Pose2D now_pose, float now_v, float now_w, double det_t, float ave_vel);

private:
    int num_of_particles_;
    int num_of_particles2_;
    int num_of_iterate_;
    float now_x_;
    float now_y_;
    float now_theta_;
    float now_v_;
    float now_w_;
    float max_lin_acc_;
    float max_ang_acc_;
    float det_T_;
    double det_t_;
    float ave_vel_;
    vector<one_particle> swarm_;
    one_particle last_best_swarm_;
    vector<one_particle> swarm2_;
    vector<geometry_msgs::Pose2D> ref_path_;
    vector<geometry_msgs::Pose2D> ref_path2_;

    ros::NodeHandle n_;
    ros::Publisher pub_trajectory_;
    ros::Publisher pub_ref_path_;

    void initParticleSwarm();
    void initParticleSwarm2();
    void updateParticleSwarm();
    void updateParticleSwarm2();
    float calculateFitnessValue(one_particle particle);
    float calculateFitnessValueTest(one_particle particle);
    void calculateFitnessValue2();

    void printParticle(one_particle particle);
    void displayTrajectory(one_particle particle, float values);
    void displayRefPath(float values);
};

bool operator<(one_particle a, one_particle b);

float myRand(float a, float b);

#endif