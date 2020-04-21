#include <iostream>
#include <ros/ros.h>
#include "pso_algorithm.h"

using namespace std;

void LPath(particle_swarm_opt& test)
{
    vector<geometry_msgs::Pose2D> ref_path;
    geometry_msgs::Pose2D now_pose;
    float now_v;
    float now_w;
    ref_path.resize(4);
    ref_path[0].x = 0.2; ref_path[0].y = 0; ref_path[0].theta = 0;
    ref_path[1].x = 0.4; ref_path[1].y = 0; ref_path[1].theta = 0;
    ref_path[2].x = 0.6; ref_path[2].y = 0; ref_path[2].theta = 0;
    ref_path[3].x = 0.8; ref_path[3].y = 0; ref_path[3].theta = 0;
    now_pose.x = 0;
    now_pose.y = 0.2;
    now_pose.theta = -0.3;
    now_v = 1;
    now_w = 0;
    test.beginParticleSwarmOpt(ref_path, now_pose, now_v, now_w);
}

int main(int argc, char** argv)
{
    cout << "Program of PSO path tracking is beginning!" << endl;
    ros::init(argc, argv, "PSO_path_tracking");
    particle_swarm_opt test1;
    double last_time = ros::Time::now().toSec();
    LPath(test1);
    cout << "The PSO running time is: " << ros::Time::now().toSec() - last_time << endl;
    
    return 0;
}