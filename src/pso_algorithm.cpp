#include "pso_algorithm.h"

particle_swarm_opt::particle_swarm_opt()
{
    srand(time(0));
    num_of_particles_ = 400;
    num_of_particles2_ = 100;
    num_of_iterate_ = 10;
    now_x_ = 0;
    now_y_ = 0;
    now_theta_ = 0;
    now_v_ = 0;
    now_w_ = 0;
    max_lin_acc_ = 1;
    max_ang_acc_ = 1;
    det_T_ = 0.2;
    det_t_ = 0.2;
    pub_trajectory_ = n_.advertise<sensor_msgs::PointCloud>("pre_trajectory", 3);
    pub_ref_path_ = n_.advertise<sensor_msgs::PointCloud>("ref_path", 3);
}

particle_swarm_opt::~particle_swarm_opt()
{}

geometry_msgs::Twist particle_swarm_opt::beginParticleSwarmOpt(vector<geometry_msgs::Pose2D> ref_path, vector<geometry_msgs::Pose2D> ref_path2, geometry_msgs::Pose2D now_pose, float now_v, float now_w, double det_t, float ave_vel)
{
    geometry_msgs::Twist ans;
    ref_path_.clear();
    for(int i = 0; i < ref_path.size(); i++)
    {
        ref_path_.push_back(ref_path[i]);
    }
    ref_path2_.clear();
    for(int i = 0; i < ref_path2.size(); i++)
    {
        ref_path2_.push_back(ref_path2[i]);
    }
    now_x_ = now_pose.x;
    now_y_ = now_pose.y;
    now_theta_ = now_pose.theta;
    now_v_ = now_v;
    now_w_ = now_w;
    det_t_ = det_t_ - det_t;
    ave_vel_ = ave_vel;
    initParticleSwarm();
    for(int i = 0; i < num_of_iterate_; i++)
    {
        updateParticleSwarm();
    }
    // initParticleSwarm2();
    // for(int i = 0; i < num_of_iterate_ - 5; i++)
    // {
    //     updateParticleSwarm2();
    // }
    // printParticle(swarm_[0]); 
    //printParticle(swarm2_[0]); 
    displayTrajectory(swarm_[0], 0.9);
    displayRefPath(49);
    // if(swarm_[0].fit_value_ > swarm2_[0].fit_value_)
    // {
    //     ans.linear.x = swarm2_[0].v_[0];
    //     ans.angular.z = swarm2_[0].w_[0];
    //     last_best_swarm_ = swarm2_[0];
    // }
    // else
    // {
        ans.linear.x = swarm_[0].v_[0];
        ans.angular.z = swarm_[0].w_[0];
        last_best_swarm_ = swarm_[0];
    // }
    return ans;
}

void particle_swarm_opt::initParticleSwarm()
{
    swarm_.clear();
    swarm_.resize(num_of_particles_);
    for(int i = 0; i < num_of_particles_; i++)
    {
        swarm_[i].v_.resize(ref_path_.size());
        swarm_[i].w_.resize(ref_path_.size());
        for(int j = 0; j < ref_path_.size(); j++)
        {
            float a, b;
            if(j == 0)
            {
                a = now_v_ - max_lin_acc_ * det_T_;
                b = now_v_ + max_lin_acc_ * det_T_;
                a = a < 0 ? 0 : a;
                b = b < 0 ? 0 : b;
                swarm_[i].v_[j] = myRand(a, b);
                a = now_w_ - max_ang_acc_ * det_T_;
                b = now_w_ + max_ang_acc_ * det_T_;
                swarm_[i].w_[j] = myRand(a, b);
            }
            else
            {
                a = swarm_[i].v_[j - 1] - max_lin_acc_ * det_T_;
                b = swarm_[i].v_[j - 1] + max_lin_acc_ * det_T_;
                a = a < 0 ? 0 : a;
                b = b < 0 ? 0 : b;
                swarm_[i].v_[j] = myRand(a, b);
                a = swarm_[i].w_[j - 1] - max_ang_acc_ * det_T_;
                b = swarm_[i].w_[j - 1] + max_ang_acc_ * det_T_;
                swarm_[i].w_[j] = myRand(a, b);
            }
        }
        swarm_[i].fit_value_ = calculateFitnessValueTest(swarm_[i]);
        //printParticle(swarm_[i]);
    }
}

void particle_swarm_opt::initParticleSwarm2()
{
    swarm2_.clear();
    swarm2_.resize(num_of_particles2_);
    for(int i = 0; i < num_of_particles2_; i++)
    {
        swarm2_[i].v_.resize(swarm_[0].v_.size() + 1);
        swarm2_[i].w_.resize(swarm_[0].w_.size() + 1);
        if(det_t_ > 0)
        {
            if(last_best_swarm_.v_.size() == swarm_[0].v_.size())
            {
                for(int j = 0; j < last_best_swarm_.v_.size(); j++)
                {
                    swarm2_[i].v_[j] = last_best_swarm_.v_[j];
                    swarm2_[i].w_[j] = last_best_swarm_.w_[j];
                }
                float a = swarm2_[i].v_[last_best_swarm_.v_.size() - 1] - max_lin_acc_ * det_T_;
                float b = swarm2_[i].v_[last_best_swarm_.v_.size() - 1] + max_lin_acc_ * det_T_;
                a = a < 0 ? 0 : a;
                b = b < 0 ? 0 : b;
                swarm2_[i].v_[last_best_swarm_.v_.size()] = myRand(a, b);
                a = swarm2_[i].w_[last_best_swarm_.v_.size() - 1] - max_ang_acc_ * det_T_;
                b = swarm2_[i].w_[last_best_swarm_.v_.size() - 1] + max_ang_acc_ * det_T_;
                swarm2_[i].w_[last_best_swarm_.v_.size()] = myRand(a, b);
            }
            else if(last_best_swarm_.v_.size() == swarm_[0].v_.size() + 1)
            {
                for(int j = 0; j < last_best_swarm_.v_.size() + 1; j++)
                {
                    swarm2_[i].v_[j] = last_best_swarm_.v_[j];
                    swarm2_[i].w_[j] = last_best_swarm_.w_[j];
                }
            }
        }
        else
        {
            det_t_ += det_T_;
            if(last_best_swarm_.v_.size() == swarm_[0].v_.size())
            {
                for(int j = 0; j < last_best_swarm_.v_.size() - 1; j++)
                {
                    swarm2_[i].v_[j] = last_best_swarm_.v_[j + 1];
                    swarm2_[i].w_[j] = last_best_swarm_.w_[j + 1];
                }
                float a = swarm2_[i].v_[last_best_swarm_.v_.size() - 2] - max_lin_acc_ * det_T_;
                float b = swarm2_[i].v_[last_best_swarm_.v_.size() - 2] + max_lin_acc_ * det_T_;
                a = a < 0 ? 0 : a;
                b = b < 0 ? 0 : b;
                swarm2_[i].v_[last_best_swarm_.v_.size() - 1] = myRand(a, b);
                a = swarm2_[i].w_[last_best_swarm_.v_.size() - 2] - max_ang_acc_ * det_T_;
                b = swarm2_[i].w_[last_best_swarm_.v_.size() - 2] + max_ang_acc_ * det_T_;
                swarm2_[i].w_[last_best_swarm_.v_.size() - 1] = myRand(a, b);
                a = swarm2_[i].v_[last_best_swarm_.v_.size() - 1] - max_lin_acc_ * det_T_;
                b = swarm2_[i].v_[last_best_swarm_.v_.size() - 1] + max_lin_acc_ * det_T_;
                a = a < 0 ? 0 : a;
                b = b < 0 ? 0 : b;
                swarm2_[i].v_[last_best_swarm_.v_.size()] = myRand(a, b);
                a = swarm2_[i].w_[last_best_swarm_.v_.size() - 1] - max_ang_acc_ * det_T_;
                b = swarm2_[i].w_[last_best_swarm_.v_.size() - 1] + max_ang_acc_ * det_T_;
                swarm2_[i].w_[last_best_swarm_.v_.size()] = myRand(a, b);
            }
            else if(last_best_swarm_.v_.size() == swarm_[0].v_.size() + 1)
            {
                for(int j = 0; j < last_best_swarm_.v_.size() - 1; j++)
                {
                    swarm2_[i].v_[j] = last_best_swarm_.v_[j + 1];
                    swarm2_[i].w_[j] = last_best_swarm_.w_[j + 1];
                }
                float a = swarm2_[i].v_[last_best_swarm_.v_.size() - 2] - max_lin_acc_ * det_T_;
                float b = swarm2_[i].v_[last_best_swarm_.v_.size() - 2] + max_lin_acc_ * det_T_;
                a = a < 0 ? 0 : a;
                b = b < 0 ? 0 : b;
                swarm2_[i].v_[last_best_swarm_.v_.size() - 1] = myRand(a, b);
                a = swarm2_[i].w_[last_best_swarm_.w_.size() - 2] - max_ang_acc_ * det_T_;
                b = swarm2_[i].w_[last_best_swarm_.w_.size() - 2] + max_ang_acc_ * det_T_;
                swarm2_[i].w_[last_best_swarm_.v_.size() - 1] = myRand(a, b);
            }
        }
    }
    calculateFitnessValue2();
}

void particle_swarm_opt::updateParticleSwarm()
{
    sort(swarm_.begin(), swarm_.end());
    float a, b;
    for(int i = 1; i < swarm_.size() / 4; i++)
    {
        for(int j = 0; j < ref_path_.size(); j++)
        {
            a = swarm_[0].v_[j] - 0.1 * max_lin_acc_ * det_T_;
            b = swarm_[0].v_[j] + 0.1 * max_lin_acc_ * det_T_;
            a = now_v_ - a > max_lin_acc_ * det_T_ ? (now_v_ - max_lin_acc_ * det_T_) : a;
            b = b - now_v_ > max_lin_acc_ * det_T_ ? (now_v_ + max_lin_acc_ * det_T_) : b;
            a = a < 0 ? 0 : a;
            b = b < 0 ? 0 : b;
            swarm_[i].v_[j] = myRand(a, b);
            a = swarm_[0].w_[j] - 0.1 * max_ang_acc_ * det_T_;
            b = swarm_[0].w_[j] + 0.1 * max_ang_acc_ * det_T_;
            a = now_w_ - a > max_ang_acc_ * det_T_ ? (now_w_ - max_ang_acc_ * det_T_) : a;
            b = b - now_w_ > max_ang_acc_ * det_T_ ? (now_w_ + max_ang_acc_ * det_T_) : b;
            swarm_[i].w_[j] = myRand(a, b);
        }
        swarm_[i].fit_value_ = calculateFitnessValueTest(swarm_[i]);
        //printParticle(swarm_[i]);
    }
    for(int i = swarm_.size() / 4; i < swarm_.size(); i++)
    {
        for(int j = 0; j < ref_path_.size(); j++)
        {
            if(j == 0)
            {
                a = now_v_ - max_lin_acc_ * det_T_;
                b = now_v_ + max_lin_acc_ * det_T_;
                a = a < 0 ? 0 : a;
                b = b < 0 ? 0 : b;
                swarm_[i].v_[j] = myRand(a, b);
                a = now_w_ - max_ang_acc_ * det_T_;
                b = now_w_ + max_ang_acc_ * det_T_;
                swarm_[i].w_[j] = myRand(a, b);
            }
            else
            {
                a = swarm_[i].v_[j - 1] - max_lin_acc_ * det_T_;
                b = swarm_[i].v_[j - 1] + max_lin_acc_ * det_T_;
                a = a < 0 ? 0 : a;
                b = b < 0 ? 0 : b;
                swarm_[i].v_[j] = myRand(a, b);
                a = swarm_[i].w_[j - 1] - max_ang_acc_ * det_T_;
                b = swarm_[i].w_[j - 1] + max_ang_acc_ * det_T_;
                swarm_[i].w_[j] = myRand(a, b);
            }
        }
        swarm_[i].fit_value_ = calculateFitnessValueTest(swarm_[i]);
        //printParticle(swarm_[i]);
    }
}

void particle_swarm_opt::updateParticleSwarm2()
{
    sort(swarm2_.begin(), swarm2_.end());
    float a, b;
    for(int i = 1; i < swarm2_.size(); i++)
    {
        for(int j = 0; j < swarm2_[0].v_.size(); j++)
        {
            a = swarm2_[0].v_[j] - 0.1 * max_lin_acc_ * det_T_;
            b = swarm2_[0].v_[j] + 0.1 * max_lin_acc_ * det_T_;
            a = now_v_ - a > max_lin_acc_ * det_T_ ? (now_v_ - max_lin_acc_ * det_T_) : a;
            b = b - now_v_ > max_lin_acc_ * det_T_ ? (now_v_ + max_lin_acc_ * det_T_) : b;
            a = a < 0 ? 0 : a;
            b = b < 0 ? 0 : b;
            swarm2_[i].v_[j] = myRand(a, b);
            a = swarm2_[0].w_[j] - 0.1 * max_ang_acc_ * det_T_;
            b = swarm2_[0].w_[j] + 0.1 * max_ang_acc_ * det_T_;
            a = now_w_ - a > max_ang_acc_ * det_T_ ? (now_w_ - max_ang_acc_ * det_T_) : a;
            b = b - now_w_ > max_ang_acc_ * det_T_ ? (now_w_ + max_ang_acc_ * det_T_) : b;
            swarm2_[i].w_[j] = myRand(a, b);
        }
    }
    calculateFitnessValue2();
}

float particle_swarm_opt::calculateFitnessValue(one_particle particle)
{
    float ans = 0;
    float x = now_x_;
    float y = now_y_;
    float theta = now_theta_;
    float v;
    float w;

    for(int i = 0; i < particle.v_.size(); i++)
    {
        v = particle.v_[i];
        w = particle.w_[i];
        w = w == 0 ? 0.000001 : w;
        x = x + v / w * (sin(theta + w * det_T_) - sin(theta));
        y = y + v / w * (cos(theta) - cos(theta + w * det_T_));
        theta += w * det_T_;
        float det_x0 = x - ref_path_[i].x;
        float det_y0 = y - ref_path_[i].y;
        float det_theta0 = theta - ref_path_[i].theta;
        det_theta0 = det_theta0 > pi ? det_theta0 - 2 * pi : det_theta0;
        det_theta0 = det_theta0 < -pi ? det_theta0 + 2 * pi : det_theta0;
        float det_x1 = det_x0 * cos(ref_path_[i].theta) + det_y0 * sin(ref_path_[i].theta);
        float det_y1 = det_y0 * cos(ref_path_[i].theta) - det_x0 * sin(ref_path_[i].theta);
        ans = ans + 0.3 * det_x1 * det_x1 + 1.5 * det_y1 * det_y1 + 0.3 * det_theta0 * det_theta0;
    }
    return ans;
}

float particle_swarm_opt::calculateFitnessValueTest(one_particle particle)
{
    float ans = 0;
    float x = now_x_;
    float y = now_y_;
    float theta = now_theta_;
    float v;
    float w;
    vector<float> xx;
    vector<float> yy;
    vector<float> ang;
    xx.push_back(now_x_);
    yy.push_back(now_y_);
    ang.push_back(now_theta_);
    for(int i = 0; i < particle.v_.size(); i++)
    {
        v = particle.v_[i];
        w = particle.w_[i];
        w = w == 0 ? 0.000001 : w;
        x = x + v / w * (sin(theta + w * det_T_) - sin(theta));
        y = y + v / w * (cos(theta) - cos(theta + w * det_T_));
        theta += w * det_T_;
        xx.push_back(x);
        yy.push_back(y);
        ang.push_back(theta);
    }
    vector<geometry_msgs::Pose2D> ref_path;
    ref_path.resize(particle.v_.size());
    float d = 0;
    for(int i = 0; i < particle.v_.size(); i++)
    {
        d += sqrt((xx[i + 1] - xx[i]) * (xx[i + 1] - xx[i]) + (yy[i + 1] - yy[i]) * (yy[i + 1] - yy[i]));
        ref_path[i] = ref_path2_[500 * d / pi];
    }
    for(int i = 0; i < particle.v_.size(); i++)
    {
        float det_x0 = xx[i + 1] - ref_path[i].x;
        float det_y0 = yy[i + 1] - ref_path[i].y;
        float det_theta0 = ang[i + 1] - ref_path[i].theta;
        det_theta0 = det_theta0 > pi ? det_theta0 - 2 * pi : det_theta0;
        det_theta0 = det_theta0 < -pi ? det_theta0 + 2 * pi : det_theta0;
        float det_x1 = det_x0 * cos(ref_path[i].theta) + det_y0 * sin(ref_path[i].theta);
        float det_y1 = det_y0 * cos(ref_path[i].theta) - det_x0 * sin(ref_path[i].theta);
        ans = ans + 0.3 * det_x1 * det_x1 + 1.5 * det_y1 * det_y1 + 0.3 * det_theta0 * det_theta0;
    }
    ans += 0.1 * fabs(ave_vel_ * det_T_ * particle.v_.size() - d);
    return ans;
}

void particle_swarm_opt::calculateFitnessValue2()
{
    if(swarm2_.size() == 0) return;
    float x = now_x_;
    float y = now_y_;
    float theta = now_theta_;
    float v1, v2;
    float w1, w2;
    for(int i = 0; i < swarm2_.size(); i++)
    {
        swarm2_[i].fit_value_ = 0;
        for(int j = 0; j < swarm2_[i].v_.size() - 1; j++)
        {
            v1 = swarm2_[i].v_[j];
            w1 = swarm2_[i].w_[j];
            w1 = w1 == 0 ? 0.0000001 : w1;
            v2 = swarm2_[i].v_[j + 1];
            w2 = swarm2_[i].w_[j + 1];
            w2 = w2 == 0 ? 0.0000001 : w2;
            x = x + v1 / w1 * (sin(theta + w1 * det_t_) - sin(theta));
            x = x + v2 / w2 * (sin(theta + w2 * (det_T_ - det_t_)) - sin(theta));
            y = y + v1 / w1 * (cos(theta) - cos(theta + w1 * det_t_));
            y = y + v2 / w2 * (cos(theta) - cos(theta + w2 * (det_T_ - det_t_)));
            theta += w1 * det_t_ + w2 * (det_T_ - det_t_);
            float det_x0 = x - ref_path_[j].x;
            float det_y0 = y - ref_path_[j].y;
            float det_theta0 = theta - ref_path_[j].theta;
            det_theta0 = det_theta0 > pi ? det_theta0 - 2 * pi : det_theta0;
            det_theta0 = det_theta0 < -pi ? det_theta0 + 2 * pi : det_theta0;
            float det_x1 = det_x0 * cos(ref_path_[j].theta) + det_y0 * sin(ref_path_[j].theta);
            float det_y1 = det_y0 * cos(ref_path_[j].theta) - det_x0 * sin(ref_path_[j].theta);
            swarm2_[i].fit_value_ += 0.3 * det_x1 * det_x1 + 1.5 * det_y1 * det_y1 + 0.3 * det_theta0 * det_theta0;
        }
    }
}

void particle_swarm_opt::printParticle(one_particle particle)
{
    cout << "********************" << endl;
    for(int i = 0; i < particle.v_.size(); i++)
    {
        cout << particle.v_[i] << ",  ";
    }
    cout << endl;
    for(int i = 0; i < particle.w_.size(); i++)
    {
        cout << particle.w_[i] << ",  ";
    }
    cout << endl;
    cout << particle.fit_value_ << endl;
    cout << "current pose:" << now_x_ << ", " << now_y_ << ", " << now_theta_ << endl;
    cout << "********************" << endl;
}

void particle_swarm_opt::displayTrajectory(one_particle particle, float values)
{
    sensor_msgs::PointCloud trajectory;
    trajectory.header.frame_id = "map";
    trajectory.points.resize(20);
    float x = now_x_;
    float y = now_y_;
    float theta = now_theta_;
    for(int i = 0; i < particle.v_.size(); i++)
    {
        particle.w_[i] = particle.w_[i] == 0 ? 0.000001 : particle.w_[i];
        for(int j = 0; j < 5; j++)
        {
            x = x + particle.v_[i] / particle.w_[i] * (sin(theta + particle.w_[i] * 0.04) - sin(theta));
            y = y + particle.v_[i] / particle.w_[i] * (cos(theta) - cos(theta + particle.w_[i] * 0.04));
            theta = theta + particle.w_[i] * 0.04;
            trajectory.points[5 * i + j].x = x;
            trajectory.points[5 * i + j].y = y;
            trajectory.points[5 * i + j].z = 0;
        }
    }
    // trajectory.channels.resize(1);
    // trajectory.channels[0].name = "distance";
    // for(int i = 0; i < 20; i++)
    // {
    //     trajectory.channels[0].values.push_back(i);
    // }
    pub_trajectory_.publish(trajectory);
}

void particle_swarm_opt::displayRefPath(float values)
{
    sensor_msgs::PointCloud trajectory;
    trajectory.header.frame_id = "map";
    trajectory.points.resize(ref_path_.size());
    for(int i = 0; i < trajectory.points.size(); i++)
    {
        trajectory.points[i].x = ref_path_[i].x;
        trajectory.points[i].y = ref_path_[i].y;
        trajectory.points[i].z = 0;
    }
    pub_ref_path_.publish(trajectory);
}

bool operator<(one_particle a, one_particle b)
{
    return a.fit_value_ < b.fit_value_;
}

float myRand(float a, float b)
{
    if(a < b)
    {
        return float(rand() % 10000) / 10000.0 * (b - a) + a;
    }
    else
    {
        return float(rand() % 10000) / 10000.0 * (a - b) + b;
    }
}