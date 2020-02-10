#include "ros/ros.h"
#include "manipulator_msgs/DoF3.h"
#include <eigen3/Eigen/Eigen>
#include <cmath>

const float PI = 3.1419;
bool greater = true;

using namespace Eigen;
const int L1 = 25;
const int L2 = 50;
const int L3 = 50;

double inc = 0;
int sw = 1;

double Time_now, Time_Start;
Eigen::Vector3d msg;
manipulator_msgs::DoF3 _msg;
Eigen::Vector3d Start;
Eigen::Vector3d Inter;
Eigen::Vector3d End;
Eigen::Vector3d Tmp;
Eigen::Vector3d End_Pos;


void joint_callback(const manipulator_msgs::DoF3::ConstPtr& msg){
    std::cout << "Joint 1\t" << msg->joint_1 <<std::endl;
    std::cout << "Joint 2\t" << msg->joint_2 <<std::endl;
    std::cout << "Joint 3\t" << msg->joint_3 <<std::endl;
}

void obstacle_1_callback(const manipulator_msgs::DoF3::ConstPtr& msg){
  Start.x() = msg->joint_1;  Start.y() = msg->joint_2;  Start.z() = msg->joint_3;

}

void obstacle_2_callback(const manipulator_msgs::DoF3::ConstPtr& msg){
  Inter.x() = msg->joint_1;  Inter.y() = msg->joint_2;  Inter.z() = msg->joint_3;

}

void obstacle_3_callback(const manipulator_msgs::DoF3::ConstPtr& msg){
  End.x() = msg->joint_1;   End.y() = msg->joint_2;     End.z() = msg->joint_3;
}

Eigen::Vector3d forward_kinematic(double joint_1, double joint_2, double joint_3){
    double x,y,z;
}

Eigen::Vector3d bezier_curve(Eigen::Vector3d Start,Eigen::Vector3d Inter, Eigen::Vector3d End, double Time){
  //P = (1-t)*P1 + t*P2
  //P = (1−t)^2*P1 + 2*(1−t)*t*P2 + t^2*P3
  //P = (1−t)^3*P1 + 3*(1−t)^2*t*P2 +3*(1−t)*t^2*P3 + t^3*P4

  Eigen::Vector3d Start_ = Start;
  Eigen::Vector3d Inter_ = Inter;
  Eigen::Vector3d End_   = End;
  Eigen::Vector3d End_Pos;

  End_Pos.x() = (pow((1-Time),2) * Start_.x()) + 2*(1-Time)*Time*Inter_.x() + pow(Time,3) * End_.x();
  End_Pos.y() = (pow((1-Time),2) * Start_.y()) + 2*(1-Time)*Time*Inter_.y() + pow(Time,3) * End_.y();
  End_Pos.z() = (pow((1-Time),2) * Start_.z()) + 2*(1-Time)*Time*Inter_.z() + pow(Time,3) * End_.z();

  return End_Pos;
}

Eigen::Vector3d inverse_kinematic(double x, double y, double z){

    double Joint1,Joint2,Joint3;
    double s3,c3;

    Eigen::Vector3d result;

    Joint1 = std::atan2(y,x);

    c3 = (pow(x,2) + pow(y,2) + pow((z - L1),2) - pow(L2,2) - pow(L3,2)) / (2*L2*L3);
    s3 = sqrt(1-pow(c3,2));

    Joint3 = atan2(s3,c3);
    Joint2 = atan2((L3*s3),(L2 + (L3*c3)));

    result(0) = Joint1;
    result(1) = Joint2;
    result(2) = Joint3;

    return result;
}

int main(int argc, char **argv) {

  ros::init(argc,argv, "controller_node");
  ros::NodeHandle nh_;
  ros::Subscriber joint_sub = nh_.subscribe("/servo_pos",1000, joint_callback);
  ros::Subscriber obstacle_1 = nh_.subscribe("/Obstacle_1",1000, obstacle_1_callback);
  ros::Subscriber obstacle_2 = nh_.subscribe("/Obstacle_2",1000, obstacle_2_callback);
  ros::Subscriber obstacle_3 = nh_.subscribe("/Obstacle_3",1000, obstacle_3_callback);
  ros::Publisher joint_pub = nh_.advertise<manipulator_msgs::DoF3>("/servo_cmd", 1000);


  while (ros::ok()) {

    // Start.x() = 1.0;  Start.y() = 0.0;  Start.z() = 0.0;
    // Inter.x() = 0.0;  Inter.y() = 0.7;  Inter.z() = 1.0;
    // End.x() = -1.0;   End.y() = 0.0;    End.z() = 0.0;

    Time_Start = ros::Time::now().toSec() - Time_now;


    if(Time_Start > 0.01){
      greater = true;
      inc+=0.002;
    }



    if(greater){
      Time_now = ros::Time::now().toSec();
      greater = false;
      // End_Pos = {0,0,1.25};
    }else{
      // End_Pos = bezier_curve(End,Inter, Start, Time_Start);
    }



    switch (sw) {
      case 1:
        std::cout << "1" << std::endl;
        End_Pos = bezier_curve(End,Inter,Start,inc);
        if(inc > 1){
          inc = 0;
          sw = 2;
        }
        break;
      case 2:
      std::cout << "2" << std::endl;
        End_Pos = bezier_curve(Start,Inter,End,inc);
        if(inc > 1){
          inc = 0;
          sw = 1;
        }
        break;
    }


    _msg.joint_1 = End_Pos.x();
    _msg.joint_2 = End_Pos.y();
    _msg.joint_3 = End_Pos.z();

    // std::cin>>x>>y>>z;

    // msg = inverse_kinematic(x,y,z);
    // _msg.joint_1 = msg(0);
    // _msg.joint_2 = msg(1);
    // _msg.joint_3 = msg(2);

    // std::cout << "x : "<< End_Pos.x() << std::endl;
    // std::cout << "y : "<< End_Pos.y() << std::endl;
    // std::cout << "z : "<< End_Pos.z() << std::endl;
    // std::cout << "Time : "<< Time_Start << std::endl;

    joint_pub.publish(_msg);

    ros::spinOnce();
  }
  return 0;
}
