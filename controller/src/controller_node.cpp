#include "ros/ros.h"
#include <eigen3/Eigen/Eigen>
#include <geometry_msgs/Point.h>
#include <cmath>

const float PI = 3.14159;
bool greater = true;

using namespace Eigen;

double inc = 0;
int sw = 1;

double Time_now, Time_Start;
Eigen::Vector3d msg;
geometry_msgs::Point _msg;
Eigen::Vector3d Start;
Eigen::Vector3d Inter;
Eigen::Vector3d End;
Eigen::Vector3d Tmp;
Eigen::Vector3d End_Pos;

void obstacle_1_callback(const geometry_msgs::Point::ConstPtr& msg){
  Start.x() = msg->x;  Start.y() = msg->y;  Start.z() = msg->z;

}

void obstacle_2_callback(const geometry_msgs::Point::ConstPtr& msg){
  Inter.x() = msg->x;  Inter.y() = msg->y;  Inter.z() = msg->z;

}

void obstacle_3_callback(const geometry_msgs::Point::ConstPtr& msg){
  End.x() = msg->x;   End.y() = msg->y;     End.z() = msg->z;
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


int main(int argc, char **argv) {

  ros::init(argc,argv, "controller_node");
  ros::NodeHandle nh_;
  ros::Subscriber obstacle_1 = nh_.subscribe("/Obstacle_1",1000, obstacle_1_callback);
  ros::Subscriber obstacle_2 = nh_.subscribe("/Obstacle_2",1000, obstacle_2_callback);
  ros::Subscriber obstacle_3 = nh_.subscribe("/Obstacle_3",1000, obstacle_3_callback);
  ros::Publisher position_pub = nh_.advertise<geometry_msgs::Point>("/Position", 1000);


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
        // std::cout << "1" << std::endl;
        End_Pos = bezier_curve(End,Inter,Start,inc);
        if(inc > 1){
          inc = 0;
          sw = 2;
        }
        break;
      case 2:
      // std::cout << "2" << std::endl;
        End_Pos = bezier_curve(Start,Inter,End,inc);
        if(inc > 1){
          inc = 0;
          sw = 1;
        }
        break;
    }


    _msg.x = End_Pos.x();
    _msg.y = End_Pos.y();
    _msg.z = End_Pos.z();

    // std::cin>>x>>y>>z;

    // msg = inverse_kinematic(x,y,z);
    // _msg.joint_1 = msg(0);
    // _msg.joint_2 = msg(1);
    // _msg.joint_3 = msg(2);

    // std::cout << "x : "<< End_Pos.x() << std::endl;
    // std::cout << "y : "<< End_Pos.y() << std::endl;
    // std::cout << "z : "<< End_Pos.z() << std::endl;
    // std::cout << "Time : "<< Time_Start << std::endl;

    position_pub.publish(_msg);

    ros::spinOnce();
  }
  return 0;
}
