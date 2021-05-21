#include <fstream>
#include <iostream>
#include <random>

#include "ros/ros.h"
#include "std_msgs/Float32.h"

double v = 0;
double w = 0;

void vCallback(const std_msgs::Float32 data) {
  v = data.data;
}

void wCallback(const std_msgs::Float32 data) {
  w = data.data;
}

int main(int argc, char **argv) {
  std::ofstream plot_file;
  const std::string home = getenv("HOME");
  plot_file.open((home + "/catkin_ws/src/robotics/plot.txt").c_str());

  ros::init(argc, argv, "talker");
  
  ros::NodeHandle n;

  ros::Publisher x_pub = n.advertise<std_msgs::Float32>("x_pub", 1000);
  ros::Publisher y_pub = n.advertise<std_msgs::Float32>("y_pub", 1000);
  ros::Publisher phi_pub = n.advertise<std_msgs::Float32>("phi_pub", 1000);

  ros::Publisher x_noise_pub = n.advertise<std_msgs::Float32>("x_noise_pub", 1000);
  ros::Publisher y_noise_pub = n.advertise<std_msgs::Float32>("y_noise_pub", 1000);
  ros::Publisher phi_noise_pub = n.advertise<std_msgs::Float32>("phi_noise_pub", 1000);

  ros::Subscriber v_sub = n.subscribe("v_pub", 1000, vCallback);
  ros::Subscriber w_sub = n.subscribe("w_pub", 1000, wCallback);

  ros::Rate loop_rate(100);

  double x_cur = 0.0;
  double y_cur = 0.0;
  double phi_cur = 0.0;
  
  double prev_time = ros::Time::now().toSec();

  const double mean = 0.0;
  const double std_dev = 0.5;
  std::default_random_engine generator;
  std::normal_distribution<double> dist(mean, std_dev);
  
  while (ros::ok()) {
    std_msgs::Float32 x;
    std_msgs::Float32 y;
    std_msgs::Float32 phi;
    
    x_cur = x_cur + v * sin(phi_cur) * (ros::Time::now().toSec() - prev_time);
    y_cur = y_cur + v * cos(phi_cur) * (ros::Time::now().toSec() - prev_time);
    phi_cur = phi_cur + w;

    x.data = x_cur;
    y.data = y_cur;
    phi.data = phi_cur;    

    plot_file<<x.data<<" "<<y.data<<"\n";
  
    x_pub.publish(x);
    y_pub.publish(y);
    phi_pub.publish(phi);

    x.data += dist(generator);
    y.data += dist(generator);
    phi.data += dist(generator);

    x_noise_pub.publish(x);
    y_noise_pub.publish(y);
    phi_noise_pub.publish(phi);

    ros::spinOnce();

    prev_time = ros::Time::now().toSec();
    loop_rate.sleep();
  }


  return 0;
}