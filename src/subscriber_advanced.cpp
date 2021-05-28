#include "ros/ros.h"
#include "std_msgs/Float32.h"

double x;
double y;
double phi;

void xCallback(const std_msgs::Float32 data) {
  x = data.data;
  ROS_INFO("x = %f", data.data);     
}

void yCallback(const std_msgs::Float32 data) {
  y = data.data;
  ROS_INFO("y = %f", data.data); 
}

void phiCallback(const std_msgs::Float32 data) {
  phi = data.data;
  ROS_INFO("phi = %f", data.data); 
}

int main(int argc, char **argv) {
  double c_1 = 1.0;
  double c_2 = 1.0;
  
  double k_v = 0.5;
  double k_w = 0.5;

  double x_ref = 8;
  double y_ref = -10;
  double phi_ref = atan(x_ref / y_ref);

  double v_cur;
  double w_cur;
  
  std_msgs::Float32 v;
  std_msgs::Float32 w;

  ros::init(argc, argv, "listener");

  ros::NodeHandle n;

  ros::Subscriber x_sub = n.subscribe("x_pub", 1000, xCallback);
  ros::Subscriber y_sub = n.subscribe("y_pub", 1000, yCallback);
  ros::Subscriber phi_sub = n.subscribe("phi_pub", 1000, phiCallback);

  ros::Publisher v_pub = n.advertise<std_msgs::Float32>("v_pub", 1000);
  ros::Publisher w_pub = n.advertise<std_msgs::Float32>("w_pub", 1000);
  

  ros::Rate loop_rate(100);

  double tolerance = 0.01;

  double prev_time = ros::Time::now().toSec();
  double start_time = ros::Time::now().toSec();

  while (ros::ok) {
     
    if (abs(x - x_ref) < tolerance && abs(y - y_ref) < tolerance) {
      v_cur = 0.0;
      w_cur = 0.0;
      v.data = v_cur;
      w.data = w_cur;
      v_pub.publish(v);
      w_pub.publish(w);
      std::cout<<"Goal reached"<<std::endl;
      break;
    }

    v_cur = -c_1 * (sin(phi) * (x + k_v * (x - x_ref) / (ros::Time::now().toSec() - prev_time) * (ros::Time::now().toSec() - start_time)) + 
                    cos(phi) * (y + k_v * (y - y_ref) / (ros::Time::now().toSec() - prev_time) * (ros::Time::now().toSec() - start_time)));
    if (v_cur > 0.5) {
      v_cur = 0.5;
    }
    if (v_cur < -0.5) {
      v_cur = -0.5;
    }

    v.data = v_cur;

    phi_ref = atan((x - x_ref) / (y - y_ref));
    w_cur = -c_2 * (phi + k_w * (phi - phi_ref) / (ros::Time::now().toSec() - prev_time) * (ros::Time::now().toSec() - start_time));
    if (w_cur > 0.1) {
      w_cur = 0.1;
    }
    if (w_cur < -0.1) {
      w_cur = -0.1;
    }

    w.data = w_cur;
 
    v_pub.publish(v);
    w_pub.publish(w);

    ros::spinOnce();

    prev_time = ros::Time::now().toSec();
    loop_rate.sleep();
  }
  
  return 0;
}