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
  double k_v = 1.0;
  double k_w = 1.0;

  double x_ref = 1;
  double y_ref = 1;
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

  const int number_of_points = 3;
  int count = 0;

  while (ros::ok) {
     
    if (abs(x - x_ref) < tolerance && abs(y - y_ref) < tolerance) {
      count++;
      if (count == 1) {
        x_ref = 2;
        y_ref = 1;
        phi_ref = atan(x_ref / y_ref);
      }
      if (count == 2) {
        x_ref = 1.5;
        y_ref = 0.5;
        phi_ref = atan(x_ref / y_ref);
      }
      if (count == number_of_points) {
        v_cur = 0.0;
        w_cur = 0.0;
        v.data = v_cur;
        w.data = w_cur;
        v_pub.publish(v);
        w_pub.publish(w);
        std::cout<<"Goal reached"<<std::endl;
        break;
      }
    }

    v_cur = -k_v * ((x - x_ref) * sin(phi) + (y - y_ref) * cos(phi));
    if (v_cur > 0.5) {
      v_cur = 0.5;
    }
    if (v_cur < -0.5) {
      v_cur = -0.5;
    }

    v.data = v_cur;

    phi_ref = atan((x - x_ref) / (y - y_ref));
    w_cur = -k_w * (phi - phi_ref);
    if (w_cur > 0.01) {
      w_cur = 0.01;
    }
    if (w_cur < -0.01) {
      w_cur = -0.01;
    }

    w.data = w_cur;
 
    v_pub.publish(v);
    w_pub.publish(w);

    ros::spinOnce();

    loop_rate.sleep();
  }
  
  return 0;
}