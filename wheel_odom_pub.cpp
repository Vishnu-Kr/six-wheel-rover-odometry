// *
//  * Automatic Addison
//  * Date: May 20, 2021
//  * ROS Version: ROS 1 - Melodic
//  * Website: https://automaticaddison.com
//  * Publishes odometry information for use with robot_pose_ekf package.
//  *   This odometry information is based on wheel encoder tick counts.
//  * Subscribe: ROS node that subscribes to the following topics:
//  *  right_ticks : Tick counts from the right motor encoder (std_msgs/Int32)
//  * 
//  *  left_ticks : Tick counts from the left motor encoder  (std_msgs/Int32)
//  * 
//  *  initial_2d : The initial position and orientation of the robot.
//  *               (geometry_msgs/PoseStamped)
//  *
//  * Publish: This node will publish to the following topics:
//  *  odom_data_euler : Position and velocity estimate. The orientation.z 
//  *                    variable is an Euler angle representing the yaw angle.
//  *                    (nav_msgs/Odometry)
//  *  odom_data_quat : Position and velocity estimate. The orientation is 
//  *                   in quaternion format.
//  *                   (nav_msgs/Odometry)
//  * Modified from Practical Robotics in C++ book (ISBN-10 : 9389423465)
//  *   by Lloyd Brombach
//  */
 
// Include various libraries
#include "ros/ros.h"
// #include "std_msgs/Int32.h"
#include "std_msgs/Int32.h"
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <cmath>
 
// Create odometry data publishers
ros::Publisher odom_data_pub;
ros::Publisher odom_data_pub_quat;
nav_msgs::Odometry odomNew;
nav_msgs::Odometry odomOld;
 
// Initial pose
const double initialX = 0.0;
const double initialY = 0.0;
const double initialTheta = 0.00000000001;
const double PI = 3.141592;
 
// Robot physical constants
const double TICKS_PER_REVOLUTION = 36124; // For reference purposes.
const double WHEEL_RADIUS = 0.11; // Wheel radius in meters
const double WHEEL_BASE = 0.89; // Center of left tire to center of right tire
const double TICKS_PER_METER = 52270; // Original was 2800
 
// Distance both wheels have traveled
double distanceLeft_Front= 0;
double distanceRight_Front= 0;
double distanceLeft_Middle = 0;
double distanceRight_Middle= 0;
double distanceLeft_Rear= 0;
double distanceRight_Rear= 0;
// Flag to see if initial pose has been received
bool initialPoseRecieved = false;
 
using namespace std;
 
// Get initial_2d message from either Rviz clicks or a manual pose publisher
void set_initial_2d(const geometry_msgs::PoseStamped &rvizClick) {
 
  odomOld.pose.pose.position.x = rvizClick.pose.position.x;
  odomOld.pose.pose.position.y = rvizClick.pose.position.y;
  odomOld.pose.pose.orientation.z = rvizClick.pose.orientation.z;
  initialPoseRecieved = true;
}
 
// Calculate the distance the left wheel has traveled since the last cycle
// void Calc_Left(const std_msgs::Int32& leftCount) {
 
//   static int lastCountL = 0;
//   if(leftCount.data != 0 && lastCountL != 0) {
         
//     int leftTicks = (leftCount.data - lastCountL);
 
//     if (leftTicks > 10000) {
//       leftTicks = 0 - (2147483648 - leftTicks);
//     }
//     else if (leftTicks < -10000) {
//       leftTicks = 2147483648-leftTicks;
//     }
//     else{}
//     distanceLeft = leftTicks/TICKS_PER_METER;
//   }
//   lastCountL = leftCount.data;
// }
void Calc_Left_Front(const std_msgs::Int32& leftCountFront) {
 
  static int lastCountLF = 0;
  if(leftCountFront.data != 0 && lastCountLF != 0) {
         
    int frontleftTicks = (leftCountFront.data - lastCountLF);
 
    if (frontleftTicks > 10000) {
      frontleftTicks = 0 - (2147483648 - frontleftTicks);
    }
    else if (frontleftTicks < -10000) {
      frontleftTicks = 2147483648-frontleftTicks;
    }
    else{}
    distanceLeft_Front = frontleftTicks/TICKS_PER_METER;
  }
  lastCountLF = leftCountFront.data;
}

void Calc_Left_Middle(const std_msgs::Int32& leftCountMiddle) {
 
  static int lastCountLM = 0;
  if(leftCountMiddle.data != 0 && lastCountLM != 0) {
         
    int middleleftTicks = (leftCountMiddle.data - lastCountLM);
 
    if (middleleftTicks > 10000) {
      middleleftTicks = 0 - (2147483648 - middleleftTicks);
    }
    else if (middleleftTicks < -10000) {
      middleleftTicks = 2147483648-middleleftTicks;
    }
    else{}
    distanceLeft_Middle = middleleftTicks/TICKS_PER_METER;
  }
  lastCountLM = leftCountMiddle.data;
}

void Calc_Left_Rear(const std_msgs::Int32& leftCountRear) {
 
  static int lastCountLR = 0;
  if(leftCountRear.data != 0 && lastCountLR != 0) {
         
    int rearleftTicks = (leftCountRear.data - lastCountLR);
 
    if (rearleftTicks > 10000) {
      rearleftTicks = 0 - (2147483648 - rearleftTicks);
    }
    else if (rearleftTicks < -10000) {
      rearleftTicks = 2147483648-rearleftTicks;
    }
    else{}
    distanceLeft_Rear = rearleftTicks/TICKS_PER_METER;
  }
  lastCountLR = leftCountRear.data;
}



void Calc_Right_Front(const std_msgs::Int32& rightCountFront) {
 
  static int lastCountLF = 0;
  if(rightCountFront.data != 0 && lastCountLF != 0) {
         
    int frontrightTicks = (rightCountFront.data - lastCountLF);
 
    if (frontrightTicks > 10000) {
      frontrightTicks = 0 - (2147483648 - frontrightTicks);
    }
    else if (frontrightTicks < -10000) {
      frontrightTicks = 2147483648-frontrightTicks;
    }
    else{}
    distanceRight_Front = frontrightTicks/TICKS_PER_METER;
  }
  lastCountLF = rightCountFront.data;
}

void Calc_Right_Middle(const std_msgs::Int32& rightCountMiddle) {
 
  static int lastCountLM = 0;
  if(rightCountMiddle.data != 0 && lastCountLM != 0) {
         
    int middlerightTicks = (rightCountMiddle.data - lastCountLM);
 
    if (middlerightTicks > 10000) {
      middlerightTicks = 0 - (2147483648 - middlerightTicks);
    }
    else if (middlerightTicks < -10000) {
      middlerightTicks = 2147483648-middlerightTicks;
    }
    else{}
    distanceRight_Middle = middlerightTicks/TICKS_PER_METER;
  }
  lastCountLM = rightCountMiddle.data;
}

void Calc_Right_Rear(const std_msgs::Int32& rightCountRear) {
 
  static int lastCountLR = 0;
  if(rightCountRear.data != 0 && lastCountLR != 0) {
         
    int rearrightTicks = (rightCountRear.data - lastCountLR);
 
    if (rearrightTicks > 10000) {
      rearrightTicks = 0 - (2147483648 - rearrightTicks);
    }
    else if (rearrightTicks < -10000) {
      rearrightTicks = 2147483648-rearrightTicks;
    }
    else{}
    distanceRight_Rear = rearrightTicks/TICKS_PER_METER;
  }
  lastCountLR = rightCountRear.data;
}


// Calculate the distance the right wheel has traveled since the last cycle
// void Calc_Right(const std_msgs::Int32& rightCount) {
   
//   static int lastCountR = 0;
//   if(rightCount.data != 0 && lastCountR != 0) {
 
//     int rightTicks = rightCount.data - lastCountR;
     
//     if (rightTicks > 10000) {
//       distanceRight = (0 - (2147483648 - distanceRight))/TICKS_PER_METER;
//     }
//     else if (rightTicks < -10000) {
//       rightTicks = 2147483648 - rightTicks;
//     }
//     else{}
//     distanceRight = rightTicks/TICKS_PER_METER;
//   }
//   lastCountR = rightCount.data;
// }
 
// Publish a nav_msgs::Odometry message in quaternion format
void publish_quat() {
 
  tf2::Quaternion q;
         
  q.setRPY(0, 0, odomNew.pose.pose.orientation.z);
 
  nav_msgs::Odometry quatOdom;
  quatOdom.header.stamp = odomNew.header.stamp;
  quatOdom.header.frame_id = "odom";
  quatOdom.child_frame_id = "base_link";
  quatOdom.pose.pose.position.x = odomNew.pose.pose.position.x;
  quatOdom.pose.pose.position.y = odomNew.pose.pose.position.y;
  quatOdom.pose.pose.position.z = odomNew.pose.pose.position.z;
  quatOdom.pose.pose.orientation.x = q.x();
  quatOdom.pose.pose.orientation.y = q.y();
  quatOdom.pose.pose.orientation.z = q.z();
  quatOdom.pose.pose.orientation.w = q.w();
  quatOdom.twist.twist.linear.x = odomNew.twist.twist.linear.x;
  quatOdom.twist.twist.linear.y = odomNew.twist.twist.linear.y;
  quatOdom.twist.twist.linear.z = odomNew.twist.twist.linear.z;
  quatOdom.twist.twist.angular.x = odomNew.twist.twist.angular.x;
  quatOdom.twist.twist.angular.y = odomNew.twist.twist.angular.y;
  quatOdom.twist.twist.angular.z = odomNew.twist.twist.angular.z;
 
  for(int i = 0; i<36; i++) {
    if(i == 0 || i == 7 || i == 14) {
      quatOdom.pose.covariance[i] = .01;
     }
     else if (i == 21 || i == 28 || i== 35) {
       quatOdom.pose.covariance[i] += 0.1;
     }
     else {
       quatOdom.pose.covariance[i] = 0;
     }
  }
 
  odom_data_pub_quat.publish(quatOdom);
}
 
// Update odometry information
void update_odom() {
 
  // Calculate the average distance
//   double cycleDistance = (distanceRight + distanceLeft) / 2;
     double cycleDistance = (-distanceRight_Front - distanceRight_Middle - distanceRight_Rear + distanceLeft_Front + distanceLeft_Middle + distanceLeft_Rear) / 6;
      double leftDistance = (distanceLeft_Front + distanceLeft_Middle + distanceLeft_Rear)/3 ;
      double rightDistance = -(distanceRight_Front + distanceRight_Middle + distanceRight_Rear)/3;

  // Calculate the number of radians the robot has turned since the last cycle
  // double cycleAngle = asin((distanceRight_Middle-distanceLeft_Middle)/WHEEL_BASE);
   double cycleAngle = asin((rightDistance-leftDistance)/WHEEL_BASE);

  // Average angle during the last cycle
  double avgAngle = cycleAngle/2 + odomOld.pose.pose.orientation.z;
     
  if (avgAngle > PI) {
    avgAngle -= 2*PI;
  }
  else if (avgAngle < -PI) {
    avgAngle += 2*PI;
  }
  else{}
 
  // Calculate the new pose (x, y, and theta)
  odomNew.pose.pose.position.x = odomOld.pose.pose.position.x + cos(avgAngle)*cycleDistance;
  odomNew.pose.pose.position.y = odomOld.pose.pose.position.y + sin(avgAngle)*cycleDistance;
  odomNew.pose.pose.orientation.z = cycleAngle + odomOld.pose.pose.orientation.z;
 
  // Prevent lockup from a single bad cycle
  if (isnan(odomNew.pose.pose.position.x) || isnan(odomNew.pose.pose.position.y)
     || isnan(odomNew.pose.pose.position.z)) {
    odomNew.pose.pose.position.x = odomOld.pose.pose.position.x;
    odomNew.pose.pose.position.y = odomOld.pose.pose.position.y;
    odomNew.pose.pose.orientation.z = odomOld.pose.pose.orientation.z;
  }
 
  // Make sure theta stays in the correct range
  if (odomNew.pose.pose.orientation.z > PI) {
    odomNew.pose.pose.orientation.z -= 2 * PI;
  }
  else if (odomNew.pose.pose.orientation.z < -PI) {
    odomNew.pose.pose.orientation.z += 2 * PI;
  }
  else{}
 
  // Compute the velocity
  odomNew.header.stamp = ros::Time::now();
  odomNew.twist.twist.linear.x = cycleDistance/(odomNew.header.stamp.toSec() - odomOld.header.stamp.toSec());
  odomNew.twist.twist.angular.z = cycleAngle/(odomNew.header.stamp.toSec() - odomOld.header.stamp.toSec());
 
  // Save the pose data for the next cycle
  odomOld.pose.pose.position.x = odomNew.pose.pose.position.x;
  odomOld.pose.pose.position.y = odomNew.pose.pose.position.y;
  odomOld.pose.pose.orientation.z = odomNew.pose.pose.orientation.z;
  odomOld.header.stamp = odomNew.header.stamp;
 
  // Publish the odometry message
  odom_data_pub.publish(odomNew);
}
 
int main(int argc, char **argv) {
   
  // Set the data fields of the odometry message
  odomNew.header.frame_id = "odom";
  odomNew.pose.pose.position.z = 0;
  odomNew.pose.pose.orientation.x = 0;
  odomNew.pose.pose.orientation.y = 0;
  odomNew.twist.twist.linear.x = 0;
  odomNew.twist.twist.linear.y = 0;
  odomNew.twist.twist.linear.z = 0;
  odomNew.twist.twist.angular.x = 0;
  odomNew.twist.twist.angular.y = 0;
  odomNew.twist.twist.angular.z = 0;
  odomOld.pose.pose.position.x = initialX;
  odomOld.pose.pose.position.y = initialY;
  odomOld.pose.pose.orientation.z = initialTheta;
 
  // Launch ROS and create a node
  ros::init(argc, argv, "wheel_odom_pub");
  ros::NodeHandle node;
 
  // Subscribe to ROS topics
//   ros::Subscriber subForRightCounts = node.subscribe("right_ticks", 100, Calc_Right, ros::TransportHints().tcpNoDelay());
//   ros::Subscriber subForLeftCounts = node.subscribe("left_ticks", 100, Calc_Left, ros::TransportHints().tcpNoDelay());
  ros::Subscriber subForLeftFrontCounts = node.subscribe("left_front_ticks", 100, Calc_Left_Front, ros::TransportHints().tcpNoDelay());
  ros::Subscriber subForLeftMiddleCounts = node.subscribe("left_middle_ticks", 100, Calc_Left_Middle, ros::TransportHints().tcpNoDelay());
  ros::Subscriber subForLeftRearCounts = node.subscribe("left_rear_ticks", 100, Calc_Left_Rear, ros::TransportHints().tcpNoDelay());

  ros::Subscriber subForRghtFrontCounts = node.subscribe("right_front_ticks", 100, Calc_Right_Front, ros::TransportHints().tcpNoDelay());
  ros::Subscriber subForRightMiddleCounts = node.subscribe("right_middle_ticks", 100, Calc_Right_Middle, ros::TransportHints().tcpNoDelay());
  ros::Subscriber subForRightRearCounts = node.subscribe("right_rear_ticks", 100, Calc_Right_Rear, ros::TransportHints().tcpNoDelay());

  ros::Subscriber subInitialPose = node.subscribe("initial_2d", 1, set_initial_2d);
 
  // Publisher of simple odom message where orientation.z is an euler angle
  odom_data_pub = node.advertise<nav_msgs::Odometry>("odom_data_euler", 100);
 
  // Publisher of full odom message where orientation is quaternion
  odom_data_pub_quat = node.advertise<nav_msgs::Odometry>("odom_data_quat", 100);
 
  ros::Rate loop_rate(30); 
     
  while(ros::ok()) {
     
    
      update_odom();
      publish_quat();
    
    ros::spinOnce();
    loop_rate.sleep();
  }
 
  return 0;
}