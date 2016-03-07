#include <string>
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <bhand_gui/bhand_message_simulation.h>

double spread_position;
double finger_1_position;
double finger_2_position;
double finger_3_position;

void chatterCallback(const bhand_gui::bhand_message_simulation::ConstPtr& msg)
{

  spread_position = msg->spread_position;
  spread_position = spread_position/100;
  finger_1_position = msg->finger_1_position;
  finger_1_position = finger_1_position/100;
  finger_2_position = msg->finger_2_position;
  finger_2_position = finger_2_position/100;
  finger_3_position = msg->finger_3_position;
  finger_3_position = finger_3_position/100;

}

int main(int argc, char** argv) 
{

  ros::init(argc, argv, "joint_position");

  ros::NodeHandle n;
  
  ros::Publisher joint_pub = n.advertise<sensor_msgs::JointState>("joint_states", 100);

  ros::Subscriber sub = n.subscribe("GUI_topic_simulation", 10, chatterCallback);

  ros::Rate loop_rate(60);

  sensor_msgs::JointState joint_state;

  joint_state.name.resize(8);
  joint_state.position.resize(8);
  joint_state.name[0] ="bh_j11_joint";
  joint_state.name[1] ="bh_j12_joint";
  joint_state.name[2] ="bh_j13_joint";
  joint_state.name[3] ="bh_j21_joint";
  joint_state.name[4] ="bh_j22_joint"; 
  joint_state.name[5] ="bh_j23_joint";
  joint_state.name[6] ="bh_j32_joint";
  joint_state.name[7] ="bh_j33_joint";

  while (ros::ok()) 
  {

    joint_state.header.stamp = ros::Time::now();
    joint_state.position[0] = spread_position;
    joint_state.position[1] = finger_1_position;
    joint_state.position[2] = finger_1_position*(0.84/2.44);
    joint_state.position[3] = spread_position;
    joint_state.position[4] = finger_2_position;
    joint_state.position[5] = finger_2_position*(0.84/2.44);
    joint_state.position[6] = finger_3_position;
    joint_state.position[7] = finger_3_position*(0.84/2.44);

    joint_pub.publish(joint_state);
   
    ros::spinOnce();

    loop_rate.sleep();
  }

return 0;

}
