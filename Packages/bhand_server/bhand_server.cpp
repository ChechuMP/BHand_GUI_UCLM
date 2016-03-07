#define LINUX
#include <ros/ros.h>
#include <stdio.h>
#include <stdlib.h>
#include <iostream>
#include <string>
#include <sstream>
#include <BHand.h>
#include <BHandAppHelper.h>
#include <bhand_gui/bhand_message_position.h>
#include <bhand_gui/bhand_message_velocity.h>
#include <bhand_gui/bhand_message_feedback.h>
#include <bhand_gui/bhand_message_start_stop.h>

using namespace std;

BHand bh;

int finger_1_position;
int finger_2_position;
int finger_3_position;
int spread_position;

int real_finger_1_position;
int real_finger_2_position;
int real_finger_3_position;
int real_spread_position;

int finger_1_velocity;
int finger_2_velocity;
int finger_3_velocity;
int spread_velocity;

int real_finger_1_velocity;
int real_finger_2_velocity;
int real_finger_3_velocity;
int real_spread_velocity;

string check_1, check_2, check_3;

bhand_gui::bhand_message_feedback msg_fb;

void callbackPosition(const bhand_gui::bhand_message_position::ConstPtr& msg)
{

    bh.GoToPosition(msg->finger.c_str(),msg->position);

    check_1 = "Ok";
  
}

void callbackVelocity(const bhand_gui::bhand_message_velocity::ConstPtr& msg)
{

  bh.Set(msg->finger.c_str(),"MOV",msg->velocity);
  bh.Set(msg->finger.c_str(),"MCV",msg->velocity);

  check_2 = "Ok";
  
}

void callbackInit(const bhand_gui::bhand_message_start_stop::ConstPtr& msg)
{

  if(msg->order == "Start")
  {

    int hwIndex = BHandHardware::getBHandHardwareIndex("BH8-280");

    bh.setHardwareDesc(hwIndex);

    handInitWithMenu(&bh);

    bh.InitHand("");

    bh.Set("1","MOV",16);
    bh.Set("1","MCV",16);
    bh.Set("2","MOV",16);
    bh.Set("2","MCV",16);
    bh.Set("3","MOV",16);
    bh.Set("3","MCV",16);
    bh.Set("S","MOV",16);
    bh.Set("S","MCV",16);
 
    check_3 = "Ok";
 
  }

  else if(msg->order == "Stop")
  {

    bh.StopMotor("");
 
    check_3 = "Ok";

  }
  
}

void FeedBack()
{

  bh.Get("1","P",&real_finger_1_position);
  bh.Get("2","P",&real_finger_2_position);
  bh.Get("3","P",&real_finger_3_position);
  bh.Get("S","P",&real_spread_position);

  cout << real_spread_position << endl;

  msg_fb.bh_finger_1_position = real_finger_1_position;
  msg_fb.bh_finger_2_position = real_finger_2_position;
  msg_fb.bh_finger_3_position = real_finger_3_position;
  msg_fb.bh_spread_position = real_spread_position;

  bh.Get("1","MOV",&real_finger_1_velocity);
  bh.Get("2","MOV",&real_finger_2_velocity);
  bh.Get("3","MOV",&real_finger_3_velocity);
  bh.Get("S","MOV",&real_spread_velocity);

  msg_fb.bh_finger_1_velocity = real_finger_1_velocity;
  msg_fb.bh_finger_2_velocity = real_finger_2_velocity;
  msg_fb.bh_finger_3_velocity = real_finger_3_velocity;
  msg_fb.bh_spread_velocity = real_spread_velocity;

}

int main(int argc, char **argv)
{
 
  ros::init(argc, argv, "bhand_server");

  ros::NodeHandle n;

  ros::Publisher pub = n.advertise<bhand_gui::bhand_message_feedback>("GUI_topic_feedback", 1);

  ros::Subscriber sub_pos = n.subscribe("GUI_topic_position", 5, callbackPosition);

  ros::Subscriber sub_vel = n.subscribe("GUI_topic_velocity", 5, callbackVelocity);

  ros::Subscriber sub_init = n.subscribe("GUI_topic_start_stop", 1, callbackInit);

  while (ros::ok())
  {

    if(check_1 == "Ok" || check_2 == "Ok" || check_3 == "Ok")
    {

      FeedBack();

      pub.publish(msg_fb);

      check_1 = "";

      check_2 = "";

      check_3 = "";

    }

    ros::spinOnce();

  }

  return 0;
}
