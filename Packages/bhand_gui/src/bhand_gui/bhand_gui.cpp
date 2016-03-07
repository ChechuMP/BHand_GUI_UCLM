#include <bhand_gui/bhand_gui.h>
#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>
#include <stdlib.h>
#include <iostream>
#include <bhand_gui/bhand_message_simulation.h>
#include <bhand_gui/bhand_message_position.h>
#include <bhand_gui/bhand_message_velocity.h>
#include <bhand_gui/bhand_message_feedback.h>
#include <bhand_gui/bhand_message_start_stop.h>
#include <QString>
#include <QTimer>

using namespace std;

ros::NodeHandle nh;

ros::Publisher pub_sim = nh.advertise<bhand_gui::bhand_message_simulation>("GUI_topic_simulation", 1);

ros::Publisher pub_pos = nh.advertise<bhand_gui::bhand_message_position>("GUI_topic_position", 1);

ros::Publisher pub_vel = nh.advertise<bhand_gui::bhand_message_velocity>("GUI_topic_velocity", 1);

ros::Publisher pub_start_stop = nh.advertise<bhand_gui::bhand_message_start_stop>("GUI_topic_start_stop", 1);

bhand_gui::bhand_message_simulation msg_sim;

bhand_gui::bhand_message_position msg_pos;

bhand_gui::bhand_message_velocity msg_vel;

bhand_gui::bhand_message_start_stop msg_start_stop;

int last_finger_1_position = 0; 
int last_finger_2_position = 0; 
int last_finger_3_position = 0; 
int last_spread_position = 0;

int last_finger_1_velocity = 20; 
int last_finger_2_velocity = 20; 
int last_finger_3_velocity = 20; 
int last_spread_velocity = 335;

double real_finger_1_position = 0;
double real_finger_2_position = 0;
double real_finger_3_position = 0;
double real_spread_position = 0;

double real_finger_1_velocity = 0.20;
double real_finger_2_velocity = 0.20;
double real_finger_3_velocity = 0.20;
double real_spread_velocity = 3.35;

int pos;
int vel;
string finger;
QString finger_command;
QString position_command;
QString velocity_command;

void chatterCallback(const bhand_gui::bhand_message_feedback::ConstPtr& msg)
{

  double number_1 = 0.0000122;

  double number_2 = 0.0000872;

  double number_3 = 0.012;

  double number_4 = 0.2092;

  real_finger_1_position = msg->bh_finger_1_position*number_1;

  real_finger_2_position = msg->bh_finger_2_position*number_1;

  real_finger_3_position = msg->bh_finger_3_position*number_1;

  real_spread_position = msg->bh_spread_position*number_2;

  real_finger_1_velocity = msg->bh_finger_1_velocity*number_3;

  real_finger_2_velocity = msg->bh_finger_2_velocity*number_3;

  real_finger_3_velocity = msg->bh_finger_3_velocity*number_3;

  real_spread_velocity = msg->bh_spread_velocity*number_4;

}

ros::Subscriber sub = nh.subscribe("GUI_topic_feedback", 1, chatterCallback);

namespace bhand_gui {

BHAND::BHAND()
  : rqt_gui_cpp::Plugin()
  , widget_(0)
{
  setObjectName("BHAND");
}

void BHAND::initPlugin(qt_gui_cpp::PluginContext& context)
{

  widget_ = new QWidget();
  
  ui_.setupUi(widget_);
  
  context.addWidget(widget_);

  ui_.stopHand->setEnabled(FALSE);

  QTimer *timer = new QTimer(this);

  timer->start(10); 

  QObject::connect(timer,SIGNAL(timeout()),this,SLOT(RosSpin()));

  QObject::connect(ui_.finger_1_position_slider,SIGNAL(valueChanged(int)),this,SLOT(valueChangeSimulation(int)));

  QObject::connect(ui_.finger_2_position_slider,SIGNAL(valueChanged(int)),this,SLOT(valueChangeSimulation(int)));
  
  QObject::connect(ui_.finger_3_position_slider,SIGNAL(valueChanged(int)),this,SLOT(valueChangeSimulation(int)));

  QObject::connect(ui_.spread_position_slider,SIGNAL(valueChanged(int)),this,SLOT(valueChangeSimulation(int)));

  QObject::connect(ui_.finger_1_velocity_slider,SIGNAL(valueChanged(int)),this,SLOT(valueChangeSimulation(int)));

  QObject::connect(ui_.finger_2_velocity_slider,SIGNAL(valueChanged(int)),this,SLOT(valueChangeSimulation(int)));
  
  QObject::connect(ui_.finger_3_velocity_slider,SIGNAL(valueChanged(int)),this,SLOT(valueChangeSimulation(int)));

  QObject::connect(ui_.spread_velocity_slider,SIGNAL(valueChanged(int)),this,SLOT(valueChangeSimulation(int)));

  QObject::connect(ui_.finger_1_position_slider,SIGNAL(sliderReleased()),this,SLOT(valueChange()));

  QObject::connect(ui_.finger_2_position_slider,SIGNAL(sliderReleased()),this,SLOT(valueChange()));
  
  QObject::connect(ui_.finger_3_position_slider,SIGNAL(sliderReleased()),this,SLOT(valueChange()));

  QObject::connect(ui_.spread_position_slider,SIGNAL(sliderReleased()),this,SLOT(valueChange()));

  QObject::connect(ui_.finger_1_velocity_slider,SIGNAL(sliderReleased()),this,SLOT(valueChange()));

  QObject::connect(ui_.finger_2_velocity_slider,SIGNAL(sliderReleased()),this,SLOT(valueChange()));
 
  QObject::connect(ui_.finger_3_velocity_slider,SIGNAL(sliderReleased()),this,SLOT(valueChange()));
 
  QObject::connect(ui_.spread_velocity_slider,SIGNAL(sliderReleased()),this,SLOT(valueChange()));

  QObject::connect(ui_.startHand,SIGNAL(pressed()),this,SLOT(init_stop_Hand()));

  QObject::connect(ui_.stopHand,SIGNAL(pressed()),this,SLOT(init_stop_Hand()));

}

void BHAND::ControlBarrett()
{

  if(last_finger_1_position != ui_.finger_1_position_slider->value())
  {

    double number_1 = ui_.finger_1_position_slider->value();

    double number_2 = 195000/244;

    pos = number_1*number_2;

    if(ui_.move_all_fingers->isChecked() == TRUE)
    {

      finger = "123";

      finger_command = "fingers"; 

    }
    else
    {

      finger = "1";

      finger_command = "finger 1";

    }

    position_command = QString::number(pos);

    msg_pos.finger = finger;

    msg_pos.position = pos;

    last_finger_1_position = ui_.finger_1_position_slider->value();

  }

  if(last_finger_2_position != ui_.finger_2_position_slider->value())
  {

    double number_1 = ui_.finger_2_position_slider->value();

    double number_2 = 195000/244;

    pos = number_1*number_2;

    if(ui_.move_all_fingers->isChecked() == TRUE)
    {

      finger = "123";

      finger_command = "fingers";

    }
    else 
    {

      finger = "2";

      finger_command = "finger 2";

    }

    position_command = QString::number(pos);

    msg_pos.finger = finger;

    msg_pos.position = pos;

    last_finger_2_position = ui_.finger_2_position_slider->value();

  }

  if(last_finger_3_position != ui_.finger_3_position_slider->value())
  {
      
    double number_1 = ui_.finger_3_position_slider->value();

    double number_2 = 195000/244;

    pos = number_1*number_2;

    if(ui_.move_all_fingers->isChecked() == TRUE) 
    {

      finger = "123";

      finger_command = "fingers"; 

    }
    else
    {

      finger = "3";

      finger_command = "finger 3";

    }

    position_command = QString::number(pos);

    msg_pos.finger = finger;

    msg_pos.position = pos;

    last_finger_3_position = ui_.finger_3_position_slider->value();

  }

  if(last_spread_position != ui_.spread_position_slider->value())
  {

    double number_1 = ui_.spread_position_slider->value();

    double number_2 = 35950/314;

    pos = number_1*number_2;

    finger = "S";

    finger_command = "spread";

    position_command = QString::number(pos);

    msg_pos.finger = finger;

    msg_pos.position = pos;

    last_spread_position = ui_.spread_position_slider->value();

  }
  
  CommandHistory();

}

void BHAND::ControlBarrettVelocity()
{

  if(last_finger_1_velocity != ui_.finger_1_velocity_slider->value())
  {

    double number_1 = ui_.finger_1_velocity_slider->value();

    double number_2 = 0.82; 

    vel = number_1*number_2;

    if(ui_.change_velocity_all_fingers->isChecked() == TRUE) 
    {

      finger = "123";

      finger_command = "fingers"; 

    }
    else
    {

      finger = "1";

      finger_command = "finger 1";

    }

    velocity_command = QString::number(vel);

    msg_vel.finger = finger;

    msg_vel.velocity = vel;

    last_finger_1_velocity = ui_.finger_1_velocity_slider->value();

  }

  if(last_finger_2_velocity != ui_.finger_2_velocity_slider->value())
  {

    double number_1 = ui_.finger_2_velocity_slider->value();

    double number_2 = 0.82; 

    vel = number_1*number_2;

    if(ui_.change_velocity_all_fingers->isChecked() == TRUE) 
    {

      finger = "123";

      finger_command = "fingers"; 

    }
    else
    {
 
      finger = "2";

      finger_command = "finger 2";

    }

    velocity_command = QString::number(vel);

    msg_vel.finger = finger;

    msg_vel.velocity = vel;

    last_finger_2_velocity = ui_.finger_2_velocity_slider->value();

  }

  if(last_finger_3_velocity != ui_.finger_3_velocity_slider->value())
  {
      
    double number_1 = ui_.finger_3_velocity_slider->value();

    double number_2 = 0.82;

    vel = number_1*number_2;

    if(ui_.change_velocity_all_fingers->isChecked() == TRUE)
    {

      finger = "123";

      finger_command = "fingers"; 

    }
    else 
    {

      finger = "3";

      finger_command = "finger 3";

    }

    velocity_command = QString::number(vel);

    msg_vel.finger = finger;

    msg_vel.velocity = vel;

    last_finger_3_velocity = ui_.finger_3_velocity_slider->value();

  }

  if(last_spread_velocity != ui_.spread_velocity_slider->value())
  {

    double number_1 = ui_.spread_velocity_slider->value();

    double number_2 = 0.0478; 

    vel = number_1*number_2;

    finger = "S";

    finger_command = "spread";

    velocity_command = QString::number(vel);

    msg_vel.finger = finger;

    msg_vel.velocity = vel;

    last_spread_velocity = ui_.spread_velocity_slider->value(); 

  }
  
  CommandHistory();

}

void BHAND::ControlBarrettSimulation()
{

  if(ui_.tabWidgetControlMode->currentIndex() == 0) 
  { 

    if(last_finger_1_position != ui_.finger_1_position_slider->value())
    {

      if(ui_.move_all_fingers->isChecked() == TRUE)
      {
  
        ui_.finger_2_position_slider->setValue(ui_.finger_1_position_slider->value()); 

        ui_.finger_3_position_slider->setValue(ui_.finger_1_position_slider->value()); 

        last_finger_1_position = ui_.finger_1_position_slider->value();

      }

    }

    else if(last_finger_2_position != ui_.finger_2_position_slider->value())
    {

      if(ui_.move_all_fingers->isChecked() == TRUE)
      {
  
        ui_.finger_1_position_slider->setValue(ui_.finger_2_position_slider->value()); 

        ui_.finger_3_position_slider->setValue(ui_.finger_2_position_slider->value()); 

        last_finger_2_position = ui_.finger_2_position_slider->value();

      }

    }

    else if(last_finger_3_position != ui_.finger_3_position_slider->value())
    {
      
      if(ui_.move_all_fingers->isChecked() == TRUE)
      {
  
        ui_.finger_1_position_slider->setValue(ui_.finger_3_position_slider->value()); 

        ui_.finger_2_position_slider->setValue(ui_.finger_3_position_slider->value()); 

        last_finger_3_position = ui_.finger_3_position_slider->value();

      }

    }

    msg_sim.finger_1_position = ui_.finger_1_position_slider->value();
    
    msg_sim.finger_2_position = ui_.finger_2_position_slider->value();

    msg_sim.finger_3_position = ui_.finger_3_position_slider->value();

    msg_sim.spread_position = ui_.spread_position_slider->value();

    pub_sim.publish(msg_sim);
   
  }
  else if(ui_.tabWidgetControlMode->currentIndex() == 1)
  {

    if(last_finger_1_velocity != ui_.finger_1_velocity_slider->value())
    {

      if(ui_.change_velocity_all_fingers->isChecked() == TRUE)
      {
  
        ui_.finger_2_velocity_slider->setValue(ui_.finger_1_velocity_slider->value()); 

        ui_.finger_3_velocity_slider->setValue(ui_.finger_1_velocity_slider->value()); 

        last_finger_1_velocity = ui_.finger_1_velocity_slider->value();

      }

    }

    else if(last_finger_2_velocity != ui_.finger_2_velocity_slider->value())
    {

      if(ui_.change_velocity_all_fingers->isChecked() == TRUE)
      {
  
        ui_.finger_1_velocity_slider->setValue(ui_.finger_2_velocity_slider->value()); 

        ui_.finger_3_velocity_slider->setValue(ui_.finger_2_velocity_slider->value()); 

        last_finger_2_velocity = ui_.finger_2_velocity_slider->value();

      }

    }

    else if(last_finger_3_velocity != ui_.finger_3_velocity_slider->value())
    {
      
      if(ui_.change_velocity_all_fingers->isChecked() == TRUE)
      {
  
        ui_.finger_1_velocity_slider->setValue(ui_.finger_3_velocity_slider->value()); 

        ui_.finger_2_velocity_slider->setValue(ui_.finger_3_velocity_slider->value()); 

        last_finger_3_velocity = ui_.finger_3_velocity_slider->value();

      }

    }

  }

}

void BHAND::CommandHistory()
{


  if(ui_.tabWidgetControlMode->currentIndex() == 0) 
  { 

    QString history = ui_.text_command_history->toPlainText(); 

    QString command = "position " + finger_command + " = " + position_command + "\n";

    ui_.text_command_history->setPlainText(history + command);

    ui_.text_command_history->moveCursor(QTextCursor::End);

    pub_pos.publish(msg_pos);
   
  }
  else if(ui_.tabWidgetControlMode->currentIndex() == 1) 
  {

    QString history = ui_.text_command_history->toPlainText(); 

    QString command = "velocity " + finger_command + " = " + velocity_command + "\n"; 

    ui_.text_command_history->setPlainText(history + command);

    ui_.text_command_history->moveCursor(QTextCursor::End);

    pub_vel.publish(msg_vel);

  }

}

void BHAND::valueChange()
{

  if(ui_.tabWidgetControlMode->currentIndex() == 0)
  { 

    ControlBarrett();
   
  }
  else if(ui_.tabWidgetControlMode->currentIndex() == 1)
  {

    ControlBarrettVelocity();

  }

}

void BHAND::valueChangeSimulation(int)
{

  if(ui_.tabWidgetControlMode->currentIndex() == 0) 
  { 

    double c = 100;

    double position = ui_.finger_1_position_slider->value();

    QString s_pos = QString::number(position/c,'f',2); 
    
    ui_.qlineedit_finger_1_position->setText(s_pos);

    position = ui_.finger_2_position_slider->value();

    s_pos = QString::number(position/c,'f',2);

    ui_.qlineedit_finger_2_position->setText(s_pos);

    position = ui_.finger_3_position_slider->value();

    s_pos = QString::number(position/c,'f',2);
 
    ui_.qlineedit_finger_3_position->setText(s_pos);

    position = ui_.spread_position_slider->value();

    s_pos = QString::number(position/c,'f',2);
   
    ui_.qlineedit_spread_position->setText(s_pos);

    ControlBarrettSimulation();
   
  }
  else if(ui_.tabWidgetControlMode->currentIndex() == 1)
  {

    double c = 100;

    double velocity = ui_.finger_1_velocity_slider->value();

    QString s_vel = QString::number(velocity/c,'f',2);
    
    ui_.qlineedit_finger_1_velocity->setText(s_vel);

    velocity = ui_.finger_2_velocity_slider->value();

    s_vel = QString::number(velocity/c,'f',2);

    ui_.qlineedit_finger_2_velocity->setText(s_vel);

    velocity = ui_.finger_3_velocity_slider->value();

    s_vel = QString::number(velocity/c,'f',2);
 
    ui_.qlineedit_finger_3_velocity->setText(s_vel);

    velocity = ui_.spread_velocity_slider->value();

    s_vel = QString::number(velocity/c,'f',2);
   
    ui_.qlineedit_spread_velocity->setText(s_vel);

    ControlBarrettSimulation();

  }

}

void BHAND::RosSpin()
{  
  
  ros::spinOnce();

  QString a = QString::number(real_finger_1_position,'f',2);

  ui_.qlineedit_finger_1_position_real->setText(a);

  a = QString::number(real_finger_2_position,'f',2);

  ui_.qlineedit_finger_2_position_real->setText(a);

  a = QString::number(real_finger_3_position,'f',2);

  ui_.qlineedit_finger_3_position_real->setText(a);

  a = QString::number(real_spread_position,'f',2);

  ui_.qlineedit_spread_position_real->setText(a);

  a = QString::number(real_finger_1_velocity,'f',2);

  ui_.qlineedit_finger_1_velocity_real->setText(a);

  a = QString::number(real_finger_2_velocity,'f',2);

  ui_.qlineedit_finger_2_velocity_real->setText(a);

  a = QString::number(real_finger_3_velocity,'f',2);

  ui_.qlineedit_finger_3_velocity_real->setText(a);

  a = QString::number(real_spread_velocity,'f',2);

  ui_.qlineedit_spread_velocity_real->setText(a);

}

void BHAND::init_stop_Hand()
{
  if(msg_start_stop.order == "")
  {

    msg_start_stop.order = "Start";

    pub_start_stop.publish(msg_start_stop);

    ui_.startHand->setEnabled(FALSE);

    ui_.stopHand->setEnabled(TRUE);

    ui_.finger_1_position_slider->setValue(0);

    ui_.finger_2_position_slider->setValue(0);

    ui_.finger_3_position_slider->setValue(0);

    ui_.spread_position_slider->setValue(0);

    ui_.finger_1_velocity_slider->setValue(20);

    ui_.finger_2_velocity_slider->setValue(20);

    ui_.finger_3_velocity_slider->setValue(20);

    ui_.spread_velocity_slider->setValue(335);

    ui_.text_command_history->setText("");

    QString history = ui_.text_command_history->toPlainText();

    QString command = "Starting BHand...\n";

    ui_.text_command_history->setPlainText(history + command);

    ui_.text_command_history->moveCursor(QTextCursor::End);

  }
  else if(msg_start_stop.order == "Start")
  {

    msg_start_stop.order = "Stop";
 
    pub_start_stop.publish(msg_start_stop);

    QString history = ui_.text_command_history->toPlainText();

    QString command = "Stopping BHand...\n";

    ui_.text_command_history->setPlainText(history + command);

    ui_.text_command_history->moveCursor(QTextCursor::End);

    ui_.startHand->setEnabled(TRUE);

    ui_.stopHand->setEnabled(FALSE);
   
    msg_start_stop.order = "";
 
  }
}

}


PLUGINLIB_DECLARE_CLASS(bhand_gui, BHAND, bhand_gui::BHAND, rqt_gui_cpp::Plugin)
