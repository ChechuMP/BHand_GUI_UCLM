#ifndef bhand_gui__bhand_gui_H
#define bhand_gui__bhand_gui_H

#include <rqt_gui_cpp/plugin.h>
#include <ui_bhand_gui.h>
#include <nodelet/nodelet.h>
#include <ros/ros.h>
#include <QWidget>
#include <bhand_gui/bhand_message_feedback.h>

namespace bhand_gui {

class BHAND
  : public rqt_gui_cpp::Plugin
{
  Q_OBJECT

public:

  BHAND();

  virtual void initPlugin(qt_gui_cpp::PluginContext& context);

  virtual void ControlBarrett();

  virtual void ControlBarrettVelocity();

  virtual void ControlBarrettSimulation();

  virtual void CommandHistory();

  QWidget* widget_;

  Ui::MyGuiWidget ui_; 

private slots:

  virtual void valueChange();

  virtual void valueChangeSimulation(int);

  virtual void RosSpin();

  virtual void init_stop_Hand();
 
};

}


#endif
