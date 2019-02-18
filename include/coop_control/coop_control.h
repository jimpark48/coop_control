#ifndef coop_control_coop_control_H
#define coop_control_coop_control_H

#include <rqt_gui_cpp/plugin.h>
#include <coop_control/ui_coop_control.h>
#include <QWidget>
//using QPushButton
#include <QPushButton>
#include <QApplication>

#include <ros/ros.h>
#include <std_msgs/Int32.h>
//using QListView 
#include <QString>
#include <QStringList>
#include <QStringListModel>

namespace coop_control
{

class coopPlugin
  : public rqt_gui_cpp::Plugin
{
  Q_OBJECT
public:
  coopPlugin();

  virtual void initPlugin(qt_gui_cpp::PluginContext& context);
  virtual void shutdownPlugin();
  virtual void saveSettings(qt_gui_cpp::Settings& plugin_settings,
      qt_gui_cpp::Settings& instance_settings) const;
  virtual void restoreSettings(const qt_gui_cpp::Settings& plugin_settings,
      const qt_gui_cpp::Settings& instance_settings);

  virtual void connectionfunc();

  // Comment in to signal that the plugin has a way to configure it
  // bool hasConfiguration() const;
  // void triggerConfiguration();

private:
  Ui::MyPluginWidget ui_;
  QWidget* widget_;
  QStringListModel* model;

  Q_SIGNALS:
    void clicked();

private slots:
  void on_enterButton_clicked(); //반드시 slots 안에 정의할 것

};
}  // namespace coop_control
#endif  // RQT_AIDIN_coop_control_H
