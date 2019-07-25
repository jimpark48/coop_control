#ifndef COOP_CONTROL_COOP_CONTROL_H
#define COOP_CONTROL_COOP_CONTROL_H

#include <rqt_gui_cpp/plugin.h>
#include <coop_control/ui_coop_control.h>
#include <QWidget>

//using QPushButton
#include <QPushButton>
#include <QApplication>

//using QListView
#include <QString>
#include <QStringList>
#include <QStringListModel>

//using QTextEdit
#include <QTextEdit>

#include "ros/ros.h"

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
  //comment in to signal that the plugin has a way to configure it
  //bool hasConfiguration() const;
  //void triggerConfiguration();

private:
  Ui::coopwidget ui_;
  QWidget* widget_;
  QStringListModel* model;

  ros::NodeHandle nh;
  ros::Publisher gain_pub;
  ros::Publisher target_pub;
  ros::Publisher reset_pub;
  ros::Publisher command_pub;

  Q_SIGNALS:
    void clicked();
    void clicked(const QModelIndex &index);

  private slots:
    void on_enterButton_clicked();
    void on_kp_plus_clicked();
    void on_kp_minus_clicked();
    void on_ki_plus_clicked();
    void on_ki_minus_clicked();
    void on_kd_plus_clicked();
    void on_kd_minus_clicked();
    void on_target_clicked();
    void on_reset_clicked();
    void on_activated_clicked();
    void on_activated_clicked2();
    void on_quit_clicked();
    void onClickListItem(const QModelIndex &index);
};
} //namespace coop_control
#endif //COOP_CONTROL_COOP_CONTROL_H