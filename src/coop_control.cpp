#include "coop_control/coop_control.h"
#include <pluginlib/class_list_macros.h>
#include <QStringList>
#include <stdlib.h>
#include <thread>
#include <string.h>

namespace coop_control
{
coopPlugin::coopPlugin()
  : rqt_gui_cpp::Plugin()
  , widget_(0)
{
  // Constructor is called first before initPlugin function, needless to say.

  // give QObjects reasonable names
  setObjectName("coopPlugin");
}

//global variables
static QString qstr1;

void coopPlugin::on_enterButton_clicked() 
{
    qstr1 = ui_.lineEdit_2->text();

    ui_.lineEdit->setText(qstr1);
}

//set initial connection of gui and functions
void coopPlugin::connectionfunc()
{
    QObject::connect(ui_.pushButton_6, SIGNAL(clicked()),
            this, SLOT(on_enterButton_clicked())    );

    //"this" means source code, and in this case, it means "aidinPlugin".
}

void coopPlugin::initPlugin(qt_gui_cpp::PluginContext& context)
{
  // access standalone command line arguments
  QStringList argv = context.argv();
  // create QWidget
  widget_ = new QWidget();
  // extend the widget with all attributes and children from UI file
  ui_.setupUi(widget_);
  // add widget to the user interface
  context.addWidget(widget_);

  // add menu to the listview
  model = new QStringListModel(this); //dynamic memories allocates
  QStringList list; //
  list  << "eigentransmit eigentransmit.launch" 
        << "rqt_gui_test rqt_gui_test_publisher" 
        << "aidinvi walking_vis.launch"
        << "rviz rviz" //set the menu list
        << "plot: /gazebo/aidinvi_footpos"
        << "plot: /aidinvi_actp"
        << "plot: /gazebo/aidinvi_jointt"
        << "plot: /aidinvi_actt"
        << "plot: /aidinvi_contact"
        << "rqt_gui_test rqt_gui_test_subscriber"
        << "command: Foottrajectory on" //msg topic command
        << "command: Foottrajectory off"
        << "command: Footprint on"
        << "command: Footprint off"
        << "command: Cobtrajectory on"
        << "command: Cobtrajectory off";
        
  model->setStringList(list);
  ui_.listView->setModel(model);
  
  connectionfunc();
}

//additional functions must exist before shutdownPlugin function!!!!!!!!!!!!!
void coopPlugin::shutdownPlugin()
{
  // unregister all publishers here
}

void coopPlugin::saveSettings(qt_gui_cpp::Settings& plugin_settings,
    qt_gui_cpp::Settings& instance_settings) const
{
  // instance_settings.setValue(k, v)
}

void coopPlugin::restoreSettings(const qt_gui_cpp::Settings& plugin_settings,
    const qt_gui_cpp::Settings& instance_settings)
{
  // v = instance_settings.value(k)
}

/*bool hasConfiguration() const
{
  return true;
}

void triggerConfiguration()
{
  // Usually used to open a dialog to offer the user a set of configuration
}*/

}  // namespace coop_control

PLUGINLIB_EXPORT_CLASS(coop_control::coopPlugin, rqt_gui_cpp::Plugin)