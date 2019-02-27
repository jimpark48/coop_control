#include "coop_control/coop_control.h"
#include <pluginlib/class_list_macros.h>
#include <QStringList>
#include <stdlib.h>
#include <string.h>
#include "coop_control/msgpid.h"

namespace coop_control
{
coopPlugin::coopPlugin()
    : rqt_gui_cpp::Plugin()
    , widget_(0)
{
    //give QObjects reasonable names
    setObjectName("coopPlugin");
}

static QString qstr;
static double kp, ki, kd;

void coopPlugin::on_enterButton_clicked() 
{
    //qstr = ui_.lineEdit_2->text();
    kp = ui_.lineEdit_2->text().toDouble();

    //qstr = ui_.lineEdit_2->text();
    ki = ui_.lineEdit_3->text().toDouble();

    //qstr = ui_.lineEdit_2->text();
    kd = ui_.lineEdit_4->text().toDouble();

    //ui_.lineEdit->setText(qstr);
    coop_control::msgpid msg;
    msg.a = kp;
    msg.b = ki;
    msg.c = kd;
    gain_pub.publish(msg);    
}

void coopPlugin::on_gainplus_clicked() 
{
    kp = kp+1;
    qstr = QString::number(kp);
    ui_.lineEdit->setText(qstr);
    ui_.lineEdit_2->setText(qstr);
}

void coopPlugin::on_gainminus_clicked()
{
    kp = kp-1;
    qstr = QString::number(kp);
    ui_.lineEdit->setText(qstr);
    ui_.lineEdit_2->setText(qstr);
}

void coopPlugin::connectionfunc()
{
    QObject::connect(ui_.pushButton_6, SIGNAL(clicked()),
            this, SLOT(on_enterButton_clicked())    );
    QObject::connect(ui_.pushButton_4, SIGNAL(clicked()),
            this, SLOT(on_gainplus_clicked())    );
    QObject::connect(ui_.pushButton_5, SIGNAL(clicked()),
            this, SLOT(on_gainminus_clicked())    );
}

void coopPlugin::initPlugin(qt_gui_cpp::PluginContext& context)
{
    //access standalone command line arguments
    QStringList argv = context.argv();
    //create QWidget
    widget_ = new QWidget();
    //extend the widget with all attributes and children from UI file
    ui_.setupUi(widget_);
    //add widget to the user interface
    context.addWidget(widget_);

    model = new QStringListModel(this); //dynamic memories allocates
    QStringList list;
    list << "undefined";

    model->setStringList(list);
    ui_.listView->setModel(model);

    gain_pub = nh.advertise<coop_control::msgpid>("pidgain", 3);

    connectionfunc();
}

void coopPlugin::shutdownPlugin()
{
    //unregister all publishers here
}

void coopPlugin::saveSettings(qt_gui_cpp::Settings& plugin_settings,
    qt_gui_cpp::Settings& instance_settings) const
{
    //instance_settings.setValue(k, v)
}

void coopPlugin::restoreSettings(const qt_gui_cpp::Settings& plugin_settings, 
    const qt_gui_cpp::Settings& instance_settings)
{
    //v = instance_settings.value(k)
}

/*bool hasConfiguration() const
{
    return true;
}

void triggerConfiguration()
{
    //Usually used to open a dialog to offer the user a set of configuration
}*/
} // namespace coop_control

PLUGINLIB_EXPORT_CLASS(coop_control::coopPlugin, rqt_gui_cpp::Plugin)