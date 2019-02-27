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
double gap = 1;
coop_control::msgpid msg;

void coopPlugin::on_enterButton_clicked() 
{
    //qstr = ui_.lineEdit_2->text();
    kp = ui_.lineEdit_2->text().toDouble();

    //qstr = ui_.lineEdit_2->text();
    ki = ui_.lineEdit_3->text().toDouble();

    //qstr = ui_.lineEdit_2->text();
    kd = ui_.lineEdit_4->text().toDouble();

    qstr = QString("kp : %1     ki : %2     kd : %3").arg(kp).arg(ki).arg(kd);
    ui_.lineEdit->setText(qstr);

    //ui_.lineEdit->setText(qstr);
    msg.a = kp;
    msg.b = ki;
    msg.c = kd;
    gain_pub.publish(msg);    
}

void coopPlugin::on_kp_plus_clicked() 
{
    gap = ui_.gap->text().toDouble();
    kp = kp+gap;
    qstr = QString::number(kp);
    //ui_.lineEdit->setText(qstr);
    ui_.lineEdit_2->setText(qstr);

    qstr = QString("kp : %1     ki : %2     kd : %3").arg(kp).arg(ki).arg(kd);
    ui_.lineEdit->setText(qstr);

    msg.a = kp;
    msg.b = ki;
    msg.c = kd;
    gain_pub.publish(msg);    
}

void coopPlugin::on_kp_minus_clicked()
{
    gap = ui_.gap->text().toDouble();
    kp = kp-gap;
    qstr = QString::number(kp);
    //ui_.lineEdit->setText(qstr);
    ui_.lineEdit_2->setText(qstr);

    qstr = QString("kp : %1     ki : %2     kd : %3").arg(kp).arg(ki).arg(kd);
    ui_.lineEdit->setText(qstr);

    msg.a = kp;
    msg.b = ki;
    msg.c = kd;
    gain_pub.publish(msg);    
}

void coopPlugin::on_ki_plus_clicked() 
{
    gap = ui_.gap->text().toDouble();
    ki = ki+gap;
    qstr = QString::number(ki);
    //ui_.lineEdit->setText(qstr);
    ui_.lineEdit_3->setText(qstr);

    qstr = QString("kp : %1     ki : %2     kd : %3").arg(kp).arg(ki).arg(kd);
    ui_.lineEdit->setText(qstr);

    msg.a = kp;
    msg.b = ki;
    msg.c = kd;
    gain_pub.publish(msg);    
}

void coopPlugin::on_ki_minus_clicked()
{
    gap = ui_.gap->text().toDouble();
    ki = ki-gap;
    qstr = QString::number(ki);
    //ui_.lineEdit->setText(qstr);
    ui_.lineEdit_3->setText(qstr);

    qstr = QString("kp : %1     ki : %2     kd : %3").arg(kp).arg(ki).arg(kd);
    ui_.lineEdit->setText(qstr);

    msg.a = kp;
    msg.b = ki;
    msg.c = kd;
    gain_pub.publish(msg);    
}

void coopPlugin::on_kd_plus_clicked() 
{
    gap = ui_.gap->text().toDouble();
    kd = kd+gap;
    qstr = QString::number(kd);
    //ui_.lineEdit->setText(qstr);
    ui_.lineEdit_4->setText(qstr);

    qstr = QString("kp : %1     ki : %2     kd : %3").arg(kp).arg(ki).arg(kd);
    ui_.lineEdit->setText(qstr);

    msg.a = kp;
    msg.b = ki;
    msg.c = kd;
    gain_pub.publish(msg);    
}

void coopPlugin::on_kd_minus_clicked()
{
    gap = ui_.gap->text().toDouble();
    kd = kd-gap;
    qstr = QString::number(kd);
    //ui_.lineEdit->setText(qstr);
    ui_.lineEdit_4->setText(qstr);

    qstr = QString("kp : %1     ki : %2     kd : %3").arg(kp).arg(ki).arg(kd);
    ui_.lineEdit->setText(qstr);

    msg.a = kp;
    msg.b = ki;
    msg.c = kd;
    gain_pub.publish(msg);    
}

void coopPlugin::connectionfunc()
{
    QObject::connect(ui_.pushButton_6, SIGNAL(clicked()),
            this, SLOT(on_enterButton_clicked())    );
    QObject::connect(ui_.kp_plus, SIGNAL(clicked()),
            this, SLOT(on_kp_plus_clicked())    );
    QObject::connect(ui_.kp_minus, SIGNAL(clicked()),
            this, SLOT(on_kp_minus_clicked())    );
    QObject::connect(ui_.ki_plus, SIGNAL(clicked()),
            this, SLOT(on_ki_plus_clicked())    );
    QObject::connect(ui_.ki_minus, SIGNAL(clicked()),
            this, SLOT(on_ki_minus_clicked())    );
    QObject::connect(ui_.kd_plus, SIGNAL(clicked()),
            this, SLOT(on_kd_plus_clicked())    );
    QObject::connect(ui_.kd_minus, SIGNAL(clicked()),
            this, SLOT(on_kd_minus_clicked())    );
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