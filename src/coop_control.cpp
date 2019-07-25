#include "coop_control/coop_control.h"
#include <pluginlib/class_list_macros.h>
#include <QStringList>
#include <stdlib.h>
#include <string.h>
#include "std_msgs/Float64.h"
#include "std_msgs/Float64MultiArray.h"
//#include "coop_control/msgpid.h"
//#include "coop_control/one.h"
#include "std_msgs/Int32.h"
#include "std_msgs/Int32MultiArray.h"

using namespace std;

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
static QString qstrlist;
static double kp, ki, kd, target;
double gapkp, gapki, gapkd;
std_msgs::Float64MultiArray msg;
std_msgs::Float64 msgtarget;
std_msgs::Int32 msgreset;
//[0] is feedforward, [1] is PID, [2] is gravity, [3] is friction
std_msgs::Int32MultiArray msgcommand;

char *str;

int command_reset = 0;

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
    msg.data[0] = kp;
    msg.data[1] = ki;
    msg.data[2] = kd;
    gain_pub.publish(msg);    
}

void coopPlugin::on_kp_plus_clicked() 
{
    kp = ui_.lineEdit_2->text().toDouble();
    ki = ui_.lineEdit_3->text().toDouble();
    kd = ui_.lineEdit_4->text().toDouble();

    gapkp = ui_.gap_kp->text().toDouble();
    kp = kp+gapkp;
    qstr = QString::number(kp);
    //ui_.lineEdit->setText(qstr);
    ui_.lineEdit_2->setText(qstr);

    qstr = QString("kp : %1     ki : %2     kd : %3").arg(kp).arg(ki).arg(kd);
    ui_.lineEdit->setText(qstr);

    msg.data[0] = kp;
    msg.data[1] = ki;
    msg.data[2] = kd;
    gain_pub.publish(msg);    
}

void coopPlugin::on_kp_minus_clicked()
{
    kp = ui_.lineEdit_2->text().toDouble();
    ki = ui_.lineEdit_3->text().toDouble();
    kd = ui_.lineEdit_4->text().toDouble();

    gapkp = ui_.gap_kp->text().toDouble();
    kp = kp-gapkp;
    qstr = QString::number(kp);
    //ui_.lineEdit->setText(qstr);
    ui_.lineEdit_2->setText(qstr);

    qstr = QString("kp : %1     ki : %2     kd : %3").arg(kp).arg(ki).arg(kd);
    ui_.lineEdit->setText(qstr);

    msg.data[0] = kp;
    msg.data[1] = ki;
    msg.data[2] = kd;
    gain_pub.publish(msg);    
}

void coopPlugin::on_ki_plus_clicked() 
{
    kp = ui_.lineEdit_2->text().toDouble();
    ki = ui_.lineEdit_3->text().toDouble();
    kd = ui_.lineEdit_4->text().toDouble();

    gapki = ui_.gap_ki->text().toDouble();
    ki = ki+gapki;
    qstr = QString::number(ki);
    //ui_.lineEdit->setText(qstr);
    ui_.lineEdit_3->setText(qstr);

    qstr = QString("kp : %1     ki : %2     kd : %3").arg(kp).arg(ki).arg(kd);
    ui_.lineEdit->setText(qstr);

    msg.data[0] = kp;
    msg.data[1] = ki;
    msg.data[2] = kd;
    gain_pub.publish(msg);    
}

void coopPlugin::on_ki_minus_clicked()
{
    kp = ui_.lineEdit_2->text().toDouble();
    ki = ui_.lineEdit_3->text().toDouble();
    kd = ui_.lineEdit_4->text().toDouble();

    gapki = ui_.gap_ki->text().toDouble();
    ki = ki-gapki;
    qstr = QString::number(ki);
    //ui_.lineEdit->setText(qstr);
    ui_.lineEdit_3->setText(qstr);

    qstr = QString("kp : %1     ki : %2     kd : %3").arg(kp).arg(ki).arg(kd);
    ui_.lineEdit->setText(qstr);

    msg.data[0] = kp;
    msg.data[1] = ki;
    msg.data[2] = kd;
    gain_pub.publish(msg);    
}

void coopPlugin::on_kd_plus_clicked() 
{
    kp = ui_.lineEdit_2->text().toDouble();
    ki = ui_.lineEdit_3->text().toDouble();
    kd = ui_.lineEdit_4->text().toDouble();

    gapkd = ui_.gap_kd->text().toDouble();
    kd = kd+gapkd;
    qstr = QString::number(kd);
    //ui_.lineEdit->setText(qstr);
    ui_.lineEdit_4->setText(qstr);

    qstr = QString("kp : %1     ki : %2     kd : %3").arg(kp).arg(ki).arg(kd);
    ui_.lineEdit->setText(qstr);

    msg.data[0] = kp;
    msg.data[1] = ki;
    msg.data[2] = kd;
    gain_pub.publish(msg);    
}

void coopPlugin::on_kd_minus_clicked()
{
    kp = ui_.lineEdit_2->text().toDouble();
    ki = ui_.lineEdit_3->text().toDouble();
    kd = ui_.lineEdit_4->text().toDouble();

    gapkd = ui_.gap_kd->text().toDouble();
    kd = kd-gapkd;
    qstr = QString::number(kd);
    //ui_.lineEdit->setText(qstr);
    ui_.lineEdit_4->setText(qstr);

    qstr = QString("kp : %1     ki : %2     kd : %3").arg(kp).arg(ki).arg(kd);
    ui_.lineEdit->setText(qstr);

    msg.data[0] = kp;
    msg.data[1] = ki;
    msg.data[2] = kd;
    gain_pub.publish(msg);    
}

void coopPlugin::on_target_clicked()
{
    target = ui_.Target->text().toDouble();
    msgtarget.data = target;
    target_pub.publish(msgtarget);
}

void coopPlugin::on_reset_clicked() 
{
    command_reset = 1;
    msgreset.data = command_reset;
    reset_pub.publish(msgreset);
}

char* string_to_char(std::string chstr) 
{
    int size_str = chstr.size();
    char* convert_char = (char*)malloc(size_str);
    int i = 0;
    for(i = 0; i < size_str; i++) {
        convert_char[i] = chstr[i];
    }
    convert_char[i] = '\0';
    return convert_char;
}

void coopPlugin::on_activated_clicked()
{
    QByteArray bytename = qstrlist.toLocal8Bit();
    str = bytename.data();    

    char *token = NULL;
    char s1[] = " ";
    token = strtok(str, s1);

    str = bytename.data();  

    int size_str;
    char *sys_message;
    
    if(strcmp(token, "rqt_plot") == 0) {
        string message1 = "gnome-terminal -e '";
        string message2 = token;
        string message3 = "'";
        message1 = message1 + message2 + message3;

        size_str = message1.size();
        sys_message = (char*)malloc(size_str);
        sys_message = string_to_char(message1);
        ROS_INFO("%s", sys_message);
        system(sys_message);
    }
    else {
        string message1 = "gnome-terminal -e 'rosrun ";
        string message2 = str;
        string message3 = "'";
        message1 = message1 + message2 + message3;

        size_str = message1.size();
        sys_message = (char*)malloc(size_str);
        sys_message = string_to_char(message1);
        ROS_INFO("%s", sys_message);
        system(sys_message);
    }
}

void coopPlugin::on_activated_clicked2()
{
    QByteArray bytename = qstrlist.toLocal8Bit();
    str = bytename.data();    
    char *token = NULL;
    char s1[] = " ";
    token = strtok(str, s1);
    str = strtok(NULL, s1);

    if(strcmp(str, "feedforward") == 0) {
        msgcommand.data[0] = 1;
        command_pub.publish(msgcommand);
    }
    else if(strcmp(str, "PID") == 0) {
        msgcommand.data[1] = 1;
        command_pub.publish(msgcommand);
    }
    else if(strcmp(str, "gravity") == 0) {
        msgcommand.data[2] = 1;
        command_pub.publish(msgcommand);
    }
    else if(strcmp(str, "friction") == 0) {
        msgcommand.data[3] = 1;
        command_pub.publish(msgcommand);
    }
}

void coopPlugin::on_quit_clicked()
{
    QByteArray bytename = qstrlist.toLocal8Bit();
    str = bytename.data();    
    char *token = NULL;
    char s1[] = " ";
    token = strtok(str, s1);
    str = strtok(NULL, s1);

    if(strcmp(str, "feedforward") == 0) {
        msgcommand.data[0] = 0;
        command_pub.publish(msgcommand);
    }
    else if(strcmp(str, "PID") == 0) {
        msgcommand.data[1] = 0;
        command_pub.publish(msgcommand);
    }
    else if(strcmp(str, "gravity") == 0) {
        msgcommand.data[2] = 0;
        command_pub.publish(msgcommand);
    }
    else if(strcmp(str, "friction") == 0) {
        msgcommand.data[3] = 0;
        command_pub.publish(msgcommand);
    }
}

void coopPlugin::onClickListItem(const QModelIndex &index)
{
    QObject::disconnect(ui_.pushButton, SIGNAL(clicked()),
                this, SLOT(on_activated_clicked())      );
    QObject::disconnect(ui_.quitbutton, SIGNAL(clicked()),
                this, SLOT(on_quit_clicked())       );
    QObject::disconnect(ui_.pushButton, SIGNAL(clicked()),
                this, SLOT(on_activated_clicked2())     );

    qstrlist = QString("%1").arg(index.data().toString());

    QByteArray bytename = qstrlist.toLocal8Bit();
    str = bytename.data();    

    char *token = NULL;
    char s1[] = " ";
    token = strtok(str, s1);

    if(strcmp(token, "command:") == 0) {
        QObject::connect(ui_.pushButton, SIGNAL(clicked()),
                this, SLOT(on_activated_clicked2())     );
        QObject::connect(ui_.quitbutton, SIGNAL(clicked()),
            this, SLOT(on_quit_clicked())       );

        str = strtok(NULL, s1); 
    }
    else {
        QObject::connect(ui_.pushButton, SIGNAL(clicked()),
            this, SLOT(on_activated_clicked())      );
    }
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
    QObject::connect(ui_.target_button, SIGNAL(clicked()),
            this, SLOT(on_target_clicked())     );
    QObject::connect(ui_.Reset_button, SIGNAL(clicked()),
            this, SLOT(on_reset_clicked())      );
    //QObject::connect(ui_.pushButton, SIGNAL(clicked()),
    //        this, SLOT(on_activated_clicked())      );
    //QObject::connect(ui_.quitbutton, SIGNAL(clicked()),
    //        this, SLOT(on_quit_clicked())       );
    QObject::connect(ui_.listView, SIGNAL(clicked(const QModelIndex&)),
            this, SLOT(onClickListItem(const QModelIndex&))     );
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
    list << "rqt_plot"
         << "command: feedforward"
         << "command: PID"
         << "command: gravity"
         << "command: friction";

    model->setStringList(list);
    ui_.listView->setModel(model);

    gain_pub = nh.advertise<std_msgs::Float64MultiArray>("pidgain", 3);
    target_pub = nh.advertise<std_msgs::Float64>("Target", 1);
    reset_pub = nh.advertise<std_msgs::Int32>("reset", 1);
    command_pub = nh.advertise<std_msgs::Int32MultiArray>("control_command", 1);

    msg.data.resize(3);
    msgcommand.data.resize(4);

    command_reset = 0;
    for(int i = 0; i < 4; i++) {
        msgcommand.data[i] = 1;
    }    
    msgreset.data = command_reset;
    reset_pub.publish(msgreset);
    command_pub.publish(msgcommand);
    
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