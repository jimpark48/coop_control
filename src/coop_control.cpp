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
static double kp_s, ki_s, kd_s;
static double kp_h, ki_h, kd_h;
static double kp_k, ki_k, kd_k;
static double kp_o, ki_o, kd_o;
double gapkp, gapki, gapkd;
std_msgs::Float64MultiArray msg;
std_msgs::Float64 msgtarget;
std_msgs::Int32 msgreset;
//[0] is feedforward, [1] is PID, [2] is gravity, [3] is friction
std_msgs::Int32MultiArray msgcommand;
std_msgs::Float64MultiArray msgleg;

int checkedcount;
QString qstr1;
QString qstr2;
char *plotstring = (char*)malloc(200);
QStringList list2;
char *message1;

char *str;

int command_reset = 0;

//sum two strings
const char *sumstring(const char *a,const char *b) {
    int i,j;
    char *newstring;
    newstring=(char*)malloc(strlen(a)+strlen(b));
    for(i=0;i<strlen(a);i++)
      newstring[i]=a[i];
    for(j=0;j<strlen(b);j++)
      newstring[i+j]=b[j];
    newstring[i+j]='\0';
    return newstring;
}
//compare long string and short string
//if long string includes short string, comparestring function returns 
//starting position value of short string in long string 
int comparestring(const char *long1, const char *short1) {
  int i, j, count2;
  for(i=0; i<strlen(long1)-strlen(short1)+1; i++) {
    count2 = 0;
    for(j=0; j<strlen(short1); j++) {
      if(long1[i+j] == short1[j]) count2++;
    }
    if(count2 == strlen(short1)) return i;
  }
  return 10000;
}
//delete b string in a string
char *minusstring(char *a, char *b) {
  int k = comparestring(a, b);
  if(k == 10000) {
    return a;
  }
  else {
    int i=k;
    while(a[i+strlen(b)] != '\0') {
      a[i] = a[i+strlen(b)+1];
      i++;
    }
    a[i] = '\0';
    return a;
  }
}

void msgCallbackinitgain(const std_msgs::Float64MultiArray::ConstPtr &msg_dummy)
{
    kp_s = msg_dummy->data[0];
    ki_s = msg_dummy->data[1];
    kd_s = msg_dummy->data[2];

    kp_h = msg_dummy->data[3];
    ki_h = msg_dummy->data[4];
    kd_h = msg_dummy->data[5];

    kp_k = msg_dummy->data[6];
    ki_k = msg_dummy->data[7];
    kd_k = msg_dummy->data[8];

    /*
    qstr = QString::number(kp_s);
    ui_.lineEdit_2->setText(qstr);
    qstr = QString::number(ki_s);
    ui_.lineEdit_3->setText(qstr);
    qstr = QString::number(kd_s);
    ui_.lineEdit_4->setText(qstr);

    qstr = QString::number(kp_h);
    ui_.lineEdit_6->setText(qstr);
    qstr = QString::number(ki_h);
    ui_.lineEdit_7->setText(qstr);
    qstr = QString::number(kd_h);
    ui_.lineEdit_5->setText(qstr);

    qstr = QString::number(kp_k);
    ui_.lineEdit_10->setText(qstr);
    qstr = QString::number(ki_k);
    ui_.lineEdit_8->setText(qstr);
    qstr = QString::number(kd_k);
    ui_.lineEdit_9->setText(qstr);
    */
}

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

    //qstr = ui_.lineEdit_2->text();
    kp = ui_.lineEdit_6->text().toDouble();
    //qstr = ui_.lineEdit_2->text();
    ki = ui_.lineEdit_7->text().toDouble();
    //qstr = ui_.lineEdit_2->text();
    kd = ui_.lineEdit_5->text().toDouble();
    msg.data[3] = kp;
    msg.data[4] = ki;
    msg.data[5] = kd;

    //qstr = ui_.lineEdit_2->text();
    kp = ui_.lineEdit_10->text().toDouble();
    //qstr = ui_.lineEdit_2->text();
    ki = ui_.lineEdit_8->text().toDouble();
    //qstr = ui_.lineEdit_2->text();
    kd = ui_.lineEdit_9->text().toDouble();
    msg.data[6] = kp;
    msg.data[7] = ki;
    msg.data[8] = kd;

    if( MPCcount == 1 )
    {
        //qstr = ui_.lineEdit_2->text();
        kp = ui_.lineEdit_12->text().toDouble();
        //qstr = ui_.lineEdit_2->text();
        ki = ui_.lineEdit_11->text().toDouble();
        //qstr = ui_.lineEdit_2->text();
        kd = ui_.lineEdit_13->text().toDouble();
        msg.data[9] = kp;
        msg.data[10] = ki;
        msg.data[11] = kd;
    }

    if( MPCcount == 0 )
        gain_pub.publish(msg);    
    else
        MPCgain_pub.publish(msg);
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

    //qstr = ui_.lineEdit_2->text();
    kp = ui_.lineEdit_6->text().toDouble();
    //qstr = ui_.lineEdit_2->text();
    ki = ui_.lineEdit_7->text().toDouble();
    //qstr = ui_.lineEdit_2->text();
    kd = ui_.lineEdit_5->text().toDouble();
    msg.data[3] = kp;
    msg.data[4] = ki;
    msg.data[5] = kd;

    //qstr = ui_.lineEdit_2->text();
    kp = ui_.lineEdit_10->text().toDouble();
    //qstr = ui_.lineEdit_2->text();
    ki = ui_.lineEdit_8->text().toDouble();
    //qstr = ui_.lineEdit_2->text();
    kd = ui_.lineEdit_9->text().toDouble();
    msg.data[6] = kp;
    msg.data[7] = ki;
    msg.data[8] = kd;

    if( MPCcount == 1 )
    {
        //qstr = ui_.lineEdit_2->text();
        kp = ui_.lineEdit_12->text().toDouble();
        //qstr = ui_.lineEdit_2->text();
        ki = ui_.lineEdit_11->text().toDouble();
        //qstr = ui_.lineEdit_2->text();
        kd = ui_.lineEdit_13->text().toDouble();
        msg.data[9] = kp;
        msg.data[10] = ki;
        msg.data[11] = kd;
    }

    if( MPCcount == 0 )
        gain_pub.publish(msg);    
    else
        MPCgain_pub.publish(msg);
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

    //qstr = ui_.lineEdit_2->text();
    kp = ui_.lineEdit_6->text().toDouble();
    //qstr = ui_.lineEdit_2->text();
    ki = ui_.lineEdit_7->text().toDouble();
    //qstr = ui_.lineEdit_2->text();
    kd = ui_.lineEdit_5->text().toDouble();
    msg.data[3] = kp;
    msg.data[4] = ki;
    msg.data[5] = kd;

    //qstr = ui_.lineEdit_2->text();
    kp = ui_.lineEdit_10->text().toDouble();
    //qstr = ui_.lineEdit_2->text();
    ki = ui_.lineEdit_8->text().toDouble();
    //qstr = ui_.lineEdit_2->text();
    kd = ui_.lineEdit_9->text().toDouble();
    msg.data[6] = kp;
    msg.data[7] = ki;
    msg.data[8] = kd;

    if( MPCcount == 1 )
    {
        //qstr = ui_.lineEdit_2->text();
        kp = ui_.lineEdit_12->text().toDouble();
        //qstr = ui_.lineEdit_2->text();
        ki = ui_.lineEdit_11->text().toDouble();
        //qstr = ui_.lineEdit_2->text();
        kd = ui_.lineEdit_13->text().toDouble();
        msg.data[9] = kp;
        msg.data[10] = ki;
        msg.data[11] = kd;
    }

    if( MPCcount == 0 )
        gain_pub.publish(msg);    
    else
        MPCgain_pub.publish(msg);
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

    //qstr = ui_.lineEdit_2->text();
    kp = ui_.lineEdit_6->text().toDouble();
    //qstr = ui_.lineEdit_2->text();
    ki = ui_.lineEdit_7->text().toDouble();
    //qstr = ui_.lineEdit_2->text();
    kd = ui_.lineEdit_5->text().toDouble();
    msg.data[3] = kp;
    msg.data[4] = ki;
    msg.data[5] = kd;

    //qstr = ui_.lineEdit_2->text();
    kp = ui_.lineEdit_10->text().toDouble();
    //qstr = ui_.lineEdit_2->text();
    ki = ui_.lineEdit_8->text().toDouble();
    //qstr = ui_.lineEdit_2->text();
    kd = ui_.lineEdit_9->text().toDouble();
    msg.data[6] = kp;
    msg.data[7] = ki;
    msg.data[8] = kd;

    if( MPCcount == 1 )
    {
        //qstr = ui_.lineEdit_2->text();
        kp = ui_.lineEdit_12->text().toDouble();
        //qstr = ui_.lineEdit_2->text();
        ki = ui_.lineEdit_11->text().toDouble();
        //qstr = ui_.lineEdit_2->text();
        kd = ui_.lineEdit_13->text().toDouble();
        msg.data[9] = kp;
        msg.data[10] = ki;
        msg.data[11] = kd;
    }

    if( MPCcount == 0 )
        gain_pub.publish(msg);    
    else
        MPCgain_pub.publish(msg); 
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

    //qstr = ui_.lineEdit_2->text();
    kp = ui_.lineEdit_6->text().toDouble();
    //qstr = ui_.lineEdit_2->text();
    ki = ui_.lineEdit_7->text().toDouble();
    //qstr = ui_.lineEdit_2->text();
    kd = ui_.lineEdit_5->text().toDouble();
    msg.data[3] = kp;
    msg.data[4] = ki;
    msg.data[5] = kd;

    //qstr = ui_.lineEdit_2->text();
    kp = ui_.lineEdit_10->text().toDouble();
    //qstr = ui_.lineEdit_2->text();
    ki = ui_.lineEdit_8->text().toDouble();
    //qstr = ui_.lineEdit_2->text();
    kd = ui_.lineEdit_9->text().toDouble();
    msg.data[6] = kp;
    msg.data[7] = ki;
    msg.data[8] = kd;

    if( MPCcount == 1 )
    {
        //qstr = ui_.lineEdit_2->text();
        kp = ui_.lineEdit_12->text().toDouble();
        //qstr = ui_.lineEdit_2->text();
        ki = ui_.lineEdit_11->text().toDouble();
        //qstr = ui_.lineEdit_2->text();
        kd = ui_.lineEdit_13->text().toDouble();
        msg.data[9] = kp;
        msg.data[10] = ki;
        msg.data[11] = kd;
    }

    if( MPCcount == 0 )
        gain_pub.publish(msg);    
    else
        MPCgain_pub.publish(msg);
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

    //qstr = ui_.lineEdit_2->text();
    kp = ui_.lineEdit_6->text().toDouble();
    //qstr = ui_.lineEdit_2->text();
    ki = ui_.lineEdit_7->text().toDouble();
    //qstr = ui_.lineEdit_2->text();
    kd = ui_.lineEdit_5->text().toDouble();
    msg.data[3] = kp;
    msg.data[4] = ki;
    msg.data[5] = kd;

    //qstr = ui_.lineEdit_2->text();
    kp = ui_.lineEdit_10->text().toDouble();
    //qstr = ui_.lineEdit_2->text();
    ki = ui_.lineEdit_8->text().toDouble();
    //qstr = ui_.lineEdit_2->text();
    kd = ui_.lineEdit_9->text().toDouble();
    msg.data[6] = kp;
    msg.data[7] = ki;
    msg.data[8] = kd;

    if( MPCcount == 1 )
    {
        //qstr = ui_.lineEdit_2->text();
        kp = ui_.lineEdit_12->text().toDouble();
        //qstr = ui_.lineEdit_2->text();
        ki = ui_.lineEdit_11->text().toDouble();
        //qstr = ui_.lineEdit_2->text();
        kd = ui_.lineEdit_13->text().toDouble();
        msg.data[9] = kp;
        msg.data[10] = ki;
        msg.data[11] = kd;
    }

    if( MPCcount == 0 )
        gain_pub.publish(msg);    
    else
        MPCgain_pub.publish(msg);  
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

    //qstr = ui_.lineEdit_2->text();
    kp = ui_.lineEdit_6->text().toDouble();
    //qstr = ui_.lineEdit_2->text();
    ki = ui_.lineEdit_7->text().toDouble();
    //qstr = ui_.lineEdit_2->text();
    kd = ui_.lineEdit_5->text().toDouble();
    msg.data[3] = kp;
    msg.data[4] = ki;
    msg.data[5] = kd;

    //qstr = ui_.lineEdit_2->text();
    kp = ui_.lineEdit_10->text().toDouble();
    //qstr = ui_.lineEdit_2->text();
    ki = ui_.lineEdit_8->text().toDouble();
    //qstr = ui_.lineEdit_2->text();
    kd = ui_.lineEdit_9->text().toDouble();
    msg.data[6] = kp;
    msg.data[7] = ki;
    msg.data[8] = kd;

    if( MPCcount == 1 )
    {
        //qstr = ui_.lineEdit_2->text();
        kp = ui_.lineEdit_12->text().toDouble();
        //qstr = ui_.lineEdit_2->text();
        ki = ui_.lineEdit_11->text().toDouble();
        //qstr = ui_.lineEdit_2->text();
        kd = ui_.lineEdit_13->text().toDouble();
        msg.data[9] = kp;
        msg.data[10] = ki;
        msg.data[11] = kd;
    }

    if( MPCcount == 0 )
        gain_pub.publish(msg);    
    else
        MPCgain_pub.publish(msg); 
}

void coopPlugin::on_kp_h_plus_clicked() 
{
    kp = ui_.lineEdit_6->text().toDouble();
    ki = ui_.lineEdit_7->text().toDouble();
    kd = ui_.lineEdit_5->text().toDouble();

    gapkp = ui_.gap_kp->text().toDouble();
    kp = kp+gapkp;
    qstr = QString::number(kp);
    //ui_.lineEdit->setText(qstr);
    ui_.lineEdit_6->setText(qstr);

    qstr = QString("kp : %1     ki : %2     kd : %3").arg(kp).arg(ki).arg(kd);
    ui_.lineEdit->setText(qstr);

    msg.data[3] = kp;
    msg.data[4] = ki;
    msg.data[5] = kd;

    //qstr = ui_.lineEdit_2->text();
    kp = ui_.lineEdit_2->text().toDouble();
    //qstr = ui_.lineEdit_2->text();
    ki = ui_.lineEdit_3->text().toDouble();
    //qstr = ui_.lineEdit_2->text();
    kd = ui_.lineEdit_4->text().toDouble();
    msg.data[0] = kp;
    msg.data[1] = ki;
    msg.data[2] = kd;

    //qstr = ui_.lineEdit_2->text();
    kp = ui_.lineEdit_10->text().toDouble();
    //qstr = ui_.lineEdit_2->text();
    ki = ui_.lineEdit_8->text().toDouble();
    //qstr = ui_.lineEdit_2->text();
    kd = ui_.lineEdit_9->text().toDouble();
    msg.data[6] = kp;
    msg.data[7] = ki;
    msg.data[8] = kd;

    if( MPCcount == 1 )
    {
        //qstr = ui_.lineEdit_2->text();
        kp = ui_.lineEdit_12->text().toDouble();
        //qstr = ui_.lineEdit_2->text();
        ki = ui_.lineEdit_11->text().toDouble();
        //qstr = ui_.lineEdit_2->text();
        kd = ui_.lineEdit_13->text().toDouble();
        msg.data[9] = kp;
        msg.data[10] = ki;
        msg.data[11] = kd;
    }

    if( MPCcount == 0 )
        gain_pub.publish(msg);    
    else
        MPCgain_pub.publish(msg);  
}

void coopPlugin::on_kp_h_minus_clicked()
{
    kp = ui_.lineEdit_6->text().toDouble();
    ki = ui_.lineEdit_7->text().toDouble();
    kd = ui_.lineEdit_5->text().toDouble();

    gapkp = ui_.gap_kp->text().toDouble();
    kp = kp-gapkp;
    qstr = QString::number(kp);
    //ui_.lineEdit->setText(qstr);
    ui_.lineEdit_6->setText(qstr);

    qstr = QString("kp : %1     ki : %2     kd : %3").arg(kp).arg(ki).arg(kd);
    ui_.lineEdit->setText(qstr);

    msg.data[3] = kp;
    msg.data[4] = ki;
    msg.data[5] = kd;

    //qstr = ui_.lineEdit_2->text();
    kp = ui_.lineEdit_2->text().toDouble();
    //qstr = ui_.lineEdit_2->text();
    ki = ui_.lineEdit_3->text().toDouble();
    //qstr = ui_.lineEdit_2->text();
    kd = ui_.lineEdit_4->text().toDouble();
    msg.data[0] = kp;
    msg.data[1] = ki;
    msg.data[2] = kd;

    //qstr = ui_.lineEdit_2->text();
    kp = ui_.lineEdit_10->text().toDouble();
    //qstr = ui_.lineEdit_2->text();
    ki = ui_.lineEdit_8->text().toDouble();
    //qstr = ui_.lineEdit_2->text();
    kd = ui_.lineEdit_9->text().toDouble();
    msg.data[6] = kp;
    msg.data[7] = ki;
    msg.data[8] = kd;

    if( MPCcount == 1 )
    {
        //qstr = ui_.lineEdit_2->text();
        kp = ui_.lineEdit_12->text().toDouble();
        //qstr = ui_.lineEdit_2->text();
        ki = ui_.lineEdit_11->text().toDouble();
        //qstr = ui_.lineEdit_2->text();
        kd = ui_.lineEdit_13->text().toDouble();
        msg.data[9] = kp;
        msg.data[10] = ki;
        msg.data[11] = kd;
    }

    if( MPCcount == 0 )
        gain_pub.publish(msg);    
    else
        MPCgain_pub.publish(msg); 
}

void coopPlugin::on_ki_h_plus_clicked() 
{
    kp = ui_.lineEdit_6->text().toDouble();
    ki = ui_.lineEdit_7->text().toDouble();
    kd = ui_.lineEdit_5->text().toDouble();

    gapki = ui_.gap_ki->text().toDouble();
    ki = ki+gapki;
    qstr = QString::number(ki);
    //ui_.lineEdit->setText(qstr);
    ui_.lineEdit_7->setText(qstr);

    qstr = QString("kp : %1     ki : %2     kd : %3").arg(kp).arg(ki).arg(kd);
    ui_.lineEdit->setText(qstr);

    msg.data[3] = kp;
    msg.data[4] = ki;
    msg.data[5] = kd;

    //qstr = ui_.lineEdit_2->text();
    kp = ui_.lineEdit_2->text().toDouble();
    //qstr = ui_.lineEdit_2->text();
    ki = ui_.lineEdit_3->text().toDouble();
    //qstr = ui_.lineEdit_2->text();
    kd = ui_.lineEdit_4->text().toDouble();
    msg.data[0] = kp;
    msg.data[1] = ki;
    msg.data[2] = kd;

    //qstr = ui_.lineEdit_2->text();
    kp = ui_.lineEdit_10->text().toDouble();
    //qstr = ui_.lineEdit_2->text();
    ki = ui_.lineEdit_8->text().toDouble();
    //qstr = ui_.lineEdit_2->text();
    kd = ui_.lineEdit_9->text().toDouble();
    msg.data[6] = kp;
    msg.data[7] = ki;
    msg.data[8] = kd;

    if( MPCcount == 1 )
    {
        //qstr = ui_.lineEdit_2->text();
        kp = ui_.lineEdit_12->text().toDouble();
        //qstr = ui_.lineEdit_2->text();
        ki = ui_.lineEdit_11->text().toDouble();
        //qstr = ui_.lineEdit_2->text();
        kd = ui_.lineEdit_13->text().toDouble();
        msg.data[9] = kp;
        msg.data[10] = ki;
        msg.data[11] = kd;
    }

    if( MPCcount == 0 )
        gain_pub.publish(msg);    
    else
        MPCgain_pub.publish(msg);
}

void coopPlugin::on_ki_h_minus_clicked()
{
    kp = ui_.lineEdit_6->text().toDouble();
    ki = ui_.lineEdit_7->text().toDouble();
    kd = ui_.lineEdit_5->text().toDouble();

    gapki = ui_.gap_ki->text().toDouble();
    ki = ki-gapki;
    qstr = QString::number(ki);
    //ui_.lineEdit->setText(qstr);
    ui_.lineEdit_7->setText(qstr);

    qstr = QString("kp : %1     ki : %2     kd : %3").arg(kp).arg(ki).arg(kd);
    ui_.lineEdit->setText(qstr);

    msg.data[3] = kp;
    msg.data[4] = ki;
    msg.data[5] = kd;

    //qstr = ui_.lineEdit_2->text();
    kp = ui_.lineEdit_2->text().toDouble();
    //qstr = ui_.lineEdit_2->text();
    ki = ui_.lineEdit_3->text().toDouble();
    //qstr = ui_.lineEdit_2->text();
    kd = ui_.lineEdit_4->text().toDouble();
    msg.data[0] = kp;
    msg.data[1] = ki;
    msg.data[2] = kd;

    //qstr = ui_.lineEdit_2->text();
    kp = ui_.lineEdit_10->text().toDouble();
    //qstr = ui_.lineEdit_2->text();
    ki = ui_.lineEdit_8->text().toDouble();
    //qstr = ui_.lineEdit_2->text();
    kd = ui_.lineEdit_9->text().toDouble();
    msg.data[6] = kp;
    msg.data[7] = ki;
    msg.data[8] = kd;

    if( MPCcount == 1 )
    {
        //qstr = ui_.lineEdit_2->text();
        kp = ui_.lineEdit_12->text().toDouble();
        //qstr = ui_.lineEdit_2->text();
        ki = ui_.lineEdit_11->text().toDouble();
        //qstr = ui_.lineEdit_2->text();
        kd = ui_.lineEdit_13->text().toDouble();
        msg.data[9] = kp;
        msg.data[10] = ki;
        msg.data[11] = kd;
    }

    if( MPCcount == 0 )
        gain_pub.publish(msg);    
    else
        MPCgain_pub.publish(msg);
}

void coopPlugin::on_kd_h_plus_clicked() 
{
    kp = ui_.lineEdit_6->text().toDouble();
    ki = ui_.lineEdit_7->text().toDouble();
    kd = ui_.lineEdit_5->text().toDouble();

    gapkd = ui_.gap_kd->text().toDouble();
    kd = kd+gapkd;
    qstr = QString::number(kd);
    //ui_.lineEdit->setText(qstr);
    ui_.lineEdit_5->setText(qstr);

    qstr = QString("kp : %1     ki : %2     kd : %3").arg(kp).arg(ki).arg(kd);
    ui_.lineEdit->setText(qstr);

    msg.data[3] = kp;
    msg.data[4] = ki;
    msg.data[5] = kd;

    //qstr = ui_.lineEdit_2->text();
    kp = ui_.lineEdit_2->text().toDouble();
    //qstr = ui_.lineEdit_2->text();
    ki = ui_.lineEdit_3->text().toDouble();
    //qstr = ui_.lineEdit_2->text();
    kd = ui_.lineEdit_4->text().toDouble();
    msg.data[0] = kp;
    msg.data[1] = ki;
    msg.data[2] = kd;

    //qstr = ui_.lineEdit_2->text();
    kp = ui_.lineEdit_10->text().toDouble();
    //qstr = ui_.lineEdit_2->text();
    ki = ui_.lineEdit_8->text().toDouble();
    //qstr = ui_.lineEdit_2->text();
    kd = ui_.lineEdit_9->text().toDouble();
    msg.data[6] = kp;
    msg.data[7] = ki;
    msg.data[8] = kd;

    if( MPCcount == 1 )
    {
        //qstr = ui_.lineEdit_2->text();
        kp = ui_.lineEdit_12->text().toDouble();
        //qstr = ui_.lineEdit_2->text();
        ki = ui_.lineEdit_11->text().toDouble();
        //qstr = ui_.lineEdit_2->text();
        kd = ui_.lineEdit_13->text().toDouble();
        msg.data[9] = kp;
        msg.data[10] = ki;
        msg.data[11] = kd;
    }

    if( MPCcount == 0 )
        gain_pub.publish(msg);    
    else
        MPCgain_pub.publish(msg);
}

void coopPlugin::on_kd_h_minus_clicked()
{
    kp = ui_.lineEdit_6->text().toDouble();
    ki = ui_.lineEdit_7->text().toDouble();
    kd = ui_.lineEdit_5->text().toDouble();

    gapkd = ui_.gap_kd->text().toDouble();
    kd = kd-gapkd;
    qstr = QString::number(kd);
    //ui_.lineEdit->setText(qstr);
    ui_.lineEdit_5->setText(qstr);

    qstr = QString("kp : %1     ki : %2     kd : %3").arg(kp).arg(ki).arg(kd);
    ui_.lineEdit->setText(qstr);

    msg.data[3] = kp;
    msg.data[4] = ki;
    msg.data[5] = kd;

    //qstr = ui_.lineEdit_2->text();
    kp = ui_.lineEdit_2->text().toDouble();
    //qstr = ui_.lineEdit_2->text();
    ki = ui_.lineEdit_3->text().toDouble();
    //qstr = ui_.lineEdit_2->text();
    kd = ui_.lineEdit_4->text().toDouble();
    msg.data[0] = kp;
    msg.data[1] = ki;
    msg.data[2] = kd;

    //qstr = ui_.lineEdit_2->text();
    kp = ui_.lineEdit_10->text().toDouble();
    //qstr = ui_.lineEdit_2->text();
    ki = ui_.lineEdit_8->text().toDouble();
    //qstr = ui_.lineEdit_2->text();
    kd = ui_.lineEdit_9->text().toDouble();
    msg.data[6] = kp;
    msg.data[7] = ki;
    msg.data[8] = kd;

    if( MPCcount == 1 )
    {
        //qstr = ui_.lineEdit_2->text();
        kp = ui_.lineEdit_12->text().toDouble();
        //qstr = ui_.lineEdit_2->text();
        ki = ui_.lineEdit_11->text().toDouble();
        //qstr = ui_.lineEdit_2->text();
        kd = ui_.lineEdit_13->text().toDouble();
        msg.data[9] = kp;
        msg.data[10] = ki;
        msg.data[11] = kd;
    }

    if( MPCcount == 0 )
        gain_pub.publish(msg);    
    else
        MPCgain_pub.publish(msg);  
}

void coopPlugin::on_kp_k_plus_clicked() 
{
    kp = ui_.lineEdit_10->text().toDouble();
    ki = ui_.lineEdit_8->text().toDouble();
    kd = ui_.lineEdit_9->text().toDouble();

    gapkp = ui_.gap_kp->text().toDouble();
    kp = kp+gapkp;
    qstr = QString::number(kp);
    //ui_.lineEdit->setText(qstr);
    ui_.lineEdit_10->setText(qstr);

    qstr = QString("kp : %1     ki : %2     kd : %3").arg(kp).arg(ki).arg(kd);
    ui_.lineEdit->setText(qstr);

    msg.data[6] = kp;
    msg.data[7] = ki;
    msg.data[8] = kd;

    //qstr = ui_.lineEdit_2->text();
    kp = ui_.lineEdit_2->text().toDouble();
    //qstr = ui_.lineEdit_2->text();
    ki = ui_.lineEdit_3->text().toDouble();
    //qstr = ui_.lineEdit_2->text();
    kd = ui_.lineEdit_4->text().toDouble();
    msg.data[0] = kp;
    msg.data[1] = ki;
    msg.data[2] = kd;

    //qstr = ui_.lineEdit_2->text();
    kp = ui_.lineEdit_6->text().toDouble();
    //qstr = ui_.lineEdit_2->text();
    ki = ui_.lineEdit_7->text().toDouble();
    //qstr = ui_.lineEdit_2->text();
    kd = ui_.lineEdit_5->text().toDouble();
    msg.data[3] = kp;
    msg.data[4] = ki;
    msg.data[5] = kd;

    if( MPCcount == 1 )
    {
        //qstr = ui_.lineEdit_2->text();
        kp = ui_.lineEdit_12->text().toDouble();
        //qstr = ui_.lineEdit_2->text();
        ki = ui_.lineEdit_11->text().toDouble();
        //qstr = ui_.lineEdit_2->text();
        kd = ui_.lineEdit_13->text().toDouble();
        msg.data[9] = kp;
        msg.data[10] = ki;
        msg.data[11] = kd;
    }

    if( MPCcount == 0 )
        gain_pub.publish(msg);    
    else
        MPCgain_pub.publish(msg); 
}

void coopPlugin::on_kp_k_minus_clicked()
{
    kp = ui_.lineEdit_10->text().toDouble();
    ki = ui_.lineEdit_8->text().toDouble();
    kd = ui_.lineEdit_9->text().toDouble();

    gapkp = ui_.gap_kp->text().toDouble();
    kp = kp-gapkp;
    qstr = QString::number(kp);
    //ui_.lineEdit->setText(qstr);
    ui_.lineEdit_10->setText(qstr);

    qstr = QString("kp : %1     ki : %2     kd : %3").arg(kp).arg(ki).arg(kd);
    ui_.lineEdit->setText(qstr);

    msg.data[6] = kp;
    msg.data[7] = ki;
    msg.data[8] = kd;

    //qstr = ui_.lineEdit_2->text();
    kp = ui_.lineEdit_2->text().toDouble();
    //qstr = ui_.lineEdit_2->text();
    ki = ui_.lineEdit_3->text().toDouble();
    //qstr = ui_.lineEdit_2->text();
    kd = ui_.lineEdit_4->text().toDouble();
    msg.data[0] = kp;
    msg.data[1] = ki;
    msg.data[2] = kd;

    //qstr = ui_.lineEdit_2->text();
    kp = ui_.lineEdit_6->text().toDouble();
    //qstr = ui_.lineEdit_2->text();
    ki = ui_.lineEdit_7->text().toDouble();
    //qstr = ui_.lineEdit_2->text();
    kd = ui_.lineEdit_5->text().toDouble();
    msg.data[3] = kp;
    msg.data[4] = ki;
    msg.data[5] = kd;

    if( MPCcount == 1 )
    {
        //qstr = ui_.lineEdit_2->text();
        kp = ui_.lineEdit_12->text().toDouble();
        //qstr = ui_.lineEdit_2->text();
        ki = ui_.lineEdit_11->text().toDouble();
        //qstr = ui_.lineEdit_2->text();
        kd = ui_.lineEdit_13->text().toDouble();
        msg.data[9] = kp;
        msg.data[10] = ki;
        msg.data[11] = kd;
    }

    if( MPCcount == 0 )
        gain_pub.publish(msg);    
    else
        MPCgain_pub.publish(msg);
}

void coopPlugin::on_ki_k_plus_clicked() 
{
    kp = ui_.lineEdit_10->text().toDouble();
    ki = ui_.lineEdit_8->text().toDouble();
    kd = ui_.lineEdit_9->text().toDouble();

    gapki = ui_.gap_ki->text().toDouble();
    ki = ki+gapki;
    qstr = QString::number(ki);
    //ui_.lineEdit->setText(qstr);
    ui_.lineEdit_8->setText(qstr);

    qstr = QString("kp : %1     ki : %2     kd : %3").arg(kp).arg(ki).arg(kd);
    ui_.lineEdit->setText(qstr);

    msg.data[6] = kp;
    msg.data[7] = ki;
    msg.data[8] = kd;

    //qstr = ui_.lineEdit_2->text();
    kp = ui_.lineEdit_2->text().toDouble();
    //qstr = ui_.lineEdit_2->text();
    ki = ui_.lineEdit_3->text().toDouble();
    //qstr = ui_.lineEdit_2->text();
    kd = ui_.lineEdit_4->text().toDouble();
    msg.data[0] = kp;
    msg.data[1] = ki;
    msg.data[2] = kd;

    //qstr = ui_.lineEdit_2->text();
    kp = ui_.lineEdit_6->text().toDouble();
    //qstr = ui_.lineEdit_2->text();
    ki = ui_.lineEdit_7->text().toDouble();
    //qstr = ui_.lineEdit_2->text();
    kd = ui_.lineEdit_5->text().toDouble();
    msg.data[3] = kp;
    msg.data[4] = ki;
    msg.data[5] = kd;

    if( MPCcount == 1 )
    {
        //qstr = ui_.lineEdit_2->text();
        kp = ui_.lineEdit_12->text().toDouble();
        //qstr = ui_.lineEdit_2->text();
        ki = ui_.lineEdit_11->text().toDouble();
        //qstr = ui_.lineEdit_2->text();
        kd = ui_.lineEdit_13->text().toDouble();
        msg.data[9] = kp;
        msg.data[10] = ki;
        msg.data[11] = kd;
    }

    if( MPCcount == 0 )
        gain_pub.publish(msg);    
    else
        MPCgain_pub.publish(msg); 
}

void coopPlugin::on_ki_k_minus_clicked()
{
    kp = ui_.lineEdit_10->text().toDouble();
    ki = ui_.lineEdit_8->text().toDouble();
    kd = ui_.lineEdit_9->text().toDouble();

    gapki = ui_.gap_ki->text().toDouble();
    ki = ki-gapki;
    qstr = QString::number(ki);
    //ui_.lineEdit->setText(qstr);
    ui_.lineEdit_8->setText(qstr);

    qstr = QString("kp : %1     ki : %2     kd : %3").arg(kp).arg(ki).arg(kd);
    ui_.lineEdit->setText(qstr);

    msg.data[6] = kp;
    msg.data[7] = ki;
    msg.data[8] = kd;

    //qstr = ui_.lineEdit_2->text();
    kp = ui_.lineEdit_2->text().toDouble();
    //qstr = ui_.lineEdit_2->text();
    ki = ui_.lineEdit_3->text().toDouble();
    //qstr = ui_.lineEdit_2->text();
    kd = ui_.lineEdit_4->text().toDouble();
    msg.data[0] = kp;
    msg.data[1] = ki;
    msg.data[2] = kd;

    //qstr = ui_.lineEdit_2->text();
    kp = ui_.lineEdit_6->text().toDouble();
    //qstr = ui_.lineEdit_2->text();
    ki = ui_.lineEdit_7->text().toDouble();
    //qstr = ui_.lineEdit_2->text();
    kd = ui_.lineEdit_5->text().toDouble();
    msg.data[3] = kp;
    msg.data[4] = ki;
    msg.data[5] = kd;

    if( MPCcount == 1 )
    {
        //qstr = ui_.lineEdit_2->text();
        kp = ui_.lineEdit_12->text().toDouble();
        //qstr = ui_.lineEdit_2->text();
        ki = ui_.lineEdit_11->text().toDouble();
        //qstr = ui_.lineEdit_2->text();
        kd = ui_.lineEdit_13->text().toDouble();
        msg.data[9] = kp;
        msg.data[10] = ki;
        msg.data[11] = kd;
    }

    if( MPCcount == 0 )
        gain_pub.publish(msg);    
    else
        MPCgain_pub.publish(msg);   
}

void coopPlugin::on_kd_k_plus_clicked() 
{
    kp = ui_.lineEdit_10->text().toDouble();
    ki = ui_.lineEdit_8->text().toDouble();
    kd = ui_.lineEdit_9->text().toDouble();

    gapkd = ui_.gap_kd->text().toDouble();
    kd = kd+gapkd;
    qstr = QString::number(kd);
    //ui_.lineEdit->setText(qstr);
    ui_.lineEdit_9->setText(qstr);

    qstr = QString("kp : %1     ki : %2     kd : %3").arg(kp).arg(ki).arg(kd);
    ui_.lineEdit->setText(qstr);

    msg.data[6] = kp;
    msg.data[7] = ki;
    msg.data[8] = kd;

    //qstr = ui_.lineEdit_2->text();
    kp = ui_.lineEdit_2->text().toDouble();
    //qstr = ui_.lineEdit_2->text();
    ki = ui_.lineEdit_3->text().toDouble();
    //qstr = ui_.lineEdit_2->text();
    kd = ui_.lineEdit_4->text().toDouble();
    msg.data[0] = kp;
    msg.data[1] = ki;
    msg.data[2] = kd;

    //qstr = ui_.lineEdit_2->text();
    kp = ui_.lineEdit_6->text().toDouble();
    //qstr = ui_.lineEdit_2->text();
    ki = ui_.lineEdit_7->text().toDouble();
    //qstr = ui_.lineEdit_2->text();
    kd = ui_.lineEdit_5->text().toDouble();
    msg.data[3] = kp;
    msg.data[4] = ki;
    msg.data[5] = kd;

    if( MPCcount == 1 )
    {
        //qstr = ui_.lineEdit_2->text();
        kp = ui_.lineEdit_12->text().toDouble();
        //qstr = ui_.lineEdit_2->text();
        ki = ui_.lineEdit_11->text().toDouble();
        //qstr = ui_.lineEdit_2->text();
        kd = ui_.lineEdit_13->text().toDouble();
        msg.data[9] = kp;
        msg.data[10] = ki;
        msg.data[11] = kd;
    }

    if( MPCcount == 0 )
        gain_pub.publish(msg);    
    else
        MPCgain_pub.publish(msg);
}

void coopPlugin::on_kd_k_minus_clicked()
{
    kp = ui_.lineEdit_10->text().toDouble();
    ki = ui_.lineEdit_8->text().toDouble();
    kd = ui_.lineEdit_9->text().toDouble();

    gapkd = ui_.gap_kd->text().toDouble();
    kd = kd-gapkd;
    qstr = QString::number(kd);
    //ui_.lineEdit->setText(qstr);
    ui_.lineEdit_9->setText(qstr);

    qstr = QString("kp : %1     ki : %2     kd : %3").arg(kp).arg(ki).arg(kd);
    ui_.lineEdit->setText(qstr);

    msg.data[6] = kp;
    msg.data[7] = ki;
    msg.data[8] = kd;
    
    //qstr = ui_.lineEdit_2->text();
    kp = ui_.lineEdit_2->text().toDouble();
    //qstr = ui_.lineEdit_2->text();
    ki = ui_.lineEdit_3->text().toDouble();
    //qstr = ui_.lineEdit_2->text();
    kd = ui_.lineEdit_4->text().toDouble();
    msg.data[0] = kp;
    msg.data[1] = ki;
    msg.data[2] = kd;

    //qstr = ui_.lineEdit_2->text();
    kp = ui_.lineEdit_6->text().toDouble();
    //qstr = ui_.lineEdit_2->text();
    ki = ui_.lineEdit_7->text().toDouble();
    //qstr = ui_.lineEdit_2->text();
    kd = ui_.lineEdit_5->text().toDouble();
    msg.data[3] = kp;
    msg.data[4] = ki;
    msg.data[5] = kd;
    
    if( MPCcount == 1 )
    {
        //qstr = ui_.lineEdit_2->text();
        kp = ui_.lineEdit_12->text().toDouble();
        //qstr = ui_.lineEdit_2->text();
        ki = ui_.lineEdit_11->text().toDouble();
        //qstr = ui_.lineEdit_2->text();
        kd = ui_.lineEdit_13->text().toDouble();
        msg.data[9] = kp;
        msg.data[10] = ki;
        msg.data[11] = kd;
    }

    if( MPCcount == 0 )
        gain_pub.publish(msg);    
    else
        MPCgain_pub.publish(msg); 
}

void coopPlugin::on_kp_o_plus_clicked() 
{
    kp = ui_.lineEdit_12->text().toDouble();
    ki = ui_.lineEdit_11->text().toDouble();
    kd = ui_.lineEdit_13->text().toDouble();

    gapkp = ui_.gap_kp->text().toDouble();
    kp = kp+gapkp;
    qstr = QString::number(kp);
    //ui_.lineEdit->setText(qstr);
    ui_.lineEdit_12->setText(qstr);

    qstr = QString("kp : %1     ki : %2     kd : %3").arg(kp).arg(ki).arg(kd);
    ui_.lineEdit->setText(qstr);

    msg.data[9] = kp;
    msg.data[10] = ki;
    msg.data[11] = kd;

    //qstr = ui_.lineEdit_2->text();
    kp = ui_.lineEdit_2->text().toDouble();
    //qstr = ui_.lineEdit_2->text();
    ki = ui_.lineEdit_3->text().toDouble();
    //qstr = ui_.lineEdit_2->text();
    kd = ui_.lineEdit_4->text().toDouble();
    msg.data[0] = kp;
    msg.data[1] = ki;
    msg.data[2] = kd;

    //qstr = ui_.lineEdit_2->text();
    kp = ui_.lineEdit_6->text().toDouble();
    //qstr = ui_.lineEdit_2->text();
    ki = ui_.lineEdit_7->text().toDouble();
    //qstr = ui_.lineEdit_2->text();
    kd = ui_.lineEdit_5->text().toDouble();
    msg.data[3] = kp;
    msg.data[4] = ki;
    msg.data[5] = kd;

    //qstr = ui_.lineEdit_2->text();
    kp = ui_.lineEdit_10->text().toDouble();
    //qstr = ui_.lineEdit_2->text();
    ki = ui_.lineEdit_8->text().toDouble();
    //qstr = ui_.lineEdit_2->text();
    kd = ui_.lineEdit_9->text().toDouble();
    msg.data[6] = kp;
    msg.data[7] = ki;
    msg.data[8] = kd;

    if( MPCcount == 0 )
        gain_pub.publish(msg);
    else
        MPCgain_pub.publish(msg);
}

void coopPlugin::on_kp_o_minus_clicked()
{
    kp = ui_.lineEdit_12->text().toDouble();
    ki = ui_.lineEdit_11->text().toDouble();
    kd = ui_.lineEdit_13->text().toDouble();

    gapkp = ui_.gap_kp->text().toDouble();
    kp = kp-gapkp;
    qstr = QString::number(kp);
    //ui_.lineEdit->setText(qstr);
    ui_.lineEdit_12->setText(qstr);

    qstr = QString("kp : %1     ki : %2     kd : %3").arg(kp).arg(ki).arg(kd);
    ui_.lineEdit->setText(qstr);

    msg.data[9] = kp;
    msg.data[10] = ki;
    msg.data[11] = kd;

    //qstr = ui_.lineEdit_2->text();
    kp = ui_.lineEdit_2->text().toDouble();
    //qstr = ui_.lineEdit_2->text();
    ki = ui_.lineEdit_3->text().toDouble();
    //qstr = ui_.lineEdit_2->text();
    kd = ui_.lineEdit_4->text().toDouble();
    msg.data[0] = kp;
    msg.data[1] = ki;
    msg.data[2] = kd;

    //qstr = ui_.lineEdit_2->text();
    kp = ui_.lineEdit_6->text().toDouble();
    //qstr = ui_.lineEdit_2->text();
    ki = ui_.lineEdit_7->text().toDouble();
    //qstr = ui_.lineEdit_2->text();
    kd = ui_.lineEdit_5->text().toDouble();
    msg.data[3] = kp;
    msg.data[4] = ki;
    msg.data[5] = kd;

    //qstr = ui_.lineEdit_2->text();
    kp = ui_.lineEdit_10->text().toDouble();
    //qstr = ui_.lineEdit_2->text();
    ki = ui_.lineEdit_8->text().toDouble();
    //qstr = ui_.lineEdit_2->text();
    kd = ui_.lineEdit_9->text().toDouble();
    msg.data[6] = kp;
    msg.data[7] = ki;
    msg.data[8] = kd;

    if( MPCcount == 0 )
        gain_pub.publish(msg);
    else
        MPCgain_pub.publish(msg);
}

void coopPlugin::on_ki_o_plus_clicked() 
{
    kp = ui_.lineEdit_12->text().toDouble();
    ki = ui_.lineEdit_11->text().toDouble();
    kd = ui_.lineEdit_13->text().toDouble();

    gapki = ui_.gap_ki->text().toDouble();
    ki = ki+gapki;
    qstr = QString::number(ki);
    //ui_.lineEdit->setText(qstr);
    ui_.lineEdit_11->setText(qstr);

    qstr = QString("kp : %1     ki : %2     kd : %3").arg(kp).arg(ki).arg(kd);
    ui_.lineEdit->setText(qstr);

    msg.data[9] = kp;
    msg.data[10] = ki;
    msg.data[11] = kd;

    //qstr = ui_.lineEdit_2->text();
    kp = ui_.lineEdit_2->text().toDouble();
    //qstr = ui_.lineEdit_2->text();
    ki = ui_.lineEdit_3->text().toDouble();
    //qstr = ui_.lineEdit_2->text();
    kd = ui_.lineEdit_4->text().toDouble();
    msg.data[0] = kp;
    msg.data[1] = ki;
    msg.data[2] = kd;

    //qstr = ui_.lineEdit_2->text();
    kp = ui_.lineEdit_6->text().toDouble();
    //qstr = ui_.lineEdit_2->text();
    ki = ui_.lineEdit_7->text().toDouble();
    //qstr = ui_.lineEdit_2->text();
    kd = ui_.lineEdit_5->text().toDouble();
    msg.data[3] = kp;
    msg.data[4] = ki;
    msg.data[5] = kd;

    //qstr = ui_.lineEdit_2->text();
    kp = ui_.lineEdit_10->text().toDouble();
    //qstr = ui_.lineEdit_2->text();
    ki = ui_.lineEdit_8->text().toDouble();
    //qstr = ui_.lineEdit_2->text();
    kd = ui_.lineEdit_9->text().toDouble();
    msg.data[6] = kp;
    msg.data[7] = ki;
    msg.data[8] = kd;

    if( MPCcount == 0 )
        gain_pub.publish(msg);
    else
        MPCgain_pub.publish(msg);   
}

void coopPlugin::on_ki_o_minus_clicked()
{
    kp = ui_.lineEdit_12->text().toDouble();
    ki = ui_.lineEdit_11->text().toDouble();
    kd = ui_.lineEdit_13->text().toDouble();

    gapki = ui_.gap_ki->text().toDouble();
    ki = ki-gapki;
    qstr = QString::number(ki);
    //ui_.lineEdit->setText(qstr);
    ui_.lineEdit_11->setText(qstr);

    qstr = QString("kp : %1     ki : %2     kd : %3").arg(kp).arg(ki).arg(kd);
    ui_.lineEdit->setText(qstr);

    msg.data[9] = kp;
    msg.data[10] = ki;
    msg.data[11] = kd;

    //qstr = ui_.lineEdit_2->text();
    kp = ui_.lineEdit_2->text().toDouble();
    //qstr = ui_.lineEdit_2->text();
    ki = ui_.lineEdit_3->text().toDouble();
    //qstr = ui_.lineEdit_2->text();
    kd = ui_.lineEdit_4->text().toDouble();
    msg.data[0] = kp;
    msg.data[1] = ki;
    msg.data[2] = kd;

    //qstr = ui_.lineEdit_2->text();
    kp = ui_.lineEdit_6->text().toDouble();
    //qstr = ui_.lineEdit_2->text();
    ki = ui_.lineEdit_7->text().toDouble();
    //qstr = ui_.lineEdit_2->text();
    kd = ui_.lineEdit_5->text().toDouble();
    msg.data[3] = kp;
    msg.data[4] = ki;
    msg.data[5] = kd;

    //qstr = ui_.lineEdit_2->text();
    kp = ui_.lineEdit_10->text().toDouble();
    //qstr = ui_.lineEdit_2->text();
    ki = ui_.lineEdit_8->text().toDouble();
    //qstr = ui_.lineEdit_2->text();
    kd = ui_.lineEdit_9->text().toDouble();
    msg.data[6] = kp;
    msg.data[7] = ki;
    msg.data[8] = kd;

    if( MPCcount == 0 )
        gain_pub.publish(msg);
    else
        MPCgain_pub.publish(msg);  
}

void coopPlugin::on_kd_o_plus_clicked() 
{
    kp = ui_.lineEdit_12->text().toDouble();
    ki = ui_.lineEdit_11->text().toDouble();
    kd = ui_.lineEdit_13->text().toDouble();

    gapkd = ui_.gap_kd->text().toDouble();
    kd = kd+gapkd;
    qstr = QString::number(kd);
    //ui_.lineEdit->setText(qstr);
    ui_.lineEdit_13->setText(qstr);

    qstr = QString("kp : %1     ki : %2     kd : %3").arg(kp).arg(ki).arg(kd);
    ui_.lineEdit->setText(qstr);

    msg.data[9] = kp;
    msg.data[10] = ki;
    msg.data[11] = kd;

    //qstr = ui_.lineEdit_2->text();
    kp = ui_.lineEdit_2->text().toDouble();
    //qstr = ui_.lineEdit_2->text();
    ki = ui_.lineEdit_3->text().toDouble();
    //qstr = ui_.lineEdit_2->text();
    kd = ui_.lineEdit_4->text().toDouble();
    msg.data[0] = kp;
    msg.data[1] = ki;
    msg.data[2] = kd;

    //qstr = ui_.lineEdit_2->text();
    kp = ui_.lineEdit_6->text().toDouble();
    //qstr = ui_.lineEdit_2->text();
    ki = ui_.lineEdit_7->text().toDouble();
    //qstr = ui_.lineEdit_2->text();
    kd = ui_.lineEdit_5->text().toDouble();
    msg.data[3] = kp;
    msg.data[4] = ki;
    msg.data[5] = kd;

    //qstr = ui_.lineEdit_2->text();
    kp = ui_.lineEdit_10->text().toDouble();
    //qstr = ui_.lineEdit_2->text();
    ki = ui_.lineEdit_8->text().toDouble();
    //qstr = ui_.lineEdit_2->text();
    kd = ui_.lineEdit_9->text().toDouble();
    msg.data[6] = kp;
    msg.data[7] = ki;
    msg.data[8] = kd;

    if( MPCcount == 0 )
        gain_pub.publish(msg);
    else
        MPCgain_pub.publish(msg);   
}

void coopPlugin::on_kd_o_minus_clicked()
{
    kp = ui_.lineEdit_12->text().toDouble();
    ki = ui_.lineEdit_11->text().toDouble();
    kd = ui_.lineEdit_13->text().toDouble();

    gapkd = ui_.gap_kd->text().toDouble();
    kd = kd-gapkd;
    qstr = QString::number(kd);
    //ui_.lineEdit->setText(qstr);
    ui_.lineEdit_9->setText(qstr);

    qstr = QString("kp : %1     ki : %2     kd : %3").arg(kp).arg(ki).arg(kd);
    ui_.lineEdit->setText(qstr);

    msg.data[9] = kp;
    msg.data[10] = ki;
    msg.data[11] = kd;
    
    //qstr = ui_.lineEdit_2->text();
    kp = ui_.lineEdit_2->text().toDouble();
    //qstr = ui_.lineEdit_2->text();
    ki = ui_.lineEdit_3->text().toDouble();
    //qstr = ui_.lineEdit_2->text();
    kd = ui_.lineEdit_4->text().toDouble();
    msg.data[0] = kp;
    msg.data[1] = ki;
    msg.data[2] = kd;

    //qstr = ui_.lineEdit_2->text();
    kp = ui_.lineEdit_6->text().toDouble();
    //qstr = ui_.lineEdit_2->text();
    ki = ui_.lineEdit_7->text().toDouble();
    //qstr = ui_.lineEdit_2->text();
    kd = ui_.lineEdit_5->text().toDouble();
    msg.data[3] = kp;
    msg.data[4] = ki;
    msg.data[5] = kd;

    //qstr = ui_.lineEdit_2->text();
    kp = ui_.lineEdit_10->text().toDouble();
    //qstr = ui_.lineEdit_2->text();
    ki = ui_.lineEdit_8->text().toDouble();
    //qstr = ui_.lineEdit_2->text();
    kd = ui_.lineEdit_9->text().toDouble();
    msg.data[6] = kp;
    msg.data[7] = ki;
    msg.data[8] = kd;

    if( MPCcount == 0 )
        gain_pub.publish(msg);
    else
        MPCgain_pub.publish(msg);
}

void coopPlugin::on_target_clicked()
{
    target = ui_.Target->text().toDouble();
    msgtarget.data = target;
    target_pub.publish(msgtarget);
}

void coopPlugin::on_reset_clicked() 
{
    qstr = QString::number(kp_s);
    ui_.lineEdit_2->setText(qstr);
    qstr = QString::number(ki_s);
    ui_.lineEdit_3->setText(qstr);
    qstr = QString::number(kd_s);
    ui_.lineEdit_4->setText(qstr);

    qstr = QString::number(kp_h);
    ui_.lineEdit_6->setText(qstr);
    qstr = QString::number(ki_h);
    ui_.lineEdit_7->setText(qstr);
    qstr = QString::number(kd_h);
    ui_.lineEdit_5->setText(qstr);

    qstr = QString::number(kp_k);
    ui_.lineEdit_10->setText(qstr);
    qstr = QString::number(ki_k);
    ui_.lineEdit_8->setText(qstr);
    qstr = QString::number(kd_k);
    ui_.lineEdit_9->setText(qstr);

    if( MPCcount == 1 )
    {
        qstr = QString::number(kp_o);
        ui_.lineEdit_12->setText(qstr);
        qstr = QString::number(ki_o);
        ui_.lineEdit_11->setText(qstr);
        qstr = QString::number(kd_o);
        ui_.lineEdit_13->setText(qstr);
    }

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
    else if(strcmp(str, "legtest") == 0) {
        msgleg.data[1] = 1;
        legtest_pub.publish(msgleg);
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
    else if(strcmp(str, "legtest") == 0) {
        msgleg.data[1] = 0;
        legtest_pub.publish(msgleg);
    }
}

//plot button function2, when listView_2 item selected
void coopPlugin::on_pushButton_3_clicked2() {
  //QString -> char* conversion
  QByteArray bytename = qstr2.toLocal8Bit();
  char *message1 = bytename.data();

  const char *message2 = "gnome-terminal -e 'rqt_plot ";
  const char *message3 = "'";
  const char *message4 = sumstring(message2, message1);
  message4 = sumstring(message4, message3);
  ROS_INFO("%s", message4);

  system(message4);
}

//plot button function1, when listView item selected
void coopPlugin::on_pushButton_3_clicked1() {
  //QString -> char* conversion
  QByteArray bytename = qstrlist.toLocal8Bit();
  /*char **/message1 = bytename.data();

  //string is divided
  char *token = NULL;
  char s1[] = " ";
  token = strtok(message1, s1);
  message1 = strtok(NULL, s1);

  const char *message2 = "gnome-terminal -e 'rqt_plot ";
  const char *message3 = "'";
  const char *message4 = sumstring(message2, message1);
  message4 = sumstring(message4, message3);
  ROS_INFO("%s", message4);

  system(message4);
}

void coopPlugin::onClickListItem2(const QModelIndex &index) {
  QObject::disconnect(ui_.pushButton_3, SIGNAL(clicked()),
            this, SLOT(on_pushButton_3_clicked1()) );
  QObject::disconnect(ui_.pushButton_3, SIGNAL(clicked()),
            this, SLOT(on_pushButton_3_clicked2()) );
  QObject::disconnect(ui_.pushButton, SIGNAL(clicked()),
            this, SLOT(on_pushButton_clicked2()) );

  //if checkedcount is 0, listView_2 activates as single section mode
  if(checkedcount == 0) {
    //qstr2 means selected row name in listView_2
    qstr2 = QString("%1").arg(index.data().toString());
    ui_.lineEdit->setText(qstr2);
  }
  //if checkedcount is 1, listView_2 activates as multi section mode
  else if(checkedcount == 1) {
    //qstr4 means new selected row name in listView_2
    QString qstr4 = QString("%1").arg(index.data().toString());
    
    //QString -> char* conversion
    QByteArray bytename = qstr2.toLocal8Bit();
    char *long1 = bytename.data();
    //QString -> char* conversion
    QByteArray bytename2 = qstr4.toLocal8Bit();
    char *short1 = bytename2.data();

    //when first selected
    if(strlen(long1) < strlen(short1)) {
      qstr2 = qstr4;
      ui_.lineEdit->setText(qstr2);
    }
    //after first selection
    else {
      //if item is selected twice, that item deletes in qstr2
      if(comparestring(long1, short1) != 10000) {
        long1 = minusstring(long1, short1);
        //char* -> QString conversion
        qstr2 = QString::fromLocal8Bit(long1);
        ui_.lineEdit->setText(qstr2);
      }
      //if item is selected once, that item adds in qstr2
      else {
        QString qstr3 = " ";
        qstr2 = qstr2 + qstr3 + qstr4;
        ui_.lineEdit->setText(qstr2);
      }
    }
  }
  QObject::connect(ui_.pushButton_3, SIGNAL(clicked()),
            this, SLOT(on_pushButton_3_clicked2()) );
//   if(commandcount == 1) {
//       QObject::connect(ui_.pushButton, SIGNAL(clicked()),
//                 this, SLOT(on_pushButton_clicked2()) );
//   }
}
//checkbox function
void coopPlugin::onChecked(bool checked) {
  if(checked == true) {
    ui_.listView_2->setSelectionMode(QAbstractItemView::MultiSelection);
    
    //to cancel highlight, initialize listView_2
    list2 = QStringList();
    model2->setStringList(QStringList{});
    ui_.listView_2->setModel(model2);

    //QString -> char* conversion
    QByteArray bytename = qstrlist.toLocal8Bit();
    /*char **/message1 = bytename.data();
    char *token = NULL;
    char s1[] = " ";
    token = strtok(message1, s1);
    message1 = strtok(NULL, s1);

    listView2Plugin(message1);
    //initialize qstr2
    qstr2 = "";
    checkedcount = 1;
  }
  else if(checked == false) {
    ui_.listView_2->setSelectionMode(QAbstractItemView::SingleSelection);

    list2 = QStringList();
    model2->setStringList(QStringList{});
    ui_.listView_2->setModel(model2);

    //QString -> char* conversion
    QByteArray bytename = qstrlist.toLocal8Bit();
    /*char **/message1 = bytename.data();
    char *token = NULL;
    char s1[] = " ";
    token = strtok(message1, s1);
    message1 = strtok(NULL, s1);

    listView2Plugin(message1);

    qstr2 = "";
    checkedcount = 0;
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
    QObject::disconnect(ui_.listView_2, SIGNAL(clicked(const QModelIndex &)),
                this, SLOT(onClickListItem2(const QModelIndex &))   );
    QObject::disconnect(ui_.pushButton_3, SIGNAL(clicked()),
                this, SLOT(on_pushButton_3_clicked1()) );
    QObject::disconnect(ui_.pushButton_3, SIGNAL(clicked()),
                this, SLOT(on_pushButton_3_clicked2()) );

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

        MPCcount = 0;

        // msg.data[0] = MPCcount;
        // gain_pub.publish(msg);
    }
    else if(strcmp(token, "plot:") == 0) {
        str = strtok(NULL, s1);

        QObject::connect(ui_.pushButton_3, SIGNAL(clicked()),
                this, SLOT(on_pushButton_3_clicked1()) );

        //plot topic list menu setting to listView_2
        listView2Plugin(str);

        MPCcount = 0;

        // msg.data[0] = MPCcount;
        // gain_pub.publish(msg);
    }
    else if(strcmp(token, "MPC") == 0) {
        // if( MPCcount == 0)
        //     MPCcount = 1;
        // else
        //     MPCcount = 0;
        MPCcount = 1;
        
        // msg.data[0] = MPCcount;
        // gain_pub.publish(msg);
    }
    else {
        MPCcount = 0;

        // msg.data[0] = MPCcount;
        // gain_pub.publish(msg);

        QObject::connect(ui_.pushButton, SIGNAL(clicked()),
            this, SLOT(on_activated_clicked())      );
    }
}

//add topics of mesg to list2 
void alphasumstring(int sum, const char *mesg) {
  const char *ram = "/";
  for(int i=0; i<sum; i++) {
    char alpha = 97+i;
    int j;
    for(j=0; j<strlen(mesg); j++) {
      plotstring[j] = mesg[j];
    }
    plotstring[j] = ram[0];
    plotstring[j+1] = alpha;
    plotstring[j+2] = '\0';
    list2 << plotstring;
  }
}
//create detailed topics list of selected main topic in listView_2
void coopPlugin::listView2Plugin(const char* message) {
  //'malloc' function is too often used
  //char *plotstring = (char*)malloc(strlen(message)+2);
  if(strcmp(message, "/targettheta") == 0) {
    list2 = QStringList();
    alphasumstring(6, message);
    model2->setStringList(list2);
    ui_.listView_2->setModel(model2);
  }

  QObject::disconnect(ui_.listView_2, SIGNAL(clicked(const QModelIndex &)),
            this, SLOT(onClickListItem2(const QModelIndex &))   );

  QObject::connect(ui_.listView_2, SIGNAL(clicked(const QModelIndex &)),
            this, SLOT(onClickListItem2(const QModelIndex &))   );
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

    QObject::connect(ui_.kp_plus_2, SIGNAL(clicked()),
            this, SLOT(on_kp_h_plus_clicked())    );
    QObject::connect(ui_.kp_minus_2, SIGNAL(clicked()),
            this, SLOT(on_kp_h_minus_clicked())    );
    QObject::connect(ui_.ki_plus_2, SIGNAL(clicked()),
            this, SLOT(on_ki_h_plus_clicked())    );
    QObject::connect(ui_.ki_minus_2, SIGNAL(clicked()),
            this, SLOT(on_ki_h_minus_clicked())    );
    QObject::connect(ui_.kd_plus_2, SIGNAL(clicked()),
            this, SLOT(on_kd_h_plus_clicked())    );
    QObject::connect(ui_.kd_minus_2, SIGNAL(clicked()),
            this, SLOT(on_kd_h_minus_clicked())    );

    QObject::connect(ui_.kp_plus_3, SIGNAL(clicked()),
            this, SLOT(on_kp_k_plus_clicked())    );
    QObject::connect(ui_.kp_minus_3, SIGNAL(clicked()),
            this, SLOT(on_kp_k_minus_clicked())    );
    QObject::connect(ui_.ki_plus_3, SIGNAL(clicked()),
            this, SLOT(on_ki_k_plus_clicked())    );
    QObject::connect(ui_.ki_minus_3, SIGNAL(clicked()),
            this, SLOT(on_ki_k_minus_clicked())    );
    QObject::connect(ui_.kd_plus_3, SIGNAL(clicked()),
            this, SLOT(on_kd_k_plus_clicked())    );
    QObject::connect(ui_.kd_minus_3, SIGNAL(clicked()),
            this, SLOT(on_kd_k_minus_clicked())    );

    QObject::connect(ui_.kp_plus_4, SIGNAL(clicked()),
            this, SLOT(on_kp_o_plus_clicked())    );
    QObject::connect(ui_.kp_minus_4, SIGNAL(clicked()),
            this, SLOT(on_kp_o_minus_clicked())    );
    QObject::connect(ui_.ki_plus_4, SIGNAL(clicked()),
            this, SLOT(on_ki_o_plus_clicked())    );
    QObject::connect(ui_.ki_minus_4, SIGNAL(clicked()),
            this, SLOT(on_ki_o_minus_clicked())    );
    QObject::connect(ui_.kd_plus_4, SIGNAL(clicked()),
            this, SLOT(on_kd_o_plus_clicked())    );
    QObject::connect(ui_.kd_minus_4, SIGNAL(clicked()),
            this, SLOT(on_kd_o_minus_clicked())    );

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

    QObject::connect(ui_.checkBox, SIGNAL(toggled(bool )),
            this, SLOT(onChecked(bool ))  );
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
         << "command: friction"
         << "command: legtest"
         << "command: gaincontrol"
         << "plot: /targettheta"
         << "MPC gain tunning";

    model->setStringList(list);
    ui_.listView->setModel(model);

    model2 = new QStringListModel(this);

    gain_pub = nh.advertise<std_msgs::Float64MultiArray>("pidgain", 3);
    target_pub = nh.advertise<std_msgs::Float64>("Target", 1);
    reset_pub = nh.advertise<std_msgs::Int32>("reset", 1);
    command_pub = nh.advertise<std_msgs::Int32MultiArray>("control_command", 1);
    legtest_pub = nh.advertise<std_msgs::Float64MultiArray>("/legtest", 3);
    gain_sub = nh.subscribe("init_gain", 1, msgCallbackinitgain);
    MPCgain_pub = nh.advertise<std_msgs::Float64MultiArray>("MPCgain", 3);

    // msg.data.resize(9);
    msg.data.resize(12);
    msgcommand.data.resize(4);
    msgleg.data.resize(2);

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