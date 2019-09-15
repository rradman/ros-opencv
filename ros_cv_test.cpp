#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include "opencv2/imgproc.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/core/mat.hpp"
#include <iostream>
#include <fstream>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <math.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>
#include <angles/angles.h>

static const std::string IME_PROZORA = "Sličica i kružić";

image_transport::Publisher objav;
using namespace std;
using namespace cv;

int mode_flag = 0;
int i = 0;
double kruzna_brzina = 0;
double linearna_brzina = 0;
int razlika;
int zbroj_piksela;
int desno = 0;
int lijevo = 0;
double robot_pozicija_x = 0;
double robot_pozicija_y = 0;
double robot_yaw = 0;
double quat_x, quat_y, quat_z, quat_w;
double roll, pitch, yaw;
int flag_rotiraj = 0;

void slikaPovPoziv(const sensor_msgs::ImageConstPtr& msg) {

    cv_bridge::CvImagePtr cv_ptr;
    try {
      cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception& e) {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }

    cv::inRange(cv_ptr->image, cv::Scalar(0,0,100), cv::Scalar(20,20,255),cv_ptr->image);
    cv::Mat locations;
    cv::findNonZero(cv_ptr->image, locations);
    desno = 0;
    lijevo = 0;
    for(i=0;i<locations.total();i++){

        Point pnt = locations.at<Point>(i);
        if(pnt.x < 320/2){
            desno = desno +1;
        }
        if(pnt.x > 320/2){
            lijevo = lijevo +1;
        }
    } 
    zbroj_piksela = 0;
    zbroj_piksela = lijevo + desno;
    razlika = abs(desno-lijevo);
    //ROS_INFO("ZBROJ PIKSELA: %d", zbroj_piksela);


    cv::imshow(IME_PROZORA, cv_ptr->image);
    cv::waitKey(3);

    objav.publish(cv_ptr->toImageMsg());
}

void chatterCallback(const nav_msgs::Odometry::ConstPtr& msg){
    quat_x = msg->pose.pose.orientation.x;
    quat_y = msg->pose.pose.orientation.y;
    quat_z = msg->pose.pose.orientation.z;
    quat_w = msg->pose.pose.orientation.w;

    //pretvorba kuta u radijane
    tf::Quaternion q(quat_x, quat_y, quat_z, quat_w);
    tf::Matrix3x3 m(q);

    m.getRPY(roll, pitch, yaw);

    robot_pozicija_x = msg->pose.pose.position.x;
    robot_pozicija_y = msg->pose.pose.position.y;
    robot_yaw = yaw;
    if(robot_yaw < 0) robot_yaw = robot_yaw + 2*M_PI;
    //ROS_INFO("POZICIJA-X = %f,  POZICIJA-Y = %f, KUT_DI_GLEDA = %f",robot_pozicija_x, robot_pozicija_y, robot_yaw);
}

void rotiraj(){
    // dok trazi loptu
    if(zbroj_piksela == 0){
        //odredivanje u kojem ce se smjeru rotirati po uzoru na trenutni polozaj(odom) i kut gledanja(yaw)
        if(flag_rotiraj == 0){
            if(robot_pozicija_x > 0 && robot_pozicija_y > 0){
                if(robot_yaw > M_PI/4) kruzna_brzina = 0.5;
                    else kruzna_brzina = -0.5;
            }
            if(robot_pozicija_x > 0 && robot_pozicija_y < 0){
                if(robot_yaw > 7*(M_PI/4)) kruzna_brzina = 0.5;
                    else kruzna_brzina = -0.5;
            }
            if(robot_pozicija_x < 0 && robot_pozicija_y > 0){
                if(robot_yaw > (3*M_PI)/4) kruzna_brzina = 0.5;
                    else kruzna_brzina = -0.5;
            }
            if(robot_pozicija_x < 0 && robot_pozicija_y < 0){
                if(robot_yaw > (5*M_PI)/4) kruzna_brzina = 0.5;
                    else kruzna_brzina = -0.5;
            } 
            flag_rotiraj = 1;  
        }     
    }

    //ako je pronasao loptu
    if(zbroj_piksela > 0){
        linearna_brzina = 0.25;
        kruzna_brzina = 0;
        mode_flag = 1;
        flag_rotiraj = 0;
    }  
}

void vozi_ravno(){

    if(zbroj_piksela == 0){
        linearna_brzina = 0;
        mode_flag = 0;
        return;
    }
    if(zbroj_piksela/2 < desno*0.5 || zbroj_piksela/2 < lijevo*0.5){       
        //ROS_INFO("APSOLUTNA RAZLIKA = %d", razlika);
        if(desno > lijevo){
            //ROS_INFO("SKRECEM LIJEVO");
            kruzna_brzina = 0.5;
        }  
        if(desno < lijevo){
            //ROS_INFO("SKRECEM DESNO");
            kruzna_brzina = -0.5;
        }
    }
    else{
        //ROS_INFO("NACILJAO SAM !");
        kruzna_brzina = 0;
    }     
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "image_converter");
    ros::NodeHandle nh_;
    ros::NodeHandle n;
    
    cv::namedWindow(IME_PROZORA);
    
    ros::Publisher pub = n.advertise<geometry_msgs::Twist>("cmd_vel", 10);
    ros::Subscriber subRobotPosition = n.subscribe("odom", 1000, chatterCallback); 
    image_transport::ImageTransport it_(nh_);
    image_transport::Subscriber pret = it_.subscribe("camera/image", 1, &slikaPovPoziv);
    objav = it_.advertise("processed/image", 1);

    ros::Rate loop_rate(20);
    while(ros::ok()){
        ros::spinOnce();
        loop_rate.sleep();

        if(mode_flag == 0){
            //rotiraj i trazi kuglu
            rotiraj();
        }
        if(mode_flag == 1){
            //idi ravno i nišani
            vozi_ravno();
        }

        geometry_msgs::Twist msg;
        msg.angular.z = kruzna_brzina;
        msg.linear.x = linearna_brzina;
        pub.publish(msg);  
    }
    ros::spin();    
    return 0;
}
