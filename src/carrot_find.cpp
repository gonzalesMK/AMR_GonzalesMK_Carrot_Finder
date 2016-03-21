#include <iostream>
#include <ros/ros.h>
#include <geometry_msgs/Quaternion.h>
#include <nav_msgs/Odometry.h>
#include <tf/tf.h>
#include <geometry_msgs/Twist.h>
#include <math.h>
#define delta 0.2
ros::Publisher pvel;

double x,f,xi,xf,xc,y,yi,yf,yc,teta,psi,beta,R,D,omega,domega,teta2; 

geometry_msgs::Twist vel;

double norma_ang( double ang )
{   if  ( ang >  M_PI) {
        ang -= 2 * M_PI; 
    }
    else if ( ang < - M_PI){
        ang += 2 * M_PI;
    }
    return ang;
}
void go2point (double Xc, double Yc, double X, double Y){
        // ang. do carro
    omega = atan2((Yc - Y) , ( Xc - X));
    domega= norma_ang(omega - psi);
    if ( fabs(domega) > 0.2 ){
        vel.angular.z = domega * 3;
        vel.linear.x = (M_PI - fabs(domega))*0.4 ;
      } else {
        vel.angular.z = domega*2 ;
        vel.linear.x = 3;
     }
    pvel.publish(vel);  
}


void odomcb(const nav_msgs::Odometry::ConstPtr &pos)
{   geometry_msgs::Quaternion qt;
    qt = pos -> pose.pose.orientation; //Cria um quaternion que pega as inf. do carro
    psi = tf::getYaw(qt); 
    //localização do carro
    x = pos->pose.pose.position.x;          
    y = pos->pose.pose.position.y;
    //determinação dos angulos
    teta2= atan2((y- yi),(x- xi));
    teta= atan2( (yf - yi) , (xf - xi) );        //angulo do endpoint em relação ao start
    beta = teta - teta2;         //ang. do delta
    //localização da cenora
    D = sqrt( pow((xi-xf),2) + pow((yi-yf),2)); // distancia do carro até o startpoint
    R = fabs(D*cos(beta));          //R é a distancia do startpoint até a perpendicular 
    xc = (R + delta)*cos(teta) + xi;
    yc = (R + delta)*sin(teta)  + yi; 
    go2point(xc,yc,x,y);
    std::cout <<"Xc: "<<xc <<"Yc::" << yc << "beta:" << beta<< std::endl;
}

int main( int argc , char **argv )
{   ros::init(argc, argv, "carrot_finder");
    ros::NodeHandle node;
    //localização startpoint
    xi =  4;
    yi= 1.5;
    //localização endpoint   
    xf = -3;
    yf = 1.5;

    ros::Subscriber s_odom = node.subscribe("/vrep/vehicle/odometry", 1, odomcb);
    pvel = node.advertise<geometry_msgs::Twist>("/kine", 1);
 
    ros::spin();   
    
}