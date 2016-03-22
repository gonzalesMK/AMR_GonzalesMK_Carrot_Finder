#include <iostream>
#include <ros/ros.h>
#include <geometry_msgs/Quaternion.h>
#include <nav_msgs/Odometry.h>
#include <tf/tf.h>
#include <geometry_msgs/Twist.h>
#include <math.h>
#define delta 0.5
ros::Publisher pvel;

double x,f,xi,xf,xc,y,yi,yf,yc,teta,psi,beta,R,D,teta2,dist; 
int i;
struct Ponto {
    double x;
    double y;
};
struct Ponto ponto[11];

double norma_ang( double ang )
{   if  ( ang >  M_PI) {
        ang -= 2 * M_PI; 
    }
    else if ( ang < - M_PI){
        ang += 2 * M_PI;
    }
    return ang;
}
void go2point (double Xc, double Yc, double X, double Y,double ang){
    double omega, domega;   
    geometry_msgs::Twist vel;    
    omega = atan2((Yc - Y) , ( Xc - X)); //Angulo do carro em relação ao carrot
    domega= norma_ang(omega - psi);
    
    if ( fabs(domega) > 0.2 ){
        vel.angular.z = domega * 9;
        vel.linear.x = (M_PI - fabs(domega))*2 + 3 ;
      } else {
        vel.angular.z = domega*10 ;
        vel.linear.x = 10;
     }
    std::cout << "vl: " << vel.linear.x << "va: "<< vel.angular.z << "i+1:" << i+1 << std::endl;
    pvel.publish(vel);  
}


void odomcb(const nav_msgs::Odometry::ConstPtr &pos)
{   geometry_msgs::Quaternion qt;
    qt = pos -> pose.pose.orientation;
    psi = tf::getYaw(qt); 
    //localização do carro
    x = pos->pose.pose.position.x;          
    y = pos->pose.pose.position.y;
    //determinação dos angulos
    dist = sqrt( pow( x - xf, 2) + pow(y - yf, 2));
    if ( dist< 1 ) {
        i++;        
        xi = ponto[i].x;
        yi= ponto[i].y;
        if ( i == 10) {
            i = -1;
        }
        xf= ponto[i+1].x;
        yf= ponto[i+1].y;
    }
    teta2= atan2((y- yi),(x- xi));
    teta= atan2( (yf - yi) , (xf - xi) );        //angulo do endpoint em relação ao start
    beta = teta - teta2;         //ang. do delta
    //localização da cenora
    D = sqrt( pow((xi-xf),2) + pow((yi-yf),2)); // distancia do carro até o startpoint
    R = fabs(D*cos(beta));          //R é a distancia do startpoint até a perpendicular 
    xc = (R + delta)*cos(teta) + xi;
    yc = (R + delta)*sin(teta)  + yi; 
    go2point(xc,yc,x,y,psi);
    
}

int main( int argc , char **argv )
{   ros::init(argc, argv, "carrot_finder");
    ros::NodeHandle node;
    i = 0;
    ponto[0].x= -12.95;
    ponto[0].y= -0.225;
    ponto[1].x= 10;
    ponto[1].y=+0.175;
    ponto[2].x= 11.1;
    ponto[2].y= -9.5;
    ponto[3].x= 15.3;
    ponto[3].y= -15;
    ponto[4].x= 8;
    ponto[4].y = -19.9;
    ponto[5].x= -6.625;
    ponto[5].y= -19.65;
    ponto[6].x = -14.5;
    ponto[6].y= -13.6;
    ponto[7].x = -13.2;
    ponto[7].y= -12.2;
    ponto[8].x = -1.3;
    ponto[8].y= -12.8;
    ponto[9].x = -1.18;
    ponto[9].y= -10.5;
    ponto[10].x= -13.1;
    ponto[10].y= -10.9;
    
    
    dist = 100;
    xi = 3.55;
    yi = 0.125;
    xf= 11;
    yf = 0.175;
    ros::Subscriber s_odom = node.subscribe("/vrep/vehicle/odometry", 1, odomcb);
    pvel = node.advertise<geometry_msgs::Twist>("/carrot_velocity", 1);
 
    ros::spin();   
    
}