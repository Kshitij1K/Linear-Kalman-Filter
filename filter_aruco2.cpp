#include <bits/stdc++.h>
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/PoseStamped.h>
#include <Eigen/Dense>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <math.h>
#define t 0.1

using namespace Eigen;

MatrixXf Fk(6,6);
MatrixXf Fktrn(6,6);
MatrixXf Bk(6,3);
MatrixXf Qk(6,6);
MatrixXf Rk(3,3);
MatrixXf Iden(6,6);
MatrixXf Hk(3,6);
MatrixXf Hktrn(6,3);

MatrixXf Imagelinpos(3,1),Imulinacc(3,1);
MatrixXf belx(6,1),belpredictx(6,1);
MatrixXf covarx(6,6),covarpredictx(6,6);
MatrixXf belangx(4,1),covarangx(4,1);
MatrixXf kalmangain(6,3);
MatrixXf g(3,1);
MatrixXf imutemp(3,1);  
MatrixXf rotmat(3,3);
geometry_msgs::PoseStamped output;
Quaternionf q;
void ImageCallback(const geometry_msgs::PoseStamped::ConstPtr& imgmsg){
    float xx,yy,zz;
    xx=imgmsg->pose.position.x;
    yy=imgmsg->pose.position.y;
    zz=imgmsg->pose.position.z;
    Imagelinpos << xx,yy,zz;

    output.header.stamp=imgmsg->header.stamp;
    output.header.seq=imgmsg->header.seq;
    output.header.frame_id=imgmsg->header.frame_id;
    output.pose.orientation.x=imgmsg->pose.orientation.x;
    output.pose.orientation.y=imgmsg->pose.orientation.y;
    output.pose.orientation.z=imgmsg->pose.orientation.z;
    output.pose.orientation.w=imgmsg->pose.orientation.w;
    
    //Calculating new belief of x
    belx=belpredictx+kalmangain*(Imagelinpos-(Hk*belpredictx));

    //Calculating new covariance of x
    covarx=covarpredictx-((kalmangain*Hk)*covarpredictx);
    /*std::cout << "belx" << std::endl << belx << std::endl << std::endl;*/
}

void ImuCallback(const sensor_msgs::Imu::ConstPtr& msg){
    float Ax,Ay,Az;
    float Angx,Angy,Angz;    
    Quaternionf q,qrev;
    q = Quaternionf(msg->orientation.w, msg->orientation.x, msg->orientation.y, msg->orientation.z);
    /*qrev=Quaternionf.Inverse(q);*/
    MatrixXf rotmat = q.toRotationMatrix();
    MatrixXf rotmat2=rotmat.inverse();

    Ax=msg->linear_acceleration.x;
    Ay=msg->linear_acceleration.y;
    Az=msg->linear_acceleration.z;
    imutemp << Ax,Ay,Az;
    g << 0,0,-9.8;
    Imulinacc =imutemp+rotmat2*g; 

    //Code for predicting belief of x
    belpredictx=Fk*belx+Bk*Imulinacc;

    //Code for predicting covariance of x
    covarpredictx=(Fk*covarx)*Fktrn+Qk;

    //Calculating kalman gain
    MatrixXf temp2,temp3;
    temp2=(Hk*covarpredictx)*Hktrn+Rk;
    temp3=temp2.inverse();
    kalmangain=(covarpredictx*Hktrn)*temp3;

    

    output.pose.position.x=belx(0,0);
    output.pose.position.y=belx(1,0);
    output.pose.position.z=belx(2,0); 
}

int main(int argc, char** argv){
  ros::init(argc,argv,"ImageFused");
  ros::NodeHandle nh;     
  ros::Subscriber  Image=nh.subscribe("/package/position",100,ImageCallback); /*/aruco_single/pose*/
  ros::Subscriber Imu=nh.subscribe("/icarus/imu",100,ImuCallback);
  ros::Publisher fused = nh.advertise<geometry_msgs::PoseStamped>("/icarus/command/pose", 1000);     
    Hk << 1,0,0,0,0,0,
          0,1,0,0,0,0,
          0,0,1,0,0,0;

    belx << 0,
            0,
            6,
            0,
            0,
            0;   

    Hktrn << 1,0,0,
             0,1,0,
             0,0,1,
             0,0,0,
             0,0,0,
             0,0,0;

    covarx << 1,0,0,0,0,0,
              0,1,0,0,0,0,
              0,0,1,0,0,0,
              0,0,0,1,0,0,
              0,0,0,0,1,0,
              0,0,0,0,0,1;

    Fktrn << 1,0,0,0,0,0,
             0,1,0,0,0,0,
             0,0,1,0,0,0,
             t,0,0,1,0,0,
             0,t,0,0,1,0,
             0,0,t,0,0,1;  

    Bk << t*t/2,0,0,
          0,t*t/2,0,
          0,0,t*t/2,
          t,0,0,
          0,t,0,
          0,0,t;

    Fk << 1,0,0,t,0,0,
          0,1,0,0,t,0,
          0,0,1,0,0,t,
          0,0,0,1,0,0,
          0,0,0,0,1,0,
          0,0,0,0,0,1;

    Qk << 0.4,0,0,0,0,0,
          0,0.4,0,0,0,0,
          0,0,0.4,0,0,0,
          0,0,0,0.4,0,0,
          0,0,0,0,0.4,0,
          0,0,0,0,0,0.4;

    Rk << 0.2,0,0,
          0,0.2,0,  
          0,0,0.2;

    ros::Rate loop_rate(100);
    while(ros::ok()){
        fused.publish(output);
        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}       
