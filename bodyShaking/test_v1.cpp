#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <tf/tf.h>
#include <opencv2/opencv.hpp>
#include <iomanip>
#include <boost/thread/mutex.hpp>
#include <sensor_msgs/LaserScan.h>
#include <float.h>
#include <stdlib.h>
#include <time.h>
#include <math.h>

using namespace cv;
using namespace std;

#define toRadian(degree)	((degree) * (M_PI / 180.))
#define toDegree(radian)	((radian) * (180. / M_PI))
#define ROTDEGREE toRadian(30)

boost::mutex mutex;
nav_msgs::Odometry g_odom;

void odomMsgCallback(const nav_msgs::Odometry &msg)
{
	// receive a '/odom' message with the mutex
	mutex.lock(); {
		g_odom = msg;
	} mutex.unlock();
}

tf::Transform getCurrentTransformation(void)
{
	tf::Transform transformation;
	nav_msgs::Odometry odom;

	mutex.lock(); {
		odom = g_odom;
	} mutex.unlock();

	printf("x : %lf, y : %lf, z : %lf\n", odom.pose.pose.position.x, odom.pose.pose.position.y, odom.pose.pose.position.z);
	printf("x : %lf, y : %lf, z : %lf\n", odom.pose.pose.orientation.x, odom.pose.pose.orientation.y, odom.pose.pose.orientation.z);

	// Save position
	transformation.setOrigin(tf::Vector3(odom.pose.pose.position.x, odom.pose.pose.position.y, odom.pose.pose.position.z));

	// Save rotation angle
	transformation.setRotation(tf::Quaternion(odom.pose.pose.orientation.x, odom.pose.pose.orientation.y, odom.pose.pose.orientation.z, odom.pose.pose.orientation.w));

	return transformation;
}

bool doRotation(ros::Publisher &pubTeleop, tf::Transform &initialTransformation, double dRotation, double dRotationSpeed)
{
	//the command will be to turn at 'rotationSpeed' rad/s
	geometry_msgs::Twist baseCmd;
	baseCmd.linear.x = 0.0;
	baseCmd.linear.y = 0.0;

	if(dRotation < 0.) {
		baseCmd.angular.z = -dRotationSpeed;
	} else {
		baseCmd.angular.z = dRotationSpeed;
	}

	// Get odometry messages of current position while moving
	bool bDone = false;
	ros::Rate loopRate(1000.0);


	while(ros::ok() && !bDone) {
		// Get callback messages
		ros::spinOnce();

		// get current transformation
		tf::Transform currentTransformation = getCurrentTransformation();
		
		//see how far we've traveled
		tf::Transform relativeTransformation = initialTransformation.inverse() * currentTransformation;
		tf::Quaternion rotationQuat = relativeTransformation.getRotation();

		double dAngleTurned = atan2((2 * rotationQuat[2] * rotationQuat[3]) , (1-(2 * (rotationQuat[2] * rotationQuat[2]) ) ));
		printf("rotationQuat[0] : %lf, rotationQuat[1] : %lf\n", rotationQuat[0], rotationQuat[1]);
		printf("rotationQuat[2] : %lf, rotationQuat[3] : %lf\n", rotationQuat[2], rotationQuat[3]);
		printf("dAngleTurned : %lf, dRotation : %lf\n", dAngleTurned, dRotation);

		// Check termination condition
		if( fabs(dAngleTurned) > fabs(dRotation) || (dRotation == 0)) 
		{
			printf("if!! plz stop!!!!!\n");
			bDone = true;
			break;
		} else {
			printf("else!! keep rotate!!!\n");
			//send the drive command
			pubTeleop.publish(baseCmd);
			// sleep!
			loopRate.sleep();
		}

	}

	// Initialization
	baseCmd.linear.x = 0.0;
	baseCmd.linear.y = 0.0;
	baseCmd.angular.z = 0.0;
	pubTeleop.publish(baseCmd);

	return bDone;
}


int main(int argc, char **argv){

	ros::init(argc, argv, "test_v1");

	ros::NodeHandle nhp;
	ros::Publisher pubTeleop = nhp.advertise<geometry_msgs::Twist>("/cmd_vel", 100);

	ros::NodeHandle nh;	
	ros::Subscriber subOdom = nh.subscribe("/odom", 100, &odomMsgCallback);

	nav_msgs::Odometry odom;

	// Move turtlebot straight forward.
	geometry_msgs::Twist baseCmd;
	baseCmd.linear.x = 0;
	baseCmd.linear.y = 0;
	baseCmd.angular.z = 0;

	bool shake_flag = true;

	while(ros::ok()) {
		/* 1. Call callback methods: odomMsgCallback( ), scanMsgCallback( ) */        
		ros::spinOnce();


		/* 2. Get current odom-info of turtlebot */
		// 2-1. Get current global odom-info of turtlebot.
		mutex.lock(); {
			odom = g_odom;
		} mutex.unlock();

		pubTeleop.publish(baseCmd);

		double deg = 0;
		if(shake_flag){
			printf("####### shaking!! #######");

                        deg = ROTDEGREE;

			tf::Transform currentTransformation = getCurrentTransformation();
			doRotation(pubTeleop, currentTransformation, ROTDEGREE, 0.25);
			shake_flag = false;

			baseCmd.linear.x = 0;
			baseCmd.linear.y = 0;
			baseCmd.angular.z = 0;
			pubTeleop.publish(baseCmd);
		}

		/* 5. Wait for key to termination */
		int nKey = waitKey(30) % 255;
		if(nKey == 27) {
			break;
		} 

	}

	return 0;

}
