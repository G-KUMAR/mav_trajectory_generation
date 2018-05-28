/**
 * @file offb_node.cpp
 * @brief Offboard control example node, written with MAVROS version 0.19.x, PX4 Pro Flight
 * Stack and tested in Gazebo SITL
 */

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/ActuatorControl.h>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/Vector3.h>
#include <mavros_msgs/Altitude.h>
#include <mavros_msgs/AttitudeTarget.h>

#include <math.h>

#define PI 3.141
double k_d = 0.1;
double k_p = 0.3; 


mavros_msgs::AttitudeTarget target;
sensor_msgs::Imu curr_state;

static void toEulerAngle(const geometry_msgs::Quaternion& q, double& roll, double& pitch, double& yaw)
{
    // roll (x-axis rotation)
    double sinr = +2.0 * (q.w * q.x + q.y * q.z);
    double cosr = +1.0 - 2.0 * (q.x * q.x + q.y * q.y);
    roll = atan2(sinr, cosr);

    // pitch (y-axis rotation)
    double sinp = +2.0 * (q.w * q.y - q.z * q.x);
    if (fabs(sinp) >= 1)
        pitch = copysign(M_PI / 2, sinp); // use 90 degrees if out of range
    else
        pitch = asin(sinp);

    // yaw (z-axis rotation)
    double siny = +2.0 * (q.w * q.z + q.x * q.y);
    double cosy = +1.0 - 2.0 * (q.y * q.y + q.z * q.z);  
    yaw = atan2(siny, cosy);
}


mavros_msgs::State current_state;
void state_cb(const mavros_msgs::State::ConstPtr& msg){
    current_state = *msg;
}


mavros_msgs::ActuatorControl moment;
double alt;
double prev_roll = 0.0;
double prev_pitch = 0.0;
double prev_yaw= 0.0;
double prev_alt = 0.0;

void pd_controller(double roll, double pitch, double yaw, geometry_msgs::Vector3 omega){

        // moment.controls[0] = k_p * (0-roll) + k_d*(prev_roll-roll)*0.004; //+ k_d * (omega.x);
        // moment.controls[1] = k_p * (0-pitch)+ k_d*(prev_pitch-pitch)*0.004;// + k_d * (omega.y);
        // moment.controls[2] = k_p * (0-yaw)+ k_d*(prev_yaw-yaw)*0.004;// + k_d * (omega.x);
        // moment.controls[3] = 0.2*(3.0 - alt) + k_d*(prev_alt - alt)*0.004 ;

        // moment.group_mix = 0;

        target.body_rate.x = k_p * (roll)  + k_d * (prev_roll-roll)*0.02; //+ k_d * (omega.x);
        target.body_rate.y = k_p * (pitch) + k_d * (prev_pitch-pitch)*0.02;// + k_d * (omega.y);
        target.body_rate.z = k_p * (yaw)   + k_d * (prev_yaw-yaw)*0.02;// + k_d * (omega.x);
        // target.thrust = 1;
        


        prev_roll = roll;
        prev_yaw = yaw;
        prev_pitch = pitch;
        prev_alt = alt;
}

void get_alt(const mavros_msgs::Altitude::ConstPtr& msg){
    alt = msg->local;
}


void generate_control(const sensor_msgs::Imu::ConstPtr& msg){

    double roll, pitch, yaw;
    curr_state = *msg;

    geometry_msgs::Quaternion quat;
    geometry_msgs::Vector3 omega;

    quat = msg->orientation;
    omega = msg -> angular_velocity;
    toEulerAngle(quat, roll, pitch, yaw);

    pd_controller(roll, pitch, yaw, omega);
    // actuator_control_pub.publish(moment);

    //ROS_INFO("Roll : %f Pitch: %f Yaw %f",roll,pitch,yaw);


}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "actuator_ctrl");
    ros::NodeHandle nh;

    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>
            ("mavros/state", 10, state_cb);
    ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>
            ("mavros/cmd/arming");
    ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>
            ("mavros/set_mode");
    ros::Publisher actuator_control_pub = nh.advertise<mavros_msgs::ActuatorControl>
            ("/mavros/actuator_control", 10);
    ros::Subscriber quaternion_sub = nh.subscribe<sensor_msgs::Imu>
            ("mavros/imu/data", 10, generate_control);
    ros::Subscriber altitude_sub = nh.subscribe<mavros_msgs::Altitude>
            ("/mavros/altitude", 10, get_alt);
    ros::Publisher target_attitude_pub = nh.advertise<mavros_msgs::AttitudeTarget>
            ("/mavros/setpoint_raw/attitude", 10);
    ros::Publisher local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>
            ("mavros/setpoint_position/local", 10);


    ros::Rate rate(50.0);
    geometry_msgs::PoseStamped pose;
    
    

    // wait for FCU connection
    while(ros::ok() && !current_state.connected){
        ros::spinOnce();
        rate.sleep();
    }


    mavros_msgs::SetMode offb_set_mode;
    offb_set_mode.request.custom_mode = "OFFBOARD";

    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = true;

    ros::Time last_request = ros::Time::now();

    while(ros::ok()){
        if( current_state.mode != "OFFBOARD" &&
            (ros::Time::now() - last_request > ros::Duration(5.0))){
            if( set_mode_client.call(offb_set_mode) &&
                offb_set_mode.response.mode_sent){
                ROS_INFO("Offboard enabled");
            }
            last_request = ros::Time::now();
        } else {
            if( !current_state.armed &&
                (ros::Time::now() - last_request > ros::Duration(5.0))){
                if( arming_client.call(arm_cmd) &&
                    arm_cmd.response.success){
                    ROS_INFO("Vehicle armed");
                }
                last_request = ros::Time::now();
            }
        }

        //actuator_control_pub.publish(moment);
        pose.pose.position.z = 2;

        local_pos_pub.publish(pose);

  


        // target_attitude_pub.publish(target);

        //ROS_INFO("Moment: Roll : %f Pitch: %f Yaw %f",moment.controls[0],moment.controls[1],moment.controls[3]);
        // ROS_INFO("thrust %f",target.thrust);
        ros::spinOnce();
        rate.sleep();
   
    }
    //the setpoint publishing rate MUST be faster than 2Hz
    /*ros::Rate rate(250.0);

    // wait for FCU connection
    while(ros::ok() && !current_state.connected){
        ros::spinOnce();
        rate.sleep();
    }

    float theta = 0.0;

    mavros_msgs::ActuatorControl moment;


   /* geometry_msgs::PoseStamped pose;
    pose.pose.position.x = 0;
    pose.pose.position.y = 0;
    pose.pose.position.z = 0;

    // Vector
    pose.pose.orientation.x = 0;
    pose.pose.orientation.y = 0;
    pose.pose.orientation.z = 0;

    //Scaler
    pose.pose.orientation.w = 0;

    //send a few setpoints before starting
    for(int i = 250; ros::ok() && i > 0; --i){
        local_pos_pub.publish(pose);
        ros::spinOnce();
        rate.sleep();
    }*/

    /*moment.controls[4] = 0;
    moment.controls[5] = 3;
    moment.controls[6] = 0;
    moment.controls[7] = 0;
    moment.controls[0] = 0;
    moment.controls[1] = 0;
    moment.controls[2] = 0;

    mavros_msgs::SetMode offb_set_mode;
    offb_set_mode.request.custom_mode = "OFFBOARD";

    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = true;

    ros::Time last_request = ros::Time::now();

    while(ros::ok()){
        if( current_state.mode != "OFFBOARD" &&
            (ros::Time::now() - last_request > ros::Duration(5.0))){
            if( set_mode_client.call(offb_set_mode) &&
                offb_set_mode.response.mode_sent){
                ROS_INFO("Offboard enabled");
            }
            last_request = ros::Time::now();
        } else {
            if( !current_state.armed &&
                (ros::Time::now() - last_request > ros::Duration(5.0))){
                if( arming_client.call(arm_cmd) &&
                    arm_cmd.response.success){
                    ROS_INFO("Vehicle armed");
                }
                last_request = ros::Time::now();
            }
        }

        
        moment.controls[0] = -0.00645824;
        moment.controls[1] = -0.186406;
        moment.controls[2] = -0.00037194;
        moment.controls[3] = 4.53587;

        


        //ROS_INFO("Value of w = %f, Value of z = %f" , pose.pose.orientation.w, pose.pose.orientation.z);
        //ROS_INFO("Value of x = %f, Value of y = %f" , pose.pose.position.x, pose.pose.position.y);

        //local_pos_pub.publish(pose);
        actuator_control_pub.publish(moment);

        
    }*/

    // ros::spin();
    // rate.sleep();

    return 0;
}

