//
// Created by philippe on 13/06/17.
//

#include <ros/ros.h>
#include <std_msgs/builtin_uint8.h>
#include <sensor_msgs/Joy.h>
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Float64MultiArray.h>
#include <controller_manager/controller_manager.h>
#include <std_msgs/Float64.h>
#include <control_msgs/GripperCommandActionGoal.h>
#include <stdio.h>
#include <algorithm>

#define NBJOINTS 6


sensor_msgs::JointState CurArmState;
ros::Publisher ArmVelCtrlPub;
ros::Publisher HandCtrlPub;
bool armDeadmans = false;
bool HadCallback = false;
int JointIndex = 0;
static double FinessePrecision[6] = {0.1, 0.05, 0.06, 0.2, 0.15, 0.2}; //Max speed in rad/s
static double Precision[6] = {0.6, 0.35, 0.45, 1.3, 1.0, 1.3}; //Max speed in rad/s
static double Acceleration[6] = {0.3, 0.05, 0.08, 2.0, 0.5, 2.0}; //Acceleration in rad/s2
double LastVel[6] = {0.0}; //Last velocity for acceleration delta calc
bool buttonsAlreadyPressed[30] = {false};
double HandState = 0.1;
double Rate = 15; // Refresh Rate

void ArmStateCB(sensor_msgs::JointState State) {
    CurArmState = State;
}

void HandCtrl(sensor_msgs::JoyPtr joy) {
/*     if (joy->buttons[0] && !buttonsAlreadyPressed[0]) {
        if (HandState == 0)
            HandState = 0.1;
        else
            HandState = 0;
        control_msgs::GripperCommandActionGoal msg;
        msg.goal.command.position = HandState;
        HandCtrlPub.publish(msg);
    } */
}

void ArmCtrl(sensor_msgs::JoyPtr joy) {

    //Controle du joint
    std_msgs::Float64MultiArray VelMsg;
    for (int i = 0; i < NBJOINTS; i++) {
        double vel = 0;
        double delta = 0;
        if (i == JointIndex){ 
            if (joy->buttons[1]){ //Calcul le delta entre ce qui est demande et la derniere vitesse
                delta = (joy->axes[0]) * FinessePrecision[i] - LastVel[i];
            } else {
                delta = (joy->axes[0]) * Precision[i] - LastVel[i];
            }
            // Clamp le delta avec lacceleration
            delta = std::max(std::min(delta, Acceleration[i]/Rate), -Acceleration[i]/Rate);
            vel = LastVel[i]+ delta;
        }
        LastVel[i] = vel;
        VelMsg.data.push_back(vel);
    }
    ArmVelCtrlPub.publish(VelMsg);
}

void JointChange(sensor_msgs::JoyPtr joy) {
    // Changement de joint
    if (joy->buttons[5] && !buttonsAlreadyPressed[5]) {
        JointIndex++;
        if (JointIndex > NBJOINTS) JointIndex = NBJOINTS;
        ROS_INFO("Switching to next controlled joint.");
    }
    if (joy->buttons[4] && !buttonsAlreadyPressed[4]) {
        JointIndex--;
        if (JointIndex < 0) JointIndex = 0;
        ROS_INFO("Switching to previous controlled joint.");
    }
}
void StopArm() {
    std_msgs::Float64MultiArray VelMsg;
    for (int i = 0; i < NBJOINTS; i++) {
        double vel = 0;
        LastVel[i] = vel;
        VelMsg.data.push_back(vel);
    }
    ArmVelCtrlPub.publish(VelMsg);
}

void JoyCB(sensor_msgs::JoyPtr joy) {
    // Si la deadmans est true on peut controller le bras
    HadCallback = true;
    if (armDeadmans) {
        ArmCtrl(joy);
    } else {
        StopArm();
    }
    JointChange(joy);

    if (JointIndex == NBJOINTS){
        HandCtrl(joy);
    }
    // Si le trigger gauche est a moitie on active le deadamns
    if ((joy->axes[2] > -0.95 && joy->axes[2] < 0.95)){
        armDeadmans = true;
    } else {
        armDeadmans = false;
    }
    for (int i = 0; i < joy->buttons.size(); i++) {
        buttonsAlreadyPressed[i] = (bool) joy->buttons[i];
    }
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "sara_teleop");
    ros::NodeHandle nh;

    // Waiting for services
    ros::service::waitForService("controller_manager/load_controller");
    ros::service::waitForService("controller_manager/list_controllers");
    ros::service::waitForService("controller_manager/switch_controller");


    ROS_INFO("starting publishers");
    // Publishers
    ArmVelCtrlPub = nh.advertise<std_msgs::Float64MultiArray>("capra_arm_velocity_controller/command", 1);
    HandCtrlPub = nh.advertise<control_msgs::GripperCommandActionGoal>(
            "/sara_gripper_action_controller/gripper_cmd/goal", 1);


    ROS_INFO("starting subscribers");
    // Subscribers
    ros::Subscriber ArmStateSub = nh.subscribe("joint_states", 1, &ArmStateCB);
    ros::Subscriber JoySub = nh.subscribe("joy", 1, &JoyCB);

    ROS_INFO("starting service clients");
    // controller services
    ros::ServiceClient Load = nh.serviceClient<controller_manager_msgs::LoadController>(
            "controller_manager/load_controller");
/*     Switch = nh.serviceClient<controller_manager_msgs::SwitchController>("controller_manager/switch_controller"); */
    ros::ServiceClient List = nh.serviceClient<controller_manager_msgs::ListControllers>(
            "controller_manager/list_controllers");


    ROS_INFO("getting controller");
    List.waitForExistence();
    controller_manager_msgs::ListControllers listmsg;

    ROS_INFO("loading controllers");
    // Load controllers
/*     controller_manager_msgs::LoadController msg;
    Load.waitForExistence();
    msg.request.name = "capra_arm_velocity_controller";
    Load.call(msg); */


    // start the loop
    // ROS_INFO("Teleop_is_off. Press both triggers to turn it on.");

    ros::Rate r(Rate); // Tourne a 15Hz
    while(ros::ok()){
        HadCallback = false;
        ros::spinOnce();
        if (!HadCallback){
            StopArm();
            ROS_INFO("Stopping arm because no message on topic.");
        }
        r.sleep();
    }

    return 0;
}
