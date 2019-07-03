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
#include <std_msgs/String.h>
#include <control_msgs/GripperCommandActionGoal.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <stdio.h>

#define NBJOINTS 6


//sensor_msgs::JointState CurArmState;
ros::Publisher ArmVelCtrlPub;
bool armDeadmans = false;
bool HadCallback = false;
int JointIndex = 0;
static double FinessePrecision = 0.2;
bool buttonsAlreadyPressed[30] = {false};
double HandState = 0.1;
double AxisDir = 0.0;
static double ThumbDeadzone = 0.3;

// trajectory_msgs::JointTrajectory MyTrajectory;
std::string JointNames[NBJOINTS] = {
        "ca_joint1", "ca_joint2", "ca_joint3", "ca_joint4", "ca_joint5", "ca_joint6"};

/* void ArmStateCB(sensor_msgs::JointState State) {
    CurArmState = State;
} */

// COMMENTE CAR PAS DE TRAJECTORY POUR CAPRA
/* void ToggleArmMode() {
    // switch arm to trajectory mode or velocity mode
    controller_manager_msgs::SwitchController msg2;
    if (ArmMode) {
        msg2.request.start_controllers.push_back("capra_arm_trajectory_controller");
        msg2.request.stop_controllers.push_back("capra_arm_velocity_controller");
    } else {
        msg2.request.start_controllers.push_back("capra_arm_velocity_controller");
        msg2.request.stop_controllers.push_back("capra_arm_trajectory_controller");
    }
    msg2.request.strictness = 50;
    Switch.waitForExistence();
    Switch.call(msg2);
    ArmMode = !ArmMode;
} */

/* void HandCtrl(sensor_msgs::JoyPtr joy) {
    if (joy->buttons[0] && !buttonsAlreadyPressed[0]) {
        if (HandState == 0)
            HandState = 0.1;
        else
            HandState = 0;
        control_msgs::GripperCommandActionGoal msg;
        msg.goal.command.position = HandState;
        HandCtrlPub.publish(msg);
    }
} */

// TODO
/* void ArmCtrl(sensor_msgs::JoyPtr joy) {
    if (ArmMode) {
        std_msgs::Float64MultiArray VelMsg;
        for (int i = 0; i < NBJOINTS; i++) {
            double vel = 0;
            if (i == JointIndex)
                vel = ((joy->axes[2]) - (joy->axes[5])) * -0.5;
            VelMsg.data.push_back(vel);
        }
        ArmVelCtrlPub.publish(VelMsg);
        if (joy->buttons[7] && !buttonsAlreadyPressed[7]) {
            JointIndex++;
            if (JointIndex >= NBJOINTS) JointIndex = 0;
        }
        if (joy->buttons[6] && !buttonsAlreadyPressed[6]) {
            JointIndex--;
            if (JointIndex < 0) JointIndex = NBJOINTS - 1;
        }
    }
} */

void ArmCtrl(sensor_msgs::JoyPtr joy) {

    //Controle du joint
    std_msgs::Float64MultiArray VelMsg;
    for (int i = 0; i < NBJOINTS; i++) {
        double vel = 0;
        if (i == JointIndex)
            // On verifie la direction demandee
            if (joy->axes[0] > ThumbDeadzone){
                AxisDir = 1.0;
            } else if (joy->axes[0] < -ThumbDeadzone){
                AxisDir = -1.0;
            } else {
                AxisDir = 0.0;
            }
            // Si B est maintenu la velocite est plus petite
            if (joy->buttons[1]){
                vel = (joy->axes[5] + 1.0 * AxisDir) * 0.5 * FinessePrecision;
            } else {
                vel = (joy->axes[5] + 1.0 * AxisDir) * 0.5;
            }
        VelMsg.data.push_back(vel);
    }
    ArmVelCtrlPub.publish(VelMsg);
    
    // Changement de joint
    if (joy->buttons[4] && !buttonsAlreadyPressed[4]) {
        JointIndex++;
        if (JointIndex >= NBJOINTS) JointIndex = 0;
        ROS_INFO("Switching to next controlled joint.");
    }
    if (joy->buttons[5] && !buttonsAlreadyPressed[5]) {
        JointIndex--;
        if (JointIndex < 0) JointIndex = NBJOINTS - 1;
        ROS_INFO("Switching to previous controlled joint.");
    }
}
void StopArm() {
    std_msgs::Float64MultiArray VelMsg;
    for (int i = 0; i < NBJOINTS; i++) {
        double vel = 0;
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
    // Si le trigger gauche est a moitie on active le deadamns
    if ((joy->axes[2] > -0.5 && joy->axes[2] < 0.5)){
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
    ArmVelCtrlPub = nh.advertise<std_msgs::Float64MultiArray>("sara_arm_velocity_controller/command", 1);
/*     BaseVelCtrlPub = nh.advertise<geometry_msgs::Twist>("cmd_vel", 1); */
   /*  HandCtrlPub = nh.advertise<control_msgs::GripperCommandActionGoal>(
            "/sara_gripper_action_controller/gripper_cmd/goal", 1); */


    ROS_INFO("starting subscribers");
    // Subscribers
    // ros::Subscriber ArmStateSub = nh.subscribe("joint_states", 1, &ArmStateCB);
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


/*     do {
        List.call(listmsg);
        unsigned long Length1 = listmsg.response.controller.size();
        for (int i = 0; i < Length1; i++) {
            if (listmsg.response.controller[i].name == "capra_arm_trajectory_controller") {
                MyTrajectory.joint_names = listmsg.response.controller[i].claimed_resources[0].resources;
            }
        }
        if (MyTrajectory.joint_names.size() == 0) {
            sleep(1);
            continue;
        } else break;

    } while (true); */

    ROS_INFO("loading controllers");
    // Load controllers
    controller_manager_msgs::LoadController msg;
    Load.waitForExistence();
    msg.request.name = "capra_arm_velocity_controller";
    Load.call(msg);


    // start the loop
    // ROS_INFO("Teleop_is_off. Press both triggers to turn it on.");
    //ros::spin();

    ros::Rate r(30); // Tourne a 30Hz
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
