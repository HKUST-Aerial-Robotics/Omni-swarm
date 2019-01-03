#include <iostream>
#include <rotor_position_control.h>
#include "ros/ros.h"
#include "stdio.h"
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/QuaternionStamped.h>
#include <nav_msgs/Odometry.h>
#include <thread>
#include <ctime>
#include <time.h>
#include <stdlib.h>
#include <swarm_msgs/swarm_pos_control_state.h>
#include <swarm_msgs/swarm_pos_ctrl_cmd.h>
#include <sensor_msgs/Joy.h>
#include <swarm_util.h>
#include <sensor_msgs/Imu.h>


class SwarmPosControl {
    ros::NodeHandle & nh;
    
    ros::Subscriber odom_sub;
    ros::Subscriber swarm_pos_cmd_sub;
    ros::Subscriber fc_att_sub;

    RotorPositionControl * pos_ctrl = nullptr;

    swarm_msgs::swarm_pos_control_state state;

    Eigen::Vector3d pos_sp = Eigen::Vector3d(0, 0, 0);
    Eigen::Vector3d vel_sp = Eigen::Vector3d(0, 0, 0);

    ros::Publisher state_pub;
    ros::Publisher control_pub;

    std::string log_path;

    double yaw_offset = 0;

    double yaw_fc = 0;
    /*
    void recordCSV() {
        fprintf(
                  //0  1 2  3  4  5  6  7   8  9 10 11 12 13 14 15 16 17 18 19 20 21 22 23 24 25 26 27
            log_file, "%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f\n",
                state.pose.position.x,state.pose.position.y,state.pose.position.z,//3
                state.global_vel.x,state.global_vel.y,state.global_vel.z,//6
                state.pitch, state.roll, state.yaw,//9
                state.imu_data.linear_acceleration.x, state.imu_data.linear_acceleration.y, state.imu_data.linear_acceleration.z,//12
                state.pitch_cmd, state.roll_cmd, state.yaw_cmd,state.abx_cmd,//16
                state.acc_cmd.x, state.acc_cmd.y, state.acc_cmd.z,//19
                state.vel_cmd.x, state.vel_cmd.y, state.vel_cmd.z,//22
                state.pos_sp.x, state.pos_sp.y, state.pos_sp.z,//25
                state.vel_desired.x, state.vel_desired.y, state.vel_desired.z//28
            );
        fflush(log_file);
    }*/

    void read_controller_param(RotorPosCtrlParam & ctrlP) {
        nh.param<double>("pid_param/p_x/p", ctrlP.p_x.p, 0);
        nh.param<double>("pid_param/p_x/i", ctrlP.p_x.i, 0);
        nh.param<double>("pid_param/p_x/d", ctrlP.p_x.d, 0);

        nh.param<double>("pid_param/p_y/p", ctrlP.p_y.p, 0);
        nh.param<double>("pid_param/p_y/i", ctrlP.p_y.i, 0);
        nh.param<double>("pid_param/p_y/d", ctrlP.p_y.d, 0);

        nh.param<double>("pid_param/p_z/p", ctrlP.p_z.p, 0);
        nh.param<double>("pid_param/p_z/i", ctrlP.p_z.i, 0);
        nh.param<double>("pid_param/p_z/d", ctrlP.p_z.d, 0);

        nh.param<double>("pid_param/v_x/p", ctrlP.v_x.p, 0);
        nh.param<double>("pid_param/v_x/i", ctrlP.v_x.i, 0);
        nh.param<double>("pid_param/v_x/d", ctrlP.v_x.d, 0);

        nh.param<double>("pid_param/v_x/max_i", ctrlP.v_x.max_err_i, 15);

        nh.param<double>("pid_param/v_y/p", ctrlP.v_y.p, 0);
        nh.param<double>("pid_param/v_y/i", ctrlP.v_y.i, 0);
        nh.param<double>("pid_param/v_y/d", ctrlP.v_y.d, 0);
        nh.param<double>("pid_param/v_y/max_i", ctrlP.v_y.max_err_i, 15);


        nh.param<double>("pid_param/v_z/p", ctrlP.v_z.p, 0);
        nh.param<double>("pid_param/v_z/i", ctrlP.v_z.i, 0);
        nh.param<double>("pid_param/v_z/d", ctrlP.v_z.d, 0);
        nh.param<double>("pid_param/v_z/max_i", ctrlP.v_z.max_err_i, 10);

        nh.param<double>("pid_param/thr/p", ctrlP.thrust_ctrl.abx.p, 0);
        nh.param<double>("pid_param/thr/i", ctrlP.thrust_ctrl.abx.i, 0);
        nh.param<double>("pid_param/thr/d", ctrlP.thrust_ctrl.abx.d, 0);
        nh.param<double>("pid_param/thr/level_thrust", ctrlP.thrust_ctrl.level_thrust, 0.5);

        
    }
    ros::Timer control_timer;


public:
    SwarmPosControl(ros::NodeHandle & _nh):
    nh(_nh) {
        RotorPosCtrlParam ctrlP;

        ctrlP.ctrl_frame = CTRL_FRAME::VEL_WORLD_ACC_WORLD;
        ctrlP.coor_sys = FRAME_COOR_SYS::FLU;

        read_controller_param(ctrlP);

        ROS_INFO("Init pos control");
        pos_ctrl = new RotorPositionControl(ctrlP);
        ROS_INFO("Pos control success");

        control_timer = nh.createTimer(ros::Duration(0.005), &SwarmPosControl::control_update, this);
        
        log_path = "/home/dji/swarm_log_lastest";

        init_log_file();

        state_pub = nh.advertise<swarm_msgs::swarm_pos_control_state>("swarm_pos_control_state", 10);
        
        odom_sub = nh.subscribe("odometry", 1 , &SwarmPosControl::OnVisualOdometry, this);
        swarm_pos_cmd_sub = nh.subscribe("swarm_pos_cmd", 1 , &SwarmPosControl::OnVisualOdometry, this);
        fc_att_sub = nh.subscribe("fc_attitude", 1, &SwarmPosControl::onFCAttitude, this);
        control_pub = nh.advertise<sensor_msgs::Joy>("dji_sdk_control", 10);
    }   

    void init_log_file() {
        /*
        time_t rawtime;
        struct tm * timeinfo;
        char buffer[80] = {0};
        time (&rawtime);
        timeinfo = localtime(&rawtime);
        int r = rand();  
        char str[100] = {0};

        sprintf(buffer, "/home/nvidia/scp_log/log_%d.csv", r);
        // strftime(buffer,sizeof(buffer), str, timeinfo);

        FILE* flog_list = fopen("/home/nvidia/scp_log/log_list.txt", "a");
        fprintf(flog_list,"%s\n", buffer);
        fflush(flog_list);
        fclose(flog_list);
        ROS_INFO("opening %s as log", buffer);

        log_file = fopen(buffer,"w");
        */
    }

    void on_imu_data(const sensor_msgs::Imu & _imu) {
        state.imu_data = _imu;
        Eigen::Vector3d acc(
            state.imu_data.linear_acceleration.x,
            state.imu_data.linear_acceleration.y,
            state.imu_data.linear_acceleration.z
        );

        pos_ctrl->set_body_acc(acc);
    }

    void set_drone_global_pos_vel(Eigen::Vector3d pos, Eigen::Vector3d vel) {
        pos_ctrl->set_pos(pos);
        pos_ctrl->set_global_vel(vel);

        state.pose.position.x = pos.x();
        state.pose.position.y = pos.y();
        state.pose.position.z = pos.z();

        state.global_vel.x = vel.x();
        state.global_vel.y = vel.y();
        state.global_vel.z = vel.z();

    }

    void OnSwarmPosCommand(const swarm_msgs::swarm_pos_ctrl_cmd & _cmd) {
        if (_cmd.ctrl_mode == swarm_pos_ctrl_cmd::POS_CTRL_POS_MODE) {
            pos_sp.x() = _cmd.pos_sp.x;
            pos_sp.y() = _cmd.pos_sp.y;
            pos_sp.z() = _cmd.pos_sp.z;
        } else if (_cmd.ctrl_mode == swarm_pos_ctrl_cmd::POS_CTRL_VEL_MODE) {
            vel_sp.x() = _cmd.vel_sp.x;
            vel_sp.y() = _cmd.vel_sp.y;
            vel_sp.z() = _cmd.vel_sp.z;
        } else if (_cmd.ctrl_mode == swarm_pos_ctrl_cmd::POS_CTRL_IDLE_MODE) {

        }

        state.ctrl_mode = _cmd.ctrl_mode;

        state.yaw_sp = _cmd.yaw_sp;
    }

    Eigen::Quaterniond yaw_offset_mocap_conversion() {
        return Eigen::Quaterniond(Eigen::AngleAxisd(yaw_offset, Vector3d::UnitZ()));
    }

    void onFCAttitude(const geometry_msgs::QuaternionStamped & _quat) {
        geometry_msgs::Quaternion quat = _quat.quaternion;
        Eigen::Quaterniond q(quat.w, 
            quat.x, quat.y, quat.z);
        Eigen::Vector3d pry = quat_to_pry(q);
        yaw_fc = pry.z();
    }

    void OnVisualOdometry(const nav_msgs::Odometry & odom) {
        auto pose = odom.pose.pose;
        auto velocity = odom.twist.twist.linear;

        Eigen::Vector3d pos(
            pose.position.x,
            pose.position.y,
            pose.position.z
        );

        Eigen::Vector3d vel(
            velocity.x,
            velocity.y,
            velocity.z
        );

        Eigen::Quaterniond quat(pose.orientation.w, 
            pose.orientation.x, pose.orientation.y, pose.orientation.z);

        pos_ctrl->set_pos(pos);
        pos_ctrl->set_global_vel(vel);
        set_drone_global_pos_vel(pos, vel);

        Eigen::Vector3d pry = quat_to_pry(quat);
        double yaw_odom = pry.z();
        yaw_offset = constrainAngle(yaw_fc - yaw_odom);
    }


    void set_drone_attitude_target(AttiCtrlOut atti_out) {
        //Use dji ros to set drone attitude target
        sensor_msgs::Joy dji_command_so3; //! @note for dji ros wrapper
        dji_command_so3.header.stamp    = ros::Time::now();
        dji_command_so3.header.frame_id = std::string("FRD");
        uint8_t flag;
        flag = VERTICAL_THRUST | HORIZONTAL_ANGLE | YAW_ANGLE | HORIZONTAL_BODY | STABLE_DISABLE;

        dji_command_so3.axes.push_back(atti_out.roll_sp);       // x
        dji_command_so3.axes.push_back(atti_out.pitch_sp);       // y
        dji_command_so3.axes.push_back(atti_out.thrust_sp * 100.0); // z
        dji_command_so3.axes.push_back(atti_out.yaw_sp + yaw_offset);       // w
        dji_command_so3.axes.push_back(flag);

        control_pub.publish(dji_command_so3);
    }

    void control_update(const ros::TimerEvent & e) {
        state.count ++;
        float dt = (e.current_real - e.last_real).toSec();
        
        Eigen::Vector3d acc_sp(0, 0, 0);

        if (state.ctrl_mode == swarm_pos_ctrl_cmd::POS_CTRL_IDLE_MODE) {
            //IDLE

            return;
        }
        if (state.ctrl_mode == swarm_pos_ctrl_cmd::POS_CTRL_POS_MODE) {
            vel_sp = pos_ctrl->control_pos(pos_sp, dt);            
        }
        
        acc_sp = pos_ctrl->control_vel(vel_sp, dt);

        acc_sp.z() =  pos_ctrl->control_vel_z(vel_sp.z(), dt);


        //Send acc sp to network or
        YawCMD yaw_cmd;
        yaw_cmd.yaw_mode = YAW_MODE_LOCK;
        yaw_cmd.yaw_sp = state.yaw_sp;

        acc_sp.x() = float_constrain(acc_sp.x(), -10, 10);
        acc_sp.y() = float_constrain(acc_sp.y(), -10, 10);
        acc_sp.z() = float_constrain(acc_sp.z(), -10, 10);


        AttiCtrlOut atti_out =  pos_ctrl->control_acc(acc_sp, yaw_cmd, dt);

        ROS_INFO("!!!!Geometry atti out R %4.3f P %4.3f Y %4.3f thr : %3.2f", 
            atti_out.roll_sp * 57.3,
            atti_out.pitch_sp * 57.3,
            atti_out.yaw_sp * 57.3,
            atti_out.thrust_sp
        );
            
        set_drone_attitude_target(atti_out);
        
        state.pos_sp.x = pos_sp.x();
        state.pos_sp.y = pos_sp.y();
        state.pos_sp.z = pos_sp.z();
        state.acc_cmd.x = acc_sp.x();
        state.acc_cmd.y = acc_sp.y();
        state.acc_cmd.z = acc_sp.z();

        state.vel_cmd.x = vel_sp.x();
        state.vel_cmd.y = vel_sp.y();
        state.vel_cmd.z = vel_sp.z();

        state_pub.publish(state);

        // recordCSV();
    }

};

int main(int argc, char** argv)
{

    ROS_INFO("SCP_POS_CONTROL_INIT\nIniting\n");

    ros::init(argc, argv, "scp_pos_control");

    ros::NodeHandle nh("scp_pos_control");

    SwarmPosControl pos_control(nh);
    ros::spin();

}
