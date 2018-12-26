#include <iostream>
#include <mavlink/common/mavlink.h>
#include <rotor_position_control.h>
#include <serial_port.h>
#include "ros/ros.h"
#include "quat_to_pry_euler.h"
#include "stdio.h"
#include <scp_pos_control/scp_pos_control_state.h>
#include <scp_pos_control/scpnet.h>
#include <geometry_msgs/PoseStamped.h>
#include <thread>
#include <ctime>
#include <time.h>
#include <stdlib.h>


using namespace scp_pos_control;

/*
double expo(double value, double exp, double g)
{
	double x = constrain(value, - 1, 1);
    g = 0;
	double gc = constrain(g, 0, 0.99);
	return expo(x, e) * (1 - gc) / (1 - fabsf(x) * gc);
}
*/

inline Eigen::Vector3d lowpass_filter(Eigen::Vector3d input, double fc, Eigen::Vector3d outputlast, double dt) {
	double RC = 1.0 / (fc *2 * M_PI);
	double alpha = dt / (RC + dt);
	return outputlast + (alpha* (input - outputlast));
}
double constrainAngle(double x){
    x = fmod(x + 180,360);
    if (x < 0)
        x += 360;
    return x - 180;
}

class SCPPosControl {
	Serial_Port *serial_port;
    ros::NodeHandle & nh;
    RotorPositionControl * pos_ctrl = nullptr;

    scp_pos_control::scp_pos_control_state state;

    std::vector<Eigen::Vector3d> pos_seq;
    std::vector<Eigen::Vector3d> vd_seq;
    std::vector<Eigen::Vector3d> accd_seq;
    Eigen::Vector3d pos_sp = Eigen::Vector3d(0, 0, 0);
    int pos_seq_ptr = 0;
    bool home_pos_seted = false;
    Eigen::Vector3d home_pos = Eigen::Vector3d(0, 0, 0);
    
    mavlink_rc_channels_t rch;
    bool has_rc_data = false;

    bool use_scpnet = false;
    bool use_manual_control = false;
    bool scp_pos_control_online = false;
    bool use_mocap = false;
    bool fix_hover_pos = false;

    Eigen::Vector3d hover_pos_fixed = Eigen::Vector3d(0,0,0);
    FILE*log_file;
    ros::Publisher state_pub;
    Eigen::Vector3d acc_sp_last = Eigen::Vector3d(0, 0, 0);
    double lowpass_fc = 20;
    Eigen::Vector3d pry_sp_last = Eigen::Vector3d(-1.57, 0 ,0);
    double feedforward_p = 1.0;

    std::string log_path;
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
    }

    void read_controller_param(RotorPosCtrlParam & ctrlP) {
        nh.param<double>("pid_param/p_x/p", ctrlP.p_x.p, 0);
        nh.param<double>("pid_param/p_x/i", ctrlP.p_x.i, 0);
        nh.param<double>("pid_param/p_x/d", ctrlP.p_x.d, 0);
        nh.param<double>("pid_param/feedforward_p", feedforward_p, 1.0);

        nh.param<double>("pid_param/p_y/p", ctrlP.p_y.p, 0);
        nh.param<double>("pid_param/p_y/i", ctrlP.p_y.i, 0);
        nh.param<double>("pid_param/p_y/d", ctrlP.p_y.d, 0);

        nh.param<double>("pid_param/p_z/p", ctrlP.p_z.p, 0);
        nh.param<double>("pid_param/p_z/i", ctrlP.p_z.i, 0);
        nh.param<double>("pid_param/p_z/d", ctrlP.p_z.d, 0);

        // nh.param<double>("pid_param/v_x/p", ctrlP.v_x.p, 0);
        // nh.param<double>("pid_param/v_x/i", ctrlP.v_x.i, 0);
        // nh.param<double>("pid_param/v_x/d", ctrlP.v_x.d, 0);
        ROS_INFO("Loading Vx");

        std::vector<double> v_list;

        if (!nh.getParam("v_list", v_list))
        {
            ROS_INFO("Failed to read list v_list");
        }
        nh.getParam("p", ctrlP.v_x.p);
        nh.getParam("i", ctrlP.v_x.i);
        nh.getParam("d", ctrlP.v_x.d);
        nh.getParam("v_list", ctrlP.v_x.v_list);

        ROS_INFO("Load param successfully %d", ctrlP.v_x.p.size());
        nh.param<double>("pid_param/v_x/max_i", ctrlP.v_x.max_err_i, 15);

        nh.param<double>("pid_param/v_y/p", ctrlP.v_y.p, 0);
        nh.param<double>("pid_param/v_y/i", ctrlP.v_y.i, 0);
        nh.param<double>("pid_param/v_y/d", ctrlP.v_y.d, 0);
        nh.param<double>("pid_param/v_y/max_i", ctrlP.v_y.max_err_i, 15);


        nh.param<double>("pid_param/v_z/p", ctrlP.v_z.p, 0);
        nh.param<double>("pid_param/v_z/i", ctrlP.v_z.i, 0);
        nh.param<double>("pid_param/v_z/d", ctrlP.v_z.d, 0);
        nh.param<double>("pid_param/v_z/max_i", ctrlP.v_z.max_err_i, 10);


        nh.param<double>("pid_param/acc_fltr", lowpass_fc, 0);

        nh.param<double>("pid_param/thr/p", ctrlP.thrust_ctrl.abx.p, 0);
        nh.param<double>("pid_param/thr/i", ctrlP.thrust_ctrl.abx.i, 0);
        nh.param<double>("pid_param/thr/d", ctrlP.thrust_ctrl.abx.d, 0);
        nh.param<double>("pid_param/thr/level_thrust", ctrlP.thrust_ctrl.level_thrust, 0.5);

        nh.param<bool>("use_scpnet", this->use_scpnet, false);
        nh.param<bool>("use_mocap", this->use_mocap, false);
        nh.param<bool>("use_manual_control", this->use_manual_control, true);
        nh.param<bool>("fix_hover_pos", this->fix_hover_pos, false);

        nh.param<double>("hover_pos_fixed/x", this->hover_pos_fixed.x(), 0);
        nh.param<double>("hover_pos_fixed/y", this->hover_pos_fixed.y(), 0);
        nh.param<double>("hover_pos_fixed/z", this->hover_pos_fixed.z(), 0); 

        if (this->fix_hover_pos) {
            ROS_INFO("Will use fixed hover position at %3.2f %3.2f %3.2f", 
                this->hover_pos_fixed.x(),
                this->hover_pos_fixed.y(),
                this->hover_pos_fixed.z()
            );
        }

        
    }
    ros::Timer read_timer;
    ros::Timer control_timer;
    ros::ServiceClient scpnet_client;
    std::thread th;

    ros::Subscriber mocap_sub;
    bool mocap_data_recved = false;

    geometry_msgs::PoseStamped _drone_pose;

    double yaw_offset = 0;
    double yaw_mocap = 0;
    double yaw_drone_compass = 0;
public:
    SCPPosControl(ros::NodeHandle & _nh):
    nh(_nh) {
        RotorPosCtrlParam ctrlP;

        ctrlP.ctrl_frame = CTRL_FRAME::VEL_WORLD_ACC_WORLD;
        ctrlP.coor_sys = FRAME_COOR_SYS::NED;

        read_controller_param(ctrlP);

        std::string uart_name = "/dev/ttyUSB0";
        int baudrate = 230400;
        nh.param<std::string>("uart_name", uart_name, "/dev/ttyUSB0");
        nh.param<int>("baudrate", baudrate, 230400);
        nh.param<bool>("use_mocap", use_mocap, false);
        
        ROS_INFO("Open px4 %s:%d", uart_name.c_str(), baudrate);

        serial_port = new Serial_Port(uart_name.c_str(), baudrate);
        serial_port->open_serial();

        ROS_INFO("Init pos control");
        pos_ctrl = new RotorPositionControl(ctrlP);
        ROS_INFO("Pos control success");

        control_timer = nh.createTimer(ros::Duration(0.02), &SCPPosControl::control_update, this);
        
        std::string csv_path = "";
        nh.param<std::string>("csv_path", csv_path, "");

        log_path = "/home/nvidia/scp_log";

        init_log_file();

        if (csv_path != "") {
            ROS_INFO("Loading traj %s", csv_path.c_str());
            load_pos_traj_csv(csv_path);
        } else {
            ROS_INFO("No traj set, use 0,0,0");
        }

        scpnet_client = nh.serviceClient<scpnet>("/scpnet");
        // scpnet_client.waitForExistence();
        th = std::thread([&] () {
            this->recv_thread();
        });

        state_pub = nh.advertise<scp_pos_control_state>("scp_pos_control_state", 10);

        if (use_mocap)
        {
            mocap_sub = nh.subscribe("/scp_drone/pose", 1, &SCPPosControl::OnMocapPoseRecv, this);
        }

    }
    void init_log_file() {
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

    Eigen::Quaterniond yaw_offset_mocap_conversion() {
        return Eigen::Quaterniond(Eigen::AngleAxisd(yaw_offset, Vector3d::UnitZ()));
    }

    void OnMocapPoseRecv(const geometry_msgs::PoseStamped & pose)
    {
        if(!mocap_data_recved)
        {
            _drone_pose = pose;
            mocap_data_recved = true;
            return;
        }

        Eigen::Vector3d pos(
            pose.pose.position.x,
            - pose.pose.position.y,
            - pose.pose.position.z
        );



        Eigen::Vector3d vel(
            pose.pose.position.x - _drone_pose.pose.position.x,
            -(pose.pose.position.y - _drone_pose.pose.position.y),
            -(pose.pose.position.z - _drone_pose.pose.position.z)
        );
        vel = vel / (pose.header.stamp - _drone_pose.header.stamp).toSec();

        pos_ctrl->set_pos(pos);
        pos_ctrl->set_global_vel(vel);
        set_drone_global_pos_vel(pos, vel);

        Eigen::Quaterniond quat(pose.pose.orientation.w, 
            pose.pose.orientation.x, pose.pose.orientation.y, pose.pose.orientation.z);

        // quat = Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitX()) * quat;
        Eigen::Vector3d pry = quat_to_pry(quat);
        yaw_mocap = - pry.z();

        yaw_offset = constrainAngle(yaw_drone_compass - yaw_mocap);
        // ROS_INFO("Yaw MOCAP!!!!!!!!!!!! %3.1f yaw drone %3.1f, offset %3.1f", yaw_mocap*57.3, yaw_drone_compass*57.3, yaw_offset*57.3);
        _drone_pose = pose;
    
    }


    void recv_thread() {
        while (true)
        {

        mavlink_message_t message;
        if (serial_port->read_message(message)){
            switch (message.msgid)
            {
                case MAVLINK_MSG_ID_HEARTBEAT:
                {
                    ROS_DEBUG("MAVLINK_MSG_ID_HEARTBEAT\n");
                    break;
                }

                case MAVLINK_MSG_ID_SYS_STATUS:
                {
                    break;
                }

                case MAVLINK_MSG_ID_BATTERY_STATUS:
                {
                    break;
                }

                case MAVLINK_MSG_ID_LOCAL_POSITION_NED:
                if (!use_mocap) {
                    static int count = 0;
                    count ++;
                    static ros::Time last_recv_time = ros::Time::now();
                    ros::Time time_now = ros::Time::now();

                    if (count % 10 == 1)
                    {
                        // ROS_INFO("Local pos ned freq %f", 1.0 / (time_now - last_recv_time).toSec());
                    }

                    last_recv_time = time_now;

                    mavlink_local_position_ned_t local_position_ned;
                    mavlink_msg_local_position_ned_decode(&message, &(local_position_ned));

                    Eigen::Vector3d pos(
                        local_position_ned.x,
                        local_position_ned.y,
                        local_position_ned.z
                    );

                    Eigen::Vector3d vel(
                        local_position_ned.vx,
                        local_position_ned.vy,
                        local_position_ned.vz
                    );

                    set_drone_global_pos_vel(pos, vel);
                    
                    // ROS_INFO("velz %3.2f", vel.z());
                    break;
                }


                case MAVLINK_MSG_ID_HIGHRES_IMU:
                {
                    //printf("MAVLINK_MSG_ID_HIGHRES_IMU\n");

                    static int count = 0;
                    count ++;
                    static ros::Time last_recv_time = ros::Time::now();
                    ros::Time time_now = ros::Time::now();

                    // if (count % 10 == 1)
                    // {
                        // ROS_INFO("Hires_imu freq %3.2f", 1.0 / (time_now - last_recv_time).toSec());
                    // }

                    last_recv_time = time_now; 
                    mavlink_highres_imu_t highres_imu;
                    mavlink_msg_highres_imu_decode(&message, &(highres_imu));

                    Eigen::Vector3d acc(
                        highres_imu.xacc,
                        highres_imu.yacc,
                        highres_imu.zacc
                    );

                    state.imu_data.linear_acceleration.x = acc.x();
                    state.imu_data.linear_acceleration.y = acc.y();
                    state.imu_data.linear_acceleration.z = acc.z();

                    pos_ctrl->set_body_acc(acc);

                    break;
                }

            
                case MAVLINK_MSG_ID_ATTITUDE_QUATERNION:
                {
                    static int count = 0;
                    count ++;
                    static ros::Time last_recv_time = ros::Time::now();
                    ros::Time time_now = ros::Time::now();

                    if (count % 10 == 1)
                    {
                        // ROS_INFO("ATTIRUDE_QUAT freq %f", 1.0 / (time_now - last_recv_time).toSec());
                    }

                    last_recv_time = time_now; 

                    //printf("MAVLINK_MSG_ID_ATTITUDE\n");
                    mavlink_attitude_quaternion_t attitude;
                    mavlink_msg_attitude_quaternion_decode(&message, &(attitude));

                    Eigen::Quaterniond quat(attitude.q1, attitude.q2, attitude.q3, attitude.q4);

                    pos_ctrl->set_attitude(quat);
                    state.pose.orientation.w = quat.w();
                    state.pose.orientation.x = quat.x();
                    state.pose.orientation.y = quat.y();
                    state.pose.orientation.z = quat.z();



                    Eigen::Vector3d pry = quat_to_pry(quat);
                    yaw_drone_compass = pry.z();

                    state.pitch = pry.x();
                    state.roll = pry.y();
                    state.yaw = pry.z();

                    break;
                }

                case MAVLINK_MSG_ID_RC_CHANNELS:
                {

                    mavlink_msg_rc_channels_decode(&message, &(rch));

                    if (rch.chan12_raw > 1900)
                    {
                        scp_pos_control_online = true;
                        if (!home_pos_seted) {
                            pos_sp = pos_ctrl->pos;
                            home_pos = pos_ctrl->pos;
                            home_pos_seted = true;

                            ROS_INFO("Set based home to %f %f %f", pos_sp.x(), pos_sp.y(), pos_sp.z());
                        }
                    } else {
                        scp_pos_control_online = false;

                        home_pos_seted = false;
                        pos_seq_ptr = 0;
                    }

                    // ROS_INFO("rc %d %d %d %d", rch.chan1_raw, rch.chan2_raw, rch.chan3_raw, rch.chan4_raw); 
                    has_rc_data = true;
                    break;
                }

                case MAVLINK_MSG_ID_STATUSTEXT:
                    mavlink_statustext_t status_text;
                    mavlink_msg_statustext_decode(&message, &(status_text));
                    // ROS_INFO("stats %s", status_text.text);
                    break;

                case MAVLINK_MSG_ID_ALTITUDE:
                {
                    static int count = 0;
                    count ++;
                    static ros::Time last_recv_time = ros::Time::now();
                    ros::Time time_now = ros::Time::now();

                    mavlink_altitude_t altitude_t;
                    mavlink_msg_altitude_decode(&message, &(altitude_t));

                    if (count % 10 == 1)
                    {
                        // ROS_INFO("ALTITUDE freq %f", 1.0 / (time_now - last_recv_time).toSec());
                    }

                    last_recv_time = time_now; 
                    break;
                }
                default:
                {
                    break;
                }

            } // end: switch msgid
        }
        }

    }

    void set_drone_attitude_target(AttiCtrlOut atti_out) {

        static int count = 0;
        count ++;
        mavlink_set_attitude_target_t atti_target;

        atti_target.q[0] = atti_out.atti_sp.w();
        atti_target.q[1] = atti_out.atti_sp.x();
        atti_target.q[2] = atti_out.atti_sp.y();
        atti_target.q[3] = atti_out.atti_sp.z();

        state.roll_cmd = atti_out.roll_sp;
        state.pitch_cmd = atti_out.pitch_sp;
        state.yaw_cmd = atti_out.yaw_sp;
        state.abx_cmd = atti_out.abx_sp;

        atti_target.body_roll_rate = atti_out.roll_sp;
        atti_target.body_pitch_rate = atti_out.pitch_sp;
        atti_target.body_yaw_rate = atti_out.yaw_sp;
        atti_target.thrust = atti_out.abx_sp;
    
        // if (count % 50 == 0)
        /*
            ROS_INFO("att_sp %3.2f %3.2f %3.2f %3.2f abx %3.2f rpy %3.2f %3.2f %3.2f", 
                atti_target.q[0],
                atti_target.q[1],
                atti_target.q[2],
                atti_target.q[3],
                atti_target.thrust,
                atti_target.body_roll_rate,
                atti_target.body_pitch_rate,
                atti_target.body_yaw_rate
            );
        */
        
        // if (count % 10 == 0)
        //     ROS_INFO("R:%3.2f P:%3.2f Y:%3.2f %4.3f",
        //         atti_target.body_roll_rate,
        //         atti_target.body_pitch_rate,
        //         atti_target.body_yaw_rate,
        //         atti_target.thrust
        //     );
        mavlink_message_t message;
	    mavlink_msg_set_attitude_target_encode(0, 0, &message, &atti_target);

        int ret = serial_port->write_message(message);
        if (ret == 0)
        {
            printf("FAILED to send message");
        }
        // printf("send %d\n", ret);
    }

    void control_update(const ros::TimerEvent & e) {
        static int count = 0;
        count++;
        float dt = (e.current_real - e.last_real).toSec();
        
        Eigen::Vector3d acc_sp(0, 0, 0);
        Eigen::Vector3d vel_sp(0, 0, 0);
        Eigen::Vector3d vel_desired(0, 0, 0);
        Eigen::Vector3d acc_desired(0, 0, 0);
        // Eigen::Vector3d pos_sp(0, 0, 0);
        if (!scp_pos_control_online)
            return;
        if (!use_manual_control)
        {
            if (pos_seq.size () > 0 && pos_seq_ptr < pos_seq.size() && (rch.chan8_raw > 1900))
            {
                pos_sp = pos_seq[pos_seq_ptr] + home_pos;
                vel_desired = vd_seq[pos_seq_ptr];
                acc_desired = accd_seq[pos_seq_ptr];
                ROS_INFO("Use pos_sp %3.2f %3.2f %3.2f", pos_sp.x(), pos_sp.y(), pos_sp.z());
                pos_seq_ptr ++;
            }

            if (this->fix_hover_pos)
            {
                pos_sp = this->hover_pos_fixed;
                vel_desired = Eigen::Vector3d(0, 0, 0);
                acc_desired = Eigen::Vector3d(0, 0, 0);
                ROS_INFO("Use pos_sp %3.2f %3.2f %3.2f, yawoff %3.2f * 57.3", pos_sp.x(), pos_sp.y(), pos_sp.z(), yaw_offset);

            }

        
            vel_sp = pos_ctrl->control_pos(pos_sp, dt) + vel_desired;  
            acc_sp = pos_ctrl->control_vel(vel_sp, dt);

            acc_sp.z() =  pos_ctrl->control_vel_z(vel_sp.z(), dt);

            acc_sp = acc_sp + acc_desired;

        }
        else {
            double vel_z_sp = 0;
            if (has_rc_data)
            {
                vel_desired.y() = vel_sp.y() = (rch.chan1_raw - 1500)/500.0*4.0;
                vel_desired.x() = vel_sp.x() = (rch.chan2_raw - 1500)/500.0*4.0;
                vel_desired.z() = vel_z_sp = 0;//-((double)(rch.chan3_raw - 1500))/500.0*1.0;
            }

            vel_sp.z() = pos_ctrl->control_pos_z(pos_sp.z(), dt);

            acc_sp = pos_ctrl->control_vel(vel_sp, dt);
            acc_sp.z() =  pos_ctrl->control_vel_z(vel_sp.z(), dt);

            // ROS_INFO("Pos z sp %3.2f now, %3.2f velz sp %3.2f accz sp%3.2f", 
                // pos_sp.z(), pos_ctrl->pos.z(), vel_sp.z(), acc_sp.z());

            // ROS_INFO("Vel zsp %3.2f velz %3.2f accsp %3.2f", vel_z_sp, pos_ctrl->vel.z(), acc_sp.z());
            if (count % 50 == 0)
                ROS_INFO("Use manual ctrl y:%4.3f x:%4.3f z:%4.3f velsp %4.3f with %d %d %d", 
                    acc_sp.y(),
                    acc_sp.x(),
                    acc_sp.z(),
                    vel_z_sp,
                    rch.chan1_raw,
                    rch.chan2_raw,
                    rch.chan3_raw
                );
        }

        //Send acc sp to network or
        YawCMD yaw_cmd;
        yaw_cmd.yaw_mode = YAW_MODE_LOCK;
        yaw_cmd.yaw_sp = 0;

        acc_sp.x() = float_constrain(acc_sp.x(), -10, 10);
        acc_sp.y() = float_constrain(acc_sp.y(), -10, 10);
        acc_sp.z() = float_constrain(acc_sp.z(), -10, 10);
	acc_sp_last = acc_sp = lowpass_filter(acc_sp, lowpass_fc, acc_sp_last, dt);

        state.acc_cmd.x = acc_sp.x();
        state.acc_cmd.y = acc_sp.y();
        state.acc_cmd.z = acc_sp.z();

        state.vel_cmd.x = vel_sp.x();
        state.vel_cmd.y = vel_sp.y();
        state.vel_cmd.z = vel_sp.z();

        AttiCtrlOut atti_out;
        if (!this->use_scpnet)
        {
            atti_out =  pos_ctrl->control_acc(acc_sp, yaw_cmd, dt);
            ROS_INFO("!!!!Geometry atti out %4.3f %4.3f %4.3f %4.3f", 
                atti_out.atti_sp.w(), 
                atti_out.atti_sp.x(), 
                atti_out.atti_sp.y(), 
                atti_out.atti_sp.z() 
            );
            
            atti_out.atti_sp = atti_out.atti_sp * Eigen::AngleAxisd(M_PI/2, Vector3d::UnitY());


            /*
            if (use_mocap)
            {
                atti_out.atti_sp = yaw_offset_mocap_conversion() * atti_out.atti_sp;
            }
            */

            Eigen::Vector3d pry = quat_to_pry(atti_out.atti_sp);
            atti_out.pitch_sp = pry.x() ;
            atti_out.roll_sp = pry.y();
            atti_out.yaw_sp = pry.z();
            
            atti_out.abx_sp = acc_sp.z() - GRAVITY;
        } else {
            // ROS_INFO("Use manual control with SCPNET");
            scpnet srv;
            //4 vx 5 vy 6 vz 7 ax 8 ay 9 az 10 vdx 11 vdy 12 vdz
            srv.request.scp_input.push_back(pos_ctrl->vel.x());
            srv.request.scp_input.push_back(pos_ctrl->vel.y());
            srv.request.scp_input.push_back(pos_ctrl->vel.z());

            srv.request.scp_input.push_back(acc_sp.x());
            srv.request.scp_input.push_back(acc_sp.y());
            srv.request.scp_input.push_back(acc_sp.z());

            srv.request.scp_input.push_back(vel_desired.x());
            srv.request.scp_input.push_back(vel_desired.y());
            srv.request.scp_input.push_back(vel_desired.z());

            if (scpnet_client.call(srv))
            {
                double pitch = srv.response.scp_output[0];
                double roll = srv.response.scp_output[1];
                double yaw = srv.response.scp_output[2];
		Eigen::Vector3d pry(pitch, roll, yaw);
		pry = pry_sp_last = lowpass_filter(pry, lowpass_fc, pry_sp_last, dt);
                atti_out.pitch_sp = constrainAngle(pry.x()); 
                atti_out.roll_sp = constrainAngle(pry.y()); 
                atti_out.yaw_sp = constrainAngle(pry.z()); 
                atti_out.abx_sp = - srv.response.scp_output[3];
                // ROS_INFO("SCPNet output %3.2f %3.2f %3.2f", pitch, roll, yaw);

            }
            else {
                // ROS_INFO("Call srv failed");
            }
        }
        if (use_mocap)
        {
            atti_out.yaw_sp = constrainAngle(atti_out.yaw_sp + yaw_offset);
        }
        else {
        }
        set_drone_attitude_target(atti_out);
        
        state.pos_sp.x = pos_sp.x();
        state.pos_sp.y = pos_sp.y();
        state.pos_sp.z = pos_sp.z();

        state.vel_desired.x = vel_desired.x();
        state.vel_desired.y = vel_desired.y();
        state.vel_desired.z = vel_desired.z();
        state_pub.publish(state);

        recordCSV();
    }

    void load_pos_traj_csv(std::string path) {
        FILE * f = fopen(path.c_str(), "r");
        if (f == nullptr)
        {
            ROS_INFO("Traj file not found");
            return;
        }
        float x, y, z, vx, vy, vz, ax, ay, az, yaw;
        while (fscanf(f, "%f,%f,%f,%f,%f,%f,%f,%f,%f,%f", &x, &y, &z, &vx, &vy, &vz, &ax, &ay, &az, &yaw) != EOF) {
            pos_seq.push_back(Eigen::Vector3d(x, y, z));
            vd_seq.push_back(Eigen::Vector3d(vx, vy, vz));
            accd_seq.push_back(Eigen::Vector3d(ax, ay, az));
            // ROS_INFO("Read %3.2f %3.2f %3.2f", x, y, z);
        }

        printf("Read seq len %ld\n", pos_seq.size());
    }

};

int main(int argc, char** argv)
{

    srand(time(NULL));   // Initialization, should only be called once.

    ROS_INFO("SCP_POS_CONTROL_INIT\nIniting\n");

    srand (time(NULL));
    
    ros::init(argc, argv, "scp_pos_control");

    ros::NodeHandle nh("scp_pos_control");

    SCPPosControl pos_control(nh);
    ros::spin();

}
