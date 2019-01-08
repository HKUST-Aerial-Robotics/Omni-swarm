#include <ros/ros.h>
#include <swarm_msgs/drone_pos_ctrl_cmd.h>
#include <swarm_msgs/drone_onboard_command.h>
#include <swarm_msgs/drone_commander_state.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Joy.h>
#include <std_msgs/UInt8.h>
#include <math.h>
#include <dji_sdk/ControlDevice.h>
#include <dji_sdk/SDKControlAuthority.h>
#include <dji_sdk/DroneArmControl.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/QuaternionStamped.h>
#include <eigen3/Eigen/Dense>


using namespace swarm_msgs;
using namespace Eigen;

#define MAX_VO_LATENCY 0.2f
#define MAX_LOSS_RC 0.1f
#define MAX_LOSS_SDK 0.1f
#define MAX_ODOM_VELOCITY 25.0f

#define RC_DEADZONE_RPY 0.01
#define RC_DEADZONE_THRUST 0.1

#define RC_MAX_TILT_VEL 5.0
#define RC_MAX_Z_VEL 2.0
#define RC_MAX_YAW_RATE 1.57
#define RC_MAX_TILT_ANGLE 0.52
#define TAKEOFF_VEL_Z 2.0
#define LANDING_VEL_Z -1.0
#define MAX_AUTO_Z_ERROR 0.05

#define LOOP_DURATION 0.02

#define DEBUG_OUTPUT
#define DEBUG_HOVER_CTRL

#define DCMD drone_commander_state
#define DPCL drone_pos_ctrl_cmd

inline Eigen::Vector3d quat_to_pry(Eigen::Quaterniond q);
class DroneCommander {
    ros::NodeHandle & nh;
    drone_commander_state state;

    ros::Subscriber vo_sub;
    ros::Subscriber onboard_cmd_sub;
    ros::Subscriber rc_sub;
    ros::Subscriber flight_status_sub;
    ros::Subscriber ctrl_dev_sub;
    ros::Subscriber fc_att_sub;

    ros::Timer loop_timer;

    ros::Time last_rc_ts;
    ros::Time last_onboard_cmd_ts;
    ros::Time last_vo_ts;
    ros::Time last_flight_status_ts;

    nav_msgs::Odometry odometry;
    sensor_msgs::Joy rc;

    ros::Time boot_time;

    ros::Publisher commander_state_pub;
    ros::Publisher ctrl_cmd_pub;

    drone_pos_ctrl_cmd * ctrl_cmd = nullptr;

    ros::ServiceClient control_auth_client;

    Eigen::Vector3d hover_pos = Eigen::Vector3d(0, 0, 0);
    Eigen::Vector3d takeoff_origin = Eigen::Vector3d(0, 0, 0);

    bool takeoff_inited = false;

    int control_count = 0;

    int last_hover_count = -1;


    double yaw_fc = 0;
    double yaw_vo = 0;

    bool yaw_sp_inited = false;

public:
    DroneCommander(ros::NodeHandle & _nh):
        nh(_nh) {
        init_states();
        init_subscribes();


        boot_time = ros::Time::now();

        last_flight_status_ts = ros::Time::now();
        last_rc_ts = ros::Time::now();
        last_vo_ts = ros::Time::now();
        last_onboard_cmd_ts = ros::Time::now();


        commander_state_pub = nh.advertise<drone_commander_state>("swarm_commander_state", 1);

        ctrl_cmd_pub = nh.advertise<drone_pos_ctrl_cmd>("/drone_position_control/drone_pos_cmd", 1);

        control_auth_client = nh.serviceClient<dji_sdk::SDKControlAuthority>("sdk_control_authority");

        ROS_INFO("Waitting for services");
        control_auth_client.waitForExistence();
        ROS_INFO("Services ready");
        
        ctrl_cmd = &state.ctrl_cmd;
        
        loop_timer = nh.createTimer(ros::Duration(LOOP_DURATION), &DroneCommander::loop, this);

    }

    void init_states() {
        state.ctrl_input_state = DCMD::CTRL_INPUT_NONE;
        state.flight_status = DCMD::FLIGHT_STATUS_IDLE;
        state.commander_ctrl_mode = DCMD::CTRL_MODE_IDLE;
        state.djisdk_valid = false;
        state.is_armed = false;
        state.rc_valid = false;
        state.onboard_cmd_valid = false;
        state.vo_valid = false;

        state.control_auth = DCMD::CTRL_AUTH_RC;
    }
    void init_subscribes() {
        vo_sub = nh.subscribe("visual_odometry", 1, &DroneCommander::vo_callback, this);
        onboard_cmd_sub = nh.subscribe("onboard_command", 10, &DroneCommander::onboard_cmd_callback, this);
        flight_status_sub = nh.subscribe("flight_status", 1, &DroneCommander::flight_status_callback, this);
        rc_sub = nh.subscribe("rc", 1, &DroneCommander::rc_callback, this);
        ctrl_dev_sub = nh.subscribe("control_device", 1, &DroneCommander::ctrl_dev_callback, this);
        fc_att_sub = nh.subscribe("fc_attitude", 1, &DroneCommander::fc_attitude_callback, this);
    }

    void vo_callback(const nav_msgs::Odometry & _odom);
    void rc_callback(const sensor_msgs::Joy & _rc);
    void flight_status_callback(const std_msgs::UInt8 & _flight_status);
    void onboard_cmd_callback(const drone_onboard_command & _cmd);
    void ctrl_dev_callback(const dji_sdk::ControlDevice & _ctrl_dev);
    void fc_attitude_callback(const geometry_msgs::QuaternionStamped & _quat);
    void loop(const ros::TimerEvent & _e);

    bool is_odom_valid(const nav_msgs::Odometry & _odom);
    bool is_rc_valid(const sensor_msgs::Joy & _rc);

    bool check_control_auth();

    void try_arm(bool arm);

    void try_control_auth(bool auth);

    void process_control();

    void process_input_source();

    bool rc_request_onboard();
    bool rc_request_vo();
    bool rc_moving_stick();

    void process_control_mode();

    void prepare_control_hover();

    void process_control_idle();
    void process_control_takeoff();
    void process_control_landing();
    void process_control_posvel();
    void process_control_att();
    void process_control_mission() {};

    void process_rc_input();
    void process_none_input();
    void process_onboard_input();

    void reset_ctrl_cmd();
    void reset_yaw_sp();

    void request_ctrl_mode(uint32_t req_ctrl_mode);
    
    void send_ctrl_cmd();
};



void DroneCommander::loop(const ros::TimerEvent & _e) {
    static int count = 0; 
    if (state.djisdk_valid && (ros::Time::now() - last_flight_status_ts).toSec() > MAX_LOSS_SDK) {
        ROS_INFO("Flight Status loss time %3.2f, is invalid", (ros::Time::now() - last_flight_status_ts).toSec());        
        state.djisdk_valid = false;
    }

    if (state.vo_valid && (ros::Time::now() - last_vo_ts).toSec() > MAX_VO_LATENCY) {
        state.vo_valid = false;
        ROS_INFO("VO loss time %3.2f, is invalid", (ros::Time::now() - last_vo_ts).toSec());
    }

    if (state.rc_valid && (ros::Time::now() - last_rc_ts).toSec() > MAX_LOSS_RC ) {
        state.rc_valid = false;
        ROS_INFO("RC loss time %3.2f, is invalid", (ros::Time::now() - last_rc_ts).toSec());
    }

    if (count ++ % 50 == 0)
    {
#ifdef DEBUG_OUTPUT
        if (rc.axes.size() >= 6)
        ROS_INFO("RC valid %d %3.2f %3.2f %3.2f %3.2f %4.0f %4.0f",
            state.rc_valid,
            rc.axes[0],
            rc.axes[1],
            rc.axes[2],
            rc.axes[3],
            rc.axes[4],
            rc.axes[5]
        );

        ROS_INFO("ctrl_input_state %d, flight_status %d\n control_auth %d  ctrl_mode %d, is_armed %d\n rc_valid %d onboard_cmd_valid %d vo_valid%d sdk_valid %d ",
            state.ctrl_input_state,
            state.flight_status,
            state.control_auth,
            state.commander_ctrl_mode,
            state.is_armed,
            state.rc_valid,
            state.onboard_cmd_valid,
            state.vo_valid,
            state.djisdk_valid
        );
#endif
    }

    if (!state.djisdk_valid) {
        return;
    }
    
    if (!yaw_sp_inited) {
        reset_yaw_sp();
    }

    reset_ctrl_cmd();
    process_input_source();

    if (check_control_auth()){
        process_control_mode();
        process_control();
    } else {
        reset_yaw_sp();
    }

    commander_state_pub.publish(state);
}


void DroneCommander::try_arm(bool arm) {
    dji_sdk::DroneArmControl arm_srv;
    if (state.djisdk_valid && state.flight_status == DCMD::FLIGHT_STATUS_IDLE) {
        // TODO:
        // rosservice call /dji_sdk_1/dji_sdk/drone_arm_control "arm: 0"
        arm_srv.request.arm = arm;
        ros::service::call("/dji_sdk_1/dji_sdk/drone_arm_control", arm_srv);

    }
}

void DroneCommander::try_control_auth(bool auth) {
    dji_sdk::SDKControlAuthority srv;
    srv.request.control_enable = auth;
    if (control_auth_client.call(srv))
    {
        ROS_INFO("Require control auth %d, res %d", auth, srv.response.ack_data);
        state.control_auth = srv.response.ack_data;
    } else {
        ROS_ERROR("Failed to call service control auth");
    }
}

bool DroneCommander::check_control_auth() {
    bool require_auth_this = true;
    if (!state.djisdk_valid)
        return false;
    if (state.rc_valid) {
        if (this->rc_request_vo()){
            require_auth_this = true;
        } else {
            require_auth_this = false;
        }
    }

    //If rc still not available, will try to grab auth
    if (!state.rc_valid) {
        require_auth_this = true;
    }

    if (require_auth_this) {
        if (state.control_auth == DCMD::CTRL_AUTH_RC || 
            state.control_auth == DCMD::CTRL_AUTH_APP) {
            ROS_INFO("Require AUTH");
        }
    }
    else {
        if (state.control_auth == DCMD::CTRL_AUTH_THIS){
            ROS_INFO("Relase AUTH");
        }
    }

    if ((require_auth_this && state.control_auth != DCMD::CTRL_AUTH_THIS) ||
        (!require_auth_this && state.control_auth == DCMD::CTRL_AUTH_THIS)
        )
        try_control_auth(require_auth_this);

    return state.control_auth == DCMD::CTRL_AUTH_THIS;
}


void DroneCommander::vo_callback(const nav_msgs::Odometry & _odom) {
    bool vo_valid = is_odom_valid(_odom);
    if (!state.vo_valid && vo_valid) {
        //Vo first time come
        //reset yaw sp use vo yaw
        reset_yaw_sp();
    }
    state.vo_valid = vo_valid;
    if (state.vo_valid) {
        odometry = _odom;
        last_vo_ts = _odom.header.stamp;
    }
    auto pose = _odom.pose.pose;
    Eigen::Quaterniond quat(pose.orientation.w, 
        pose.orientation.x, pose.orientation.y, pose.orientation.z);
    Eigen::Vector3d pry = quat_to_pry(quat);
    yaw_vo = pry.z();
}


void DroneCommander::rc_callback(const sensor_msgs::Joy & _rc) {
    state.rc_valid = is_rc_valid(_rc);
    
    if (state.rc_valid) {
        rc = _rc;
        last_rc_ts = ros::Time::now();
    }

    state.djisdk_valid = true;
}

void DroneCommander::flight_status_callback(const std_msgs::UInt8 & _flight_status) {
    //TODO:
    uint8_t _status = _flight_status.data;
    if (_status == 0) {
        //IS on land not arm
        state.flight_status = DCMD::FLIGHT_STATUS_IDLE;
        state.is_armed = false;
    }

    if (_status == 1) {
        // Onland armed
        state.flight_status = DCMD::FLIGHT_STATUS_ARMED;
        state.is_armed = true;
    }

    if (_status == 2) {
        //In air
        state.flight_status = DCMD::FLIGHT_STATUS_IN_AIR;
        state.is_armed = true;
    }

    state.djisdk_valid = true;
    last_flight_status_ts = ros::Time::now();
}

void DroneCommander::fc_attitude_callback(const geometry_msgs::QuaternionStamped & _quat) {
    geometry_msgs::Quaternion quat = _quat.quaternion;
    Eigen::Quaterniond q(quat.w, 
        quat.x, quat.y, quat.z);
    Eigen::Vector3d pry = quat_to_pry(q);
    yaw_fc = pry.z();
}


void DroneCommander::onboard_cmd_callback(const drone_onboard_command & _cmd) {
    //TODO:
}


bool DroneCommander::rc_request_onboard() {
    return (rc.axes[4] == 10000 && rc.axes[5] == -10000);
}

bool DroneCommander::rc_request_vo() {
    return (rc.axes[4] == 10000);
}

bool DroneCommander::rc_moving_stick () {
    bool if_move =  fabs(rc.axes[0]) > RC_DEADZONE_RPY;
    if_move = if_move || fabs(rc.axes[1]) > RC_DEADZONE_RPY;
    if_move = if_move || fabs(rc.axes[3]) > RC_DEADZONE_RPY;
    if_move = if_move || fabs(rc.axes[2]) > RC_DEADZONE_THRUST;

    return if_move;
}

void DroneCommander::process_input_source () {
    if (state.ctrl_input_state == DCMD::CTRL_INPUT_NONE) {
        if (state.rc_valid) {
            state.ctrl_input_state = DCMD::CTRL_AUTH_RC;
        } else if (state.onboard_cmd_valid){
            state.ctrl_input_state = DCMD::CTRL_INPUT_ONBOARD;
        }
    }

    if (state.ctrl_input_state == DCMD::CTRL_INPUT_RC) {
        if (!state.rc_valid){
            state.ctrl_input_state = DCMD::CTRL_INPUT_NONE;
            if (state.onboard_cmd_valid) {
                state.ctrl_input_state = DCMD::CTRL_INPUT_ONBOARD;    
            }
        } else if (this->rc_request_onboard()) {
            state.ctrl_input_state = DCMD::CTRL_INPUT_ONBOARD;
        }
    }

    if (state.ctrl_input_state == DCMD::CTRL_INPUT_ONBOARD) {
        if (!state.onboard_cmd_valid)
        {
            if (state.rc_valid) {
                state.ctrl_input_state = DCMD::CTRL_INPUT_RC;
            } else {
                state.ctrl_input_state = DCMD::CTRL_INPUT_NONE;
            }
        }
    }

    switch (state.ctrl_input_state) {
        case DCMD::CTRL_INPUT_RC:
            process_rc_input();
            break;
        case DCMD::CTRL_INPUT_ONBOARD:
            process_onboard_input();
            break;
        case DCMD::CTRL_INPUT_NONE:
            process_none_input();
            break;
    }
}


void DroneCommander::process_rc_input () {
    if (rc_moving_stick()) {
        request_ctrl_mode(DCMD::CTRL_MODE_POSVEL);
    } else {
        if (state.commander_ctrl_mode != DCMD::CTRL_MODE_MISSION) {
            request_ctrl_mode(DCMD::CTRL_MODE_HOVER);
        }
    }

    //TODO: Generate command using rc
    double y = rc.axes[0];
    double x = rc.axes[1];
    double r = rc.axes[2];
    double z = rc.axes[3];


    switch (state.commander_ctrl_mode) {
        case DCMD::CTRL_MODE_POSVEL: {
            ctrl_cmd->yaw_sp = ctrl_cmd->yaw_sp + r * RC_MAX_YAW_RATE * LOOP_DURATION;
            ctrl_cmd->vel_sp.x = x * RC_MAX_TILT_VEL;
            ctrl_cmd->vel_sp.y = y * RC_MAX_TILT_VEL;
            ctrl_cmd->vel_sp.z = z * RC_MAX_Z_VEL;
            ctrl_cmd->ctrl_mode = DPCL::POS_CTRL_VEL_MODE;

            break;
        }

        case DCMD::CTRL_MODE_MISSION:
            break;        
        case DCMD::CTRL_MODE_HOVER:
            prepare_control_hover();
            break;

        case DCMD::CTRL_MODE_TAKEOFF: 
        case DCMD::CTRL_MODE_LANDING:
        case DCMD::CTRL_MODE_IDLE:        
        case DCMD::CTRL_MODE_ATT:
        default: {
            ctrl_cmd->ctrl_mode = DPCL::POS_CTRL_ATT_VELZ_MODE;
            double yaw = ctrl_cmd->yaw_sp = ctrl_cmd->yaw_sp + r * RC_MAX_YAW_RATE * LOOP_DURATION;
            ctrl_cmd->z_sp = z * RC_MAX_Z_VEL;
            
            double roll = x * RC_MAX_TILT_ANGLE;
            double pitch = x * RC_MAX_TILT_ANGLE;

            Quaterniond quat_sp = AngleAxisd(roll, Vector3d::UnitX())
                * AngleAxisd(pitch, Vector3d::UnitY())
                * AngleAxisd(yaw, Vector3d::UnitZ());
                
            ctrl_cmd->att_sp.w = quat_sp.w();
            ctrl_cmd->att_sp.x = quat_sp.x();
            ctrl_cmd->att_sp.y = quat_sp.y();
            ctrl_cmd->att_sp.z = quat_sp.z();
            break;
        }
    }


}

void DroneCommander::process_none_input () {
    if (state.commander_ctrl_mode != DCMD::CTRL_MODE_MISSION)
    {
        return;
    }
    else {
        request_ctrl_mode(DCMD::CTRL_MODE_LANDING);
    }
}

void DroneCommander::process_control_idle() {
    //Landing and disarm
    if (state.flight_status == DCMD::FLIGHT_STATUS_IN_AIR) {
        request_ctrl_mode(DCMD::CTRL_MODE_LANDING);
        process_control_landing();
    } else {
        if (state.is_armed) {
            this->try_arm(false);
        }
    }
}

void DroneCommander::process_onboard_input () {
    //TODO: writing onboard input
}

void DroneCommander::process_control() {
    control_count ++;

    if (state.control_auth != DCMD::CTRL_AUTH_THIS 
        || state.flight_status == DCMD::FLIGHT_STATUS_IDLE)
        return;

    
    switch (state.commander_ctrl_mode) {
        case DCMD::CTRL_MODE_HOVER:
            prepare_control_hover();
            process_control_posvel();
            break;
        case DCMD::CTRL_MODE_POSVEL:
            process_control_posvel();
            break;
        case DCMD::CTRL_MODE_ATT:
            process_control_att();
            break;
        case DCMD::CTRL_MODE_TAKEOFF:
            process_control_takeoff();
            break;
        case DCMD::CTRL_MODE_LANDING:
            process_control_landing();
            break;
        case DCMD::CTRL_MODE_MISSION:
            process_control_mission();
            break;
        
        case DCMD::CTRL_MODE_IDLE:
        default:
            process_control_idle();
            break;
    }
}
void DroneCommander::process_control_posvel () {
    // Check command first
    bool is_cmd_valid = true;
    if (is_cmd_valid)
    {
        send_ctrl_cmd();
    } else {
        ROS_ERROR("POSVEL Ctrl cmd invaild!");
        //TODO:
        // ctrl_cmd_pub.publish(*ctrl_cmd);
    }

}

void DroneCommander::process_control_att() {
    bool is_cmd_valid = true;
    if (is_cmd_valid)
    {
        ctrl_cmd_pub.publish(*ctrl_cmd);
    } else {
        ROS_ERROR("Att ctrl cmd invaild!");
        //TODO:
    }
}

void DroneCommander::process_control_takeoff() {
    //TODO: write takeoff scirpt
    bool is_in_air = state.flight_status == DCMD::FLIGHT_STATUS_IN_AIR;
    bool is_takeoff_finish = false;
    if (state.vo_valid) {
        is_takeoff_finish = odometry.pose.pose.position.z  > (state.takeoff_target_height - MAX_AUTO_Z_ERROR);
    } else {
        is_takeoff_finish = is_in_air;
    }

    if (!takeoff_inited) {
        takeoff_inited = true;
        if (state.vo_valid) {
            takeoff_origin.x() = odometry.pose.pose.position.x;
            takeoff_origin.y() = odometry.pose.pose.position.y;
            takeoff_origin.z() = odometry.pose.pose.position.z;
        }
    }

    if (is_in_air && state.vo_valid) {
        //Already in air, process as a  posvel control
        ctrl_cmd->ctrl_mode = DPCL::POS_CTRL_POS_MODE;
        ctrl_cmd->pos_sp.x = takeoff_origin.x();
        ctrl_cmd->pos_sp.y = takeoff_origin.y();
        ctrl_cmd->pos_sp.z = state.takeoff_target_height;

        
    } else {
        ctrl_cmd->ctrl_mode = DPCL::POS_CTRL_ATT_VELZ_MODE;
        Eigen::Quaterniond quat_sp = (Eigen::Quaterniond) Eigen::AngleAxisd(ctrl_cmd->yaw_sp, Eigen::Vector3d::UnitZ());
        ctrl_cmd->att_sp.w = quat_sp.w();
        ctrl_cmd->att_sp.x = quat_sp.x();
        ctrl_cmd->att_sp.y = quat_sp.y();
        ctrl_cmd->att_sp.z = quat_sp.z();
        ctrl_cmd->z_sp = TAKEOFF_VEL_Z;
    }

    if (is_takeoff_finish) {
        request_ctrl_mode(DCMD::CTRL_MODE_HOVER);
    }

    send_ctrl_cmd();
}



void DroneCommander::process_control_landing() {
    //TODO: write better landing
    bool is_landing_finish = state.flight_status < DCMD::FLIGHT_STATUS_IN_AIR;

    if (is_landing_finish) {
        request_ctrl_mode(DCMD::CTRL_MODE_IDLE);
    } else {
        ctrl_cmd->ctrl_mode = DPCL::POS_CTRL_ATT_VELZ_MODE;
        Eigen::Quaterniond quat_sp = (Eigen::Quaterniond) Eigen::AngleAxisd(ctrl_cmd->yaw_sp, Eigen::Vector3d::UnitZ());
        ctrl_cmd->att_sp.w = quat_sp.w();
        ctrl_cmd->att_sp.x = quat_sp.x();
        ctrl_cmd->att_sp.y = quat_sp.y();
        ctrl_cmd->att_sp.z = quat_sp.z();

        ctrl_cmd->z_sp = LANDING_VEL_Z;
    }

    send_ctrl_cmd();

}

void DroneCommander::request_ctrl_mode(uint32_t req_ctrl_mode) {
    switch (req_ctrl_mode) {
        case DCMD::CTRL_MODE_LANDING: {
            state.commander_ctrl_mode = req_ctrl_mode;
            break;
        }

        case DCMD::CTRL_MODE_TAKEOFF: {
            if (state.flight_status == DCMD::FLIGHT_STATUS_IN_AIR) {
                request_ctrl_mode(DCMD::CTRL_MODE_HOVER);       
            } else {
                state.commander_ctrl_mode = req_ctrl_mode;
            }
            break;
        }
        case DCMD::CTRL_MODE_MISSION:
        case DCMD::CTRL_MODE_HOVER:
        case DCMD::CTRL_MODE_POSVEL:{
            if (state.vo_valid) {
                state.commander_ctrl_mode = req_ctrl_mode;
            } else {
                request_ctrl_mode(DCMD::CTRL_MODE_ATT);
                return;
            }
            break;
        }
        case DCMD::CTRL_MODE_IDLE:
            state.commander_ctrl_mode = req_ctrl_mode;
            break;

        case DCMD::CTRL_MODE_ATT: {
            state.commander_ctrl_mode = req_ctrl_mode;
        }
    }

    if (state.ctrl_input_state == DCMD::CTRL_INPUT_NONE && state.commander_ctrl_mode != DCMD::CTRL_MODE_MISSION) {
        if (state.flight_status == DCMD::FLIGHT_STATUS_IN_AIR)
            request_ctrl_mode(DCMD::CTRL_MODE_LANDING);
    }


}

void DroneCommander::process_control_mode() {
    //Request self ctrl mode can check vo vaild
    request_ctrl_mode(state.commander_ctrl_mode);
}

void DroneCommander::send_ctrl_cmd() {
    ctrl_cmd_pub.publish(*ctrl_cmd);
}

void DroneCommander::prepare_control_hover() {
    if (last_hover_count != control_count - 1) {
        //Need to start new hover

        hover_pos.x() = odometry.pose.pose.position.x;
        hover_pos.y() = odometry.pose.pose.position.y;
        hover_pos.z() = odometry.pose.pose.position.z;

        ROS_INFO("Entering hover mode, will hover at %3.2f %3.2f %3.2f",
            hover_pos.x(),
            hover_pos.y(),
            hover_pos.z()
        );
    }

    ctrl_cmd->pos_sp.x = hover_pos.x();
    ctrl_cmd->pos_sp.y = hover_pos.y();
    ctrl_cmd->pos_sp.z = hover_pos.z();
    ctrl_cmd->vel_sp.x = 0;
    ctrl_cmd->vel_sp.y = 0;
    ctrl_cmd->vel_sp.z = 0;

    ctrl_cmd->ctrl_mode = DPCL::POS_CTRL_POS_MODE;
}


void DroneCommander::reset_ctrl_cmd() {
    ctrl_cmd->ctrl_mode = DPCL::POS_CTRL_IDLE_MODE;
    ctrl_cmd->pos_sp.x = 0;
    ctrl_cmd->pos_sp.y = 0;
    ctrl_cmd->pos_sp.z = 0;

    ctrl_cmd->vel_sp.x = 0;
    ctrl_cmd->vel_sp.y = 0;
    ctrl_cmd->vel_sp.z = 0;

    ctrl_cmd->att_sp.w = 0;
    ctrl_cmd->att_sp.x = 0;
    ctrl_cmd->att_sp.y = 0;
    ctrl_cmd->att_sp.z = 0;
    
    ctrl_cmd->z_sp = 0;
}


bool DroneCommander::is_odom_valid(const nav_msgs::Odometry & _odom) {
    if ( fabs(_odom.twist.twist.linear.x) > MAX_ODOM_VELOCITY ||
        fabs(_odom.twist.twist.linear.y) > MAX_ODOM_VELOCITY ||
        fabs(_odom.twist.twist.linear.z) > MAX_ODOM_VELOCITY
    )
    {
        return false;
    }

    if ((ros::Time::now() - _odom.header.stamp).toSec() > MAX_VO_LATENCY ) {
        return false;
    }

    return true;
}

bool DroneCommander::is_rc_valid(const sensor_msgs::Joy & _rc) {
    //TODO: Test rc vaild function,
    // This only works for SBUS!!!!
    if (
        _rc.axes[0] == 0 && 
        _rc.axes[1] == 0 && 
        _rc.axes[2] == 0 && 
        _rc.axes[3] == 0
    ) {
        return false;
    }
    return true;
}


void DroneCommander::ctrl_dev_callback(const dji_sdk::ControlDevice & _ctrl_dev) {
    //RC 0
    //App 1
    //SDK 2
    if (_ctrl_dev.controlDevice == 2) {
        state.control_auth = DCMD::CTRL_AUTH_THIS;
    }

    if (_ctrl_dev.controlDevice == 1) {
        state.control_auth = DCMD::CTRL_AUTH_APP;
    }

    if (_ctrl_dev.controlDevice == 0) {
        state.control_auth = DCMD::CTRL_AUTH_RC;
    }

}

void DroneCommander::reset_yaw_sp() {
    if (state.djisdk_valid) {
        if (state.vo_valid) {
            ctrl_cmd->yaw_sp = yaw_vo;
        } else {
            ctrl_cmd->yaw_sp = yaw_fc;
        }
        yaw_sp_inited = true;
    }
}

inline Eigen::Vector3d quat_to_pry(Eigen::Quaterniond q) {
    q.normalize();
    // Set up convenience variables

    double w = q.w(); 
    double x = q.x(); 
    double y = q.y(); 
    double z = q.z();
    double w2 = w*w; 
    double x2 = x*x; 
    double y2 = y*y; 
    double z2 = z*z;
    double xy = x*y; 
    double xz = x*z; 
    double yz = y*z;
    double wx = w*x; 
    double wy = w*y; 
    double wz = w*z;
    Eigen::Matrix3d R = q.toRotationMatrix();
    // R << w2+x2-y2-z2 , 2*(xy - wz) , 2*(wy + xz) ,
        //  2*(wz + xy) , w2-x2+y2-z2 , 2*(yz - wx) ,
        //  2*(xz - wy) , 2*(wx + yz) , w2-x2-y2+z2;

    // std::cout << R << std::endl;

    Eigen::Vector3d pry;      
    pry.z() = atan2(-R(0,1),R(1,1));//zxy(1)
    pry.y() = asin(R(2,1));//zxy(2)
    pry.x() = atan2(-R(2,0),R(2,2));//zxy(3)
  
    // printf("q:%4.3f %4.3f %4.3f %4.3f\n", q.w(), q.x(), q.y(), q.z());
    // printf("pry %4.3f %4.3f %4.3f\n", pry.x(), pry.y(), pry.z());

    return pry;//R.eulerAngles(1, 0, 2);;
};

int main(int argc, char** argv)
{

    ROS_INFO("SWARM_COMMANDER_CONTROL_INIT\nIniting\n");

    ros::init(argc, argv, "drone_commander");

    ros::NodeHandle nh("drone_commander");

    DroneCommander swarm_commander(nh);

    ROS_INFO("Drone Commander is ONLINE! \n");
    ros::spin();

}