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

using namespace swarm_msgs;

#define MAX_VO_LATENCY 0.2f
#define MAX_LOSS_RC 0.1f
#define MAX_LOSS_SDK 0.1f
#define MAX_ODOM_VELOCITY 25.0f
#define DEBUG_OUTPUT_RC

class DroneCommander {
    ros::NodeHandle & nh;
    drone_commander_state state;

    ros::Subscriber vo_sub;
    ros::Subscriber onboard_cmd_sub;
    ros::Subscriber rc_sub;
    ros::Subscriber flight_status_sub;
    ros::Subscriber ctrl_dev_sub;

    ros::Timer loop_timer;

    ros::Time last_rc_ts;
    ros::Time last_onboard_cmd_ts;
    ros::Time last_vo_ts;
    ros::Time last_flight_status_ts;

    nav_msgs::Odometry odometry;
    sensor_msgs::Joy rc;

    ros::Time boot_time;

    ros::Publisher commander_state_pub;

    ros::ServiceClient control_auth_client;
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

    
        control_auth_client = nh.serviceClient<dji_sdk::SDKControlAuthority>("sdk_control_authority");

        ROS_INFO("Waitting for services");
        control_auth_client.waitForExistence();
        ROS_INFO("Services ready");

        loop_timer = nh.createTimer(ros::Duration(0.02), &DroneCommander::loop, this);

    }

    void init_states() {
        state.ctrl_input_state = drone_commander_state::CTRL_INPUT_NONE;
        state.flight_status = drone_commander_state::FLIGHT_STATUS_IDLE;
        state.commander_ctrl_mode = drone_commander_state::CTRL_MODE_IDLE;
        state.djisdk_valid = false;
        state.arm_status = false;
        state.rc_valid = false;
        state.onboard_cmd_valid = false;
        state.vo_valid = false;

        state.control_auth = drone_commander_state::CTRL_AUTH_RC;
    }
    void init_subscribes() {
        vo_sub = nh.subscribe("visual_odometry", 1, &DroneCommander::vo_callback, this);
        onboard_cmd_sub = nh.subscribe("onboard_command", 10, &DroneCommander::onboard_cmd_callback, this);
        flight_status_sub = nh.subscribe("flight_status", 1, &DroneCommander::flight_status_callback, this);
        rc_sub = nh.subscribe("rc", 1, &DroneCommander::rc_callback, this);
        ctrl_dev_sub = nh.subscribe("control_device", 1, &DroneCommander::ctrl_dev_callback, this);
    }

    void vo_callback(const nav_msgs::Odometry & _odom);
    void rc_callback(const sensor_msgs::Joy & _rc);
    void flight_status_callback(const std_msgs::UInt8 & _flight_status);
    void onboard_cmd_callback(const drone_onboard_command & _cmd);
    void ctrl_dev_callback(const dji_sdk::ControlDevice & _ctrl_dev);

    void loop(const ros::TimerEvent & _e);

    bool is_odom_valid(const nav_msgs::Odometry & _odom);
    bool is_rc_valid(const sensor_msgs::Joy & _rc);

    void check_control_auth();

    void try_arm(bool arm);

    void try_control_auth(bool auth);
};


void DroneCommander::try_arm(bool arm) {
    if (state.djisdk_valid) {
        // TODO:
        // rosservice call /dji_sdk_1/dji_sdk/drone_arm_control "arm: 0"
    }
}

void DroneCommander::vo_callback(const nav_msgs::Odometry & _odom) {
    state.vo_valid = is_odom_valid(_odom);
    if (state.vo_valid) {
        odometry = _odom;
        last_vo_ts = _odom.header.stamp;
    }
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
        state.flight_status = drone_commander_state::FLIGHT_STATUS_IDLE;
        state.arm_status = false;
    }

    if (_status == 1) {
        // Onland armed
        state.flight_status = drone_commander_state::FLIGHT_STATUS_ARMED;
        state.arm_status = true;
    }

    if (_status == 2) {
        //In air
        state.flight_status = drone_commander_state::FLIGHT_STATUS_IN_AIR;
        state.arm_status = true;
    }

    state.djisdk_valid = true;
    last_flight_status_ts = ros::Time::now();
}


void DroneCommander::onboard_cmd_callback(const drone_onboard_command & _cmd) {
    //TODO:
}

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
#ifdef DEBUG_OUTPUT_RC
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
#endif
    }

    check_control_auth();
    commander_state_pub.publish(state);
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
    //TODO: Test rc vaild function
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

void DroneCommander::check_control_auth() {
    bool require_auth_this = true;
    if (!state.djisdk_valid)
        return;
    if (state.rc_valid) {
        //TODO:
        //Use RC
        //Depend on button
    }

    //If rc still not available, will try to grab auth
    if (!state.rc_valid) {
        require_auth_this = true;
    }

    if (require_auth_this) {
        if (state.control_auth == drone_commander_state::CTRL_AUTH_RC || 
            state.control_auth == drone_commander_state::CTRL_AUTH_APP) {
            //Try to grab auth
            //TODO:
            // rosservice call /dji_sdk_1/dji_sdk/sdk_control_authority 1
            ROS_INFO("Require AUTH");
        }
    }
    else {
        if (state.control_auth == drone_commander_state::CTRL_AUTH_THIS){
            //TODO:
            // rosservice call /dji_sdk_1/dji_sdk/sdk_control_authority 0
            ROS_INFO("Relase AUTH");
        }
    }

    if ((require_auth_this && state.control_auth != drone_commander_state::CTRL_AUTH_THIS) ||
        (!require_auth_this && state.control_auth == drone_commander_state::CTRL_AUTH_THIS)
        )
    try_control_auth(require_auth_this);
}

void DroneCommander::ctrl_dev_callback(const dji_sdk::ControlDevice & _ctrl_dev) {
    //RC 0
    //App 1
    //SDK 2
    if (_ctrl_dev.controlDevice == 2) {
        state.control_auth = drone_commander_state::CTRL_AUTH_THIS;
    }

    if (_ctrl_dev.controlDevice == 1) {
        state.control_auth = drone_commander_state::CTRL_AUTH_APP;
    }

    if (_ctrl_dev.controlDevice == 0) {
        state.control_auth = drone_commander_state::CTRL_AUTH_RC;
    }

}

int main(int argc, char** argv)
{

    ROS_INFO("SWARM_COMMANDER_CONTROL_INIT\nIniting\n");

    ros::init(argc, argv, "drone_commander");

    ros::NodeHandle nh("drone_commander");

    DroneCommander swarm_commander(nh);

    ROS_INFO("Drone Commander is ONLINE! \n");
    ros::spin();

}