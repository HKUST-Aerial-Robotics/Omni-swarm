#pragma once

#include <eigen3/Eigen/Eigen>
#include <vector>
#include <stdio.h>

using namespace Eigen;

#define GRAVITY (9.695f)
#define MAX_HORIZON_VEL 10.0
#define MAX_VERTICAL_VEL 3.0
#define MAX_VERTICAL_ACC 10.0
#define MAX_HORIZON_ACC 10.0
#define MAX_TILT_ANGLE 0.8
#define MIN_THRUST 0.0

struct PIDParam {
    double p = 0;
    double i = 0;
    double d = 0;
    
    double max_err_i = 0;
};

struct SchulingPIDParam {
    std::vector<double> v_list;
    std::vector<double> p;
    std::vector<double> i;
    std::vector<double> d;
    double max_err_i = 0;

    int cases_num()
    {
        return v_list.size();
    }

};

enum CTRL_FRAME {
    VEL_WORLD_ACC_WORLD,
    VEL_WORLD_ACC_BODY,
    VEL_BODY_ACC_BODY
};

enum FRAME_COOR_SYS {
    NED,
    FLU
};

enum THRUST_SP_TYPE {
    THRUST_PASSTHROUGH,
    THRUST_ACC
};

enum YAW_MODE {
    YAW_MODE_LOCK,
    YAW_MODE_KEEP,
    YAW_MODE_RATE
};

struct YawCMD{
    float yaw_sp = 0;
    YAW_MODE yaw_mode = YAW_MODE_LOCK;
};

struct RotorThrustControlParam {
    PIDParam abx;
    double level_thrust = 0.5;
    FRAME_COOR_SYS coor_sys = NED;
};

struct RotorPosCtrlParam {
    PIDParam p_x, p_y, p_z;
    PIDParam v_y, v_z, v_x;
    CTRL_FRAME ctrl_frame = CTRL_FRAME::VEL_WORLD_ACC_WORLD;
    RotorThrustControlParam thrust_ctrl;
    FRAME_COOR_SYS coor_sys = NED;
};

struct AttiCtrlOut {
    enum {
        THRUST_MODE_THRUST,
        THRUST_MODE_VELZ
    };
    Eigen::Quaterniond atti_sp = Eigen::Quaterniond(1, 0, 0, 0);
    double roll_sp = 0, pitch_sp = 0, yaw_sp = 0;
    double thrust_sp = 0;
    double abx_sp = 0;
    int thrust_mode;
};

inline Eigen::Vector3d quat2eulers(const Eigen::Quaterniond & quat) {
    Eigen::Vector3d rpy;
    rpy.x() = atan2(2 * (quat.w() * quat.x() + quat.y() * quat.z()),
                    1 - 2 * (quat.x() * quat.x() + quat.y() * quat.y()));
    rpy.y() = asin(2 * (quat.w() * quat.y() - quat.z() * quat.x()));
    rpy.z() = atan2(2 * (quat.w() * quat.z() + quat.x() * quat.y()),
                    1 - 2 * (quat.y() * quat.y() + quat.z() * quat.z()));
    return rpy;
}

double float_constrain(double v, double min, double max)
{
    if (v < min) {
        return min;
    }
    if (v > max) {
        return max;
    }
    return v;
}




class PIDController {
    double err_integrate = 0;
    double err_last = 0;

    bool inited = false;
protected:
    PIDParam param;
    
public:
    PIDController(PIDParam _param):
        param(_param)
    {}

    PIDController()
    {}

    void reset() {
        err_integrate = 0;
        err_last = 0;
    }

    virtual inline double control(const double & err, double dt, bool report=false) {
        //TODO:
        if (!inited) {
            err_last = err;
            inited = true;
        }

        double ret = err * param.p + err_integrate * param.i + (err - err_last)/dt * param.d;

        err_last = err;
        err_integrate = err_integrate + err * dt;

        if (err_integrate > param.max_err_i)
        {
            err_integrate = param.max_err_i;
        }
        if (err_integrate < - param.max_err_i)
        {
            err_integrate = -param.max_err_i;
        }

        // if (report)
            // printf("ERR %3.2f ERRI %3.2f OUTPUTI %3.2f OUTPUT %3.2f\n", err, err_integrate,  err_integrate * param.i, ret);

        return ret;


    }


};

class SchulingPIDController: public PIDController {
    SchulingPIDParam schul_param;

public:
    SchulingPIDController(SchulingPIDParam _param):
        schul_param(_param)
    {
        printf("schul size %ld", schul_param.v_list.size());
        param.p = schul_param.p[0];
        param.i = schul_param.i[0];
        param.d = schul_param.d[0];
    }

    void calc_pid(const double x)
    {
        int ptr = 0;
        // printf("Schul cases %d\n", schul_param.cases_num());
        if (schul_param.cases_num() == 1)
        {
            param.p = schul_param.p[0];
            param.i = schul_param.i[0];
            param.d = schul_param.d[0];
            return;
        }

        while (ptr < schul_param.cases_num() && schul_param.v_list[ptr] < x) {
            ptr ++;
        }

        printf("ptr is %d\n", ptr);

        if (ptr == 0) {
            param.p = schul_param.p[0];
            param.i = schul_param.i[0];
            param.d = schul_param.d[0];
            return;
        }

        if (ptr == schul_param.cases_num())
        {
            param.p = schul_param.p[schul_param.cases_num()-1];
            param.i = schul_param.i[schul_param.cases_num()-1];
            param.d = schul_param.d[schul_param.cases_num()-1];
            return;
        }

        //Mix ptr - 1 and ptr

        double d = schul_param.v_list[ptr] - schul_param.v_list[ptr-1];
        double r1 = x - schul_param.v_list[ptr - 1];
        
        param.p = float_constrain((d-r1)/d * schul_param.p[ptr-1] + r1/d * schul_param.p[ptr], 0, 100);
        param.i = float_constrain((d-r1)/d * schul_param.i[ptr-1] + r1/d * schul_param.i[ptr], 0, 100);
        param.d = float_constrain((d-r1)/d * schul_param.d[ptr-1] + r1/d * schul_param.d[ptr], 0, 100);

        // printf("Lower case is %f P %f, Upper is %f P %f\n", schul_param.v_list[ptr - 1], schul_param.p[ptr - 1], schul_param.v_list[ptr], schul_param.p[ptr]);
        // printf("x %f r1 %f d %f p final %f\n", x, r1, d, param.p);
    } 

    virtual inline double control(const double x, const double & err, double dt, bool report=false) {
        this->calc_pid(x);
        // printf("V is %2.1f, PID %3.1f %3.1f %3.1f\n", x, param.p, param.i, param.d);
        
        PIDController::control(err, dt, report);
    }
    
};


class RotorThrustControl {
    RotorThrustControlParam param;
    PIDController con;
public:
    double acc = 0;

    RotorThrustControl(RotorThrustControlParam _param):
        param(_param), con(_param.abx) {
        //TODO:
    }

    //We using NED in our control system
    void set_acc(double _acc) {
        acc = _acc;
    }

    void reset() {
        con.reset();
    }

    inline double control(const double & abx_sp, double dt) {
        return con.control(abx_sp - acc, dt) + param.level_thrust;
    }
};


//This position control output world accerelation
class RotorPositionControl {
    RotorPosCtrlParam param;
    PIDController px_con, py_con, pz_con;
    PIDController vx_con, vy_con, vz_con;


    Eigen::Quaterniond yaw_transverse;

    bool pos_inited = false;
    bool vel_inited = false;
    bool acc_inited = false;
    bool att_inited = false;

    Eigen::Vector3d euler_rpy = Eigen::Vector3d(0, 0, 0);

public:
    RotorThrustControl thrust_ctrl;

    Eigen::Vector3d pos = Eigen::Vector3d(0, 0, 0);
    Eigen::Vector3d vel = Eigen::Vector3d(0, 0, 0);
    Eigen::Vector3d acc = Eigen::Vector3d(0, 0, 0);
    Eigen::Vector3d acc_global = Eigen::Vector3d(0, 0, 0);
    Eigen::Vector3d vel_body = Eigen::Vector3d(0, 0, 0);
    Eigen::Quaterniond quat = Eigen::Quaterniond(1, 0, 0, 0);


    RotorPositionControl(RotorPosCtrlParam _param):
        param(_param),
        px_con(_param.p_x), py_con(_param.p_y), pz_con(_param.p_z),
        vx_con(_param.v_x), vy_con(_param.v_y), vz_con(_param.v_z),
        thrust_ctrl(_param.thrust_ctrl) {

    }

    virtual void set_pos(const Eigen::Vector3d & _pos) {
        pos = _pos;
        pos_inited = true;
    }
    
    virtual void set_global_vel(const Eigen::Vector3d & _vel) {
        vel = _vel;
        vel_inited = true;

        vel_body = quat.inverse() * vel;
    }

    virtual void set_body_acc(const Eigen::Vector3d & _acc) {
        acc = _acc;

        thrust_ctrl.set_acc(_acc.z());

        if (att_inited)
        {
            acc_global = quat * acc;
        }
    }

    virtual void set_attitude(const Eigen::Quaterniond & _quat) {
        quat = _quat;

        euler_rpy = quat2eulers(quat);

        yaw_transverse = Eigen::AngleAxisd(euler_rpy.z(), Vector3d::UnitZ());

        att_inited = true;
    }

    virtual double control_pos_z(const double z_sp, double dt) {
            return float_constrain(pz_con.control(z_sp - pos.z(), dt), -10,3);
    }
    
    virtual Eigen::Vector3d control_pos(const Eigen::Vector3d & pos_sp, double dt) {
        Eigen::Vector3d vel_sp(0, 0, 0);
        if (pos_inited) {
            vel_sp.x() = float_constrain(px_con.control(pos_sp.x() - pos.x(), dt), -MAX_HORIZON_VEL, MAX_HORIZON_VEL);
            vel_sp.y() = float_constrain(py_con.control(pos_sp.y() - pos.y(), dt), -MAX_HORIZON_VEL, MAX_HORIZON_VEL);
            vel_sp.z() = control_pos_z(pos_sp.z(), dt);

            if (param.ctrl_frame == VEL_BODY_ACC_BODY) {
                vel_sp = quat.inverse() * vel_sp;
            }
        }

        return vel_sp;
    }

    //Issue on acc output!!!
    //Should not on *body*, should based on cooridnate transversed by yaw
    //Which means x and y is on planar

    virtual double control_vel_z(double vel_z_sp, double dt)
    {
        double one_g = GRAVITY;
        if (param.ctrl_frame == CTRL_FRAME::VEL_WORLD_ACC_WORLD)
        {
            one_g = 0;
        }
        double az = vz_con.control(vel_z_sp - vel.z(), dt);
        if(param.coor_sys == FRAME_COOR_SYS::FLU) {
            return az + one_g;
        } else if(param.coor_sys == FRAME_COOR_SYS::NED) {
           return  az - one_g;
        }
        return 0;
    }

    virtual Eigen::Vector3d control_vel(const Eigen::Vector3d & vel_sp, double dt, bool input_body_frame=true, bool output_body_frame=true) {
        Eigen::Vector3d acc_sp(0, 0, 0);
        if (vel_inited) {
            acc_sp.x() = float_constrain(vx_con.control(vel_sp.x() - vel.x(), dt), -MAX_HORIZON_ACC, MAX_HORIZON_ACC);
            acc_sp.y() = float_constrain(vy_con.control(vel_sp.y() - vel.y(), dt), -MAX_HORIZON_ACC, MAX_HORIZON_ACC);
            acc_sp.z() = float_constrain(control_vel_z(vel_sp.z(), dt), -MAX_VERTICAL_ACC, MAX_VERTICAL_ACC);

            if (param.ctrl_frame == CTRL_FRAME::VEL_WORLD_ACC_BODY) {
                acc_sp = yaw_transverse.inverse() * acc_sp;
            }
        }

        return acc_sp;    
    }
    virtual void reset() {
        vx_con.reset();
        vy_con.reset();
        vz_con.reset();

        px_con.reset();
        py_con.reset();
        pz_con.reset();

        thrust_ctrl.reset();
    }

    virtual AttiCtrlOut control_acc(Eigen::Vector3d acc_sp, YawCMD yaw_cmd, double dt, double yaw_now) {
        AttiCtrlOut ret;
        
        double pitch_sp = 0;
        double roll_sp = 0;
        double yaw_sp = euler_rpy.z();
        
        if (param.ctrl_frame == CTRL_FRAME::VEL_WORLD_ACC_WORLD) {
            //Convert to body frame first
            Eigen::Quaterniond yaw_transverse(Eigen::AngleAxisd(yaw_now, Vector3d::UnitZ()));
            acc_sp = yaw_transverse.inverse() * acc_sp;
        }

        if(param.coor_sys == FRAME_COOR_SYS::FLU) {
            acc_sp.z() = - acc_sp.z();
            acc_sp.y() = - acc_sp.y();
        }

        acc_sp.z() = acc_sp.z() - 9.8;
        
        // TODO:
        // Do not care about aerodynamics drag
        // Only for hover
        ret.abx_sp = acc_sp.norm();

        ret.thrust_sp = float_constrain(thrust_ctrl.control(ret.abx_sp, dt), MIN_THRUST, 1);

        if (fabs(acc_sp.z()) > 0.1) {
            pitch_sp = float_constrain(- asin(acc_sp.x() / acc_sp.norm()), -MAX_TILT_ANGLE, MAX_TILT_ANGLE);
            roll_sp = float_constrain(asin(acc_sp.y() /( acc_sp.norm() * cos(pitch_sp))), -MAX_TILT_ANGLE, MAX_TILT_ANGLE);
        }

        if (yaw_cmd.yaw_mode == YAW_MODE::YAW_MODE_LOCK) {
            yaw_sp = yaw_cmd.yaw_sp;
        }

        if (yaw_cmd.yaw_mode == YAW_MODE::YAW_MODE_RATE) {
            yaw_sp = yaw_cmd.yaw_sp * dt + yaw_sp;
        }

        // printf("accsp %3.2f %3.2f %3.2f\nsp p %3.2f r %3.2f y %3.2f\n",
            // acc_sp.x(), acc_sp.y(), acc_sp.z(),
            // pitch_sp, roll_sp, yaw_sp);
        ret.roll_sp = roll_sp;
        ret.pitch_sp = pitch_sp;
        ret.yaw_sp = yaw_sp;
        ret.thrust_mode = AttiCtrlOut::THRUST_MODE_THRUST;
        ret.atti_sp = Eigen::AngleAxisd(yaw_sp, Vector3d::UnitZ()) * Eigen::AngleAxisd(pitch_sp, Vector3d::UnitY()) * Eigen::AngleAxisd(roll_sp, Vector3d::UnitX());

        return ret;
    }  
};