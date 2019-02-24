#include <iostream>
#include <eigen3/Eigen/Dense>
#include "ceres/ceres.h"
#include <vector>
#include <algorithm>
#include <set>
#include <map>
#include <time.h>
#include <thread>  
#include <unistd.h>
#include "swarm_vo_fuse/swarm_types.hpp"

using ceres::CostFunction;
using ceres::Problem;
using ceres::Solver;
using ceres::Solve;
using ceres::SizedCostFunction;
using ceres::Covariance;

using namespace swarm;
using namespace Eigen;

typedef std::vector<Vector3d> vec_array;
typedef std::vector<Quaterniond> quat_array;

// #define USE_BIAS

class SwarmDistanceResidual : public CostFunction {
    Vector3d Anntenna; 

    inline void Zik_the_ik(int idi, Vector3d & Zik, double & thetaik, double const * Zxyzth) const {
        Zik.x() = 0;
        Zik.y() = 0;
        Zik.z() = 0;
        thetaik = 0;

        //What does this stand for?
        int i = id2index(idi);
        if (i >= 0) {
            Zik.x() = Zxyzth[i*4];
            Zik.y() = Zxyzth[i*4 + 1];
            Zik.z() = Zxyzth[i*4 + 2];
            thetaik = Zxyzth[i*4 + 3];
        }

    }


    static inline Matrix3d rho_mat(double th) {
        Matrix3d rho;
        rho <<  cos(th), sin(th), 0,
                -sin(th), cos(th), 0,
                0, 0, 1;

        return rho;
    }

    static inline Matrix3d partial_rho_by_theta(double th) {
        Matrix3d rho;
        rho <<  -sin(th), cos(th), 0,
                -cos(th), -sin(th), 0,
                0, 0, 0;

        return rho;
    }


    inline void Z_idji_theji(int idj, int idi, Vector3d& Zji, double&thetaji, double const * Zxyzth) const
    {
        double ztheik = 0;
        double zthejk = 0;
        Vector3d Zik(0, 0, 0);
        Vector3d Zjk(0 ,0, 0);
        Zik_the_ik(idi, Zik, ztheik, Zxyzth);
        Zik_the_ik(idj, Zjk, zthejk, Zxyzth);
        
        Matrix3d rho_inv = rho_mat(-ztheik);
        Zji = rho_inv*(Zjk - Zik);
        thetaji = zthejk - ztheik;
    }

    

    inline Eigen::Vector3d partial_Zji_by_Zdelta_k(int idj, int idi, int id_delta, int m, double const * Zxyzth) const {

        double ztheik = 0;
        double zthejk = 0;
        Vector3d Zik(0, 0, 0);
        Vector3d Zjk(0, 0 ,0);

        Zik_the_ik(idi, Zik, ztheik, Zxyzth);
        Zik_the_ik(idj, Zjk, zthejk, Zxyzth);
        Vector3d ret(0,0,0);

        if (id_delta == idi) {
            if(m == 3) {// Is theta axis 
                Eigen::Matrix3d prhoT = partial_rho_by_theta(ztheik).transpose();
                return prhoT * (Zjk - Zik);
            }
            else {
                ret(m) = 1;
            }
            return -rho_mat(-ztheik)*ret;
        }

        if (id_delta == idj) {
            if (m==3) {
                return Eigen::Vector3d(0,0,0);
            }
            else {
                ret(m) = 1;
            }
            return rho_mat(-ztheik)*ret;
        }
        return Eigen::Vector3d{0, 0, 0};
    }


    inline double Jacobian_y_idij_by_Z_delta_m(int idi, int idj, int iddelta, int m, const Eigen::Vector3d &_rel_dir,
                                               double const *Zxyzth) const {
        if(iddelta != idi && iddelta != idj)
            return 0;
        double Ztheji = 0;
        Eigen::Vector3d Zji(0, 0 ,0);
        Z_idji_theji(idj, idi, Zji, Ztheji, Zxyzth);
        Eigen::Vector3d ret = partial_Zji_by_Zdelta_k(idj, idi, iddelta, m, Zxyzth);
        if (m==3)
        {
            if (idi == iddelta)
            {
                //Conver
                ret =  ret - partial_rho_by_theta(Ztheji) *  (get_drone_self_pos(idj) + get_drone_self_quat(idj) * Anntenna);
            }

            if (idj == iddelta)
            {
                ret =  ret + partial_rho_by_theta(Ztheji) * (get_drone_self_pos(idj) + get_drone_self_quat(idj) * Anntenna);
            }
        }
        return _rel_dir.dot(ret);
    }

    void put_jacobian(int y, int id, int m, double v, double *jaco) const {
        put_jacobian(y, id2index(id) * 4 + m, v, jaco);
    }

    void put_jacobian(int y, int x, double v, double *jaco) const {
        jaco[y * num_params() + x] = v;
    }

    virtual bool Evaluate(double const* const* _Zxyzth, double* residual, double** jacobians) const {

        double const * Zxyzth = *_Zxyzth;

        int count = 0;

        bool need_jacobians = jacobians != NULL && jacobians[0] != NULL;
        //Set jacobian all zero
        // printf("Jacobian size will be %d of double\n",  num_params() * num_residuals() );
        if (jacobians != NULL && jacobians[0] != NULL) {
            memset(jacobians[0], 0 , sizeof(double) * num_params() * num_residuals());
        }

        double *jaco = jacobians[0];

        auto nodes = swarm_frame.node_id_list;
        for (unsigned int i = 0; i < nodes.size(); i++)
        {
            int idi = nodes[i];
            if (!swarm_frame.HasUWB(idi)) {
                continue;
            }
            for (unsigned int j = i + 1; j < nodes.size(); j++)
            {
                int idj = nodes[j];

                if (!swarm_frame.HasDis(idj, idi) &&
                    !swarm_frame.HasDis(idi, idj))
                    continue;

                Eigen::Vector3d _rel = distance_idj_idi(idj, idi, Zxyzth);
                // Because dis_matrix(i,j) != dis_matrix(j, i), so we use average instead

                double d_bar = 0;
                if (swarm_frame.HasDis(idj, idi) && swarm_frame.HasDis(idi, idj)) {
                    d_bar = (dis_measure_idj2i(idi, idj) + dis_measure_idj2i(idj, idi)) * 0.5;
                } else if (swarm_frame.HasDis(idj, idi)) {
                    d_bar = dis_measure_idj2i(idj, idi);
                } else if (swarm_frame.HasDis(idi, idj)) {
                    d_bar = dis_measure_idj2i(idi, idj);
                }

#ifdef USE_BIAS
                residual[count] = (_rel.norm() - d_bar - bias_idi_idj(i, j, Zxyzth));
#else
                residual[count] = (_rel.norm() - d_bar);
#endif
                // printf("Bias %d %d %f\n", i, j, bias_ij(i, j, Zxyzth));

                if (need_jacobians) {
                    for (int m =0; m<4 ; m++) {
                        double jac_im = Jacobian_y_idij_by_Z_delta_m(idi, idj, idi, m, _rel / _rel.norm(), Zxyzth);
                        // jacobians[0][count*num_params() + id2index(idi)*4+m] = jac_im;
                        put_jacobian(count, idi, m, jac_im, jaco);

                        double jac_jm = Jacobian_y_idij_by_Z_delta_m(idi, idj, idj, m, _rel / _rel.norm(), Zxyzth);
                        // jacobians[0][count*num_params() + id2index(idj)*4+m] = jac_jm;
                        put_jacobian(count, idj, m, jac_jm, jaco);

                    }
                }

                count ++;
            }
        }
        
        //Jacbian for bias is always -1
        //i array
        //The question is when drone num changes, we can't still use last Zxyth

        //TODO:
        //May has issue when drone_num < drone_num_total
        //??????????
#ifdef USE_BIAS
        if (need_jacobians ) {
            for (int i = 0; i < count; i++) {
                // printf("drone_num %d %d :%d\n",drone_num_total,drone_num_total * (drone_num_total - 1) /2, i);
                put_jacobian(i, num_params() + i, -1 )
            }
        }
#endif
        return true;
    }
public:
    SwarmDistanceResidual(const SwarmFrame &_sf, std::vector<int> &_all_nodes, std::map<int, int> &_id_index_map) :
            swarm_frame(_sf), all_nodes(_all_nodes), id_index_map(_id_index_map)
        {
            int drone_num_now = swarm_frame.id2nodeframe.size();
            set_num_residuals(drone_num_now * (drone_num_now-1) / 2);
            mutable_parameter_block_sizes()->push_back(num_params());
        }

    Vector3d est_id_pose_in_k(int idj, int idi, double const * Zxyzth, bool estimate_antenna = false) const
    {
        double Ztheji = 0;
        Eigen::Vector3d Zji(0, 0, 0);

        Z_idji_theji(idj, idi, Zji, Ztheji, Zxyzth);
        
        Matrix3d rho_ji = rho_mat(Ztheji);
        Vector3d  _rel(0,0,0);

        if (estimate_antenna)
        {
            _rel =  Zji + rho_ji * (get_drone_self_pos(idj) + get_drone_self_quat(idj) * Anntenna);
        }
        else {
            _rel = Zji + rho_ji * get_drone_self_pos(idj);
        }
        return _rel;
    }

    Vector3d est_id_vel_in_k(int idj, int idi, double const * Zxyzth) const
    {
        double Ztheji = 0;
        Eigen::Vector3d Zji(0, 0, 0);

        Z_idji_theji(idj, idi, Zji, Ztheji, Zxyzth);
        
        Matrix3d rho_ji = rho_mat(Ztheji);

        Vector3d  _rel = rho_ji * get_drone_self_vel(idj);

        return _rel;
    }

    Quaterniond est_id_quat_in_k(int idj, int idi, double const * Zxyzth) const {
        double Ztheji = 0;
        Eigen::Vector3d Zji(0, 0, 0);

        Z_idji_theji(idj, idi, Zji, Ztheji, Zxyzth);

        Quaterniond _quat = AngleAxisd(-Ztheji, Vector3d::UnitZ()) * get_drone_self_quat(idj);
        return _quat;
    }
    
    inline Eigen::Vector3d distance_idj_idi(int idj, int idi, double const* Zxyzth) const {
        Eigen::Vector3d  _rel = est_id_pose_in_k(idj, idi, Zxyzth, true) - (get_drone_self_pos(idi) + get_drone_self_quat(idi) * Anntenna);
        return _rel;
    }

    inline double bias_idi_idj(int idi, int idj, double const * Zxyzth) const {
        int i = id2index(idi);
        int j = id2index(idj);
        return bias_ij(i, j, Zxyzth);
    }

    inline double bias_ij(int i, int j, double const * Zxyzth) const {
        if (i == j)
            return 0;
        
        if (i > j)
        {
            int tmp = j;
            j = i;
            i = tmp;
        }

        int n = drone_num_total();

        int no_of_bias = (2*n - 1 - i)*i/2 + j - i - 1;

        // printf("bias %d %d index %d:%d\n", i, j,no_of_bias, (drone_num_total -1 )*4+ no_of_bias);
        return Zxyzth[(drone_num_total() - 1) * 4 + no_of_bias];
    }


private:

    inline Eigen::Vector3d get_drone_self_pos(int _id) const {
        return swarm_frame.position(_id);
    }

    inline Eigen::Vector3d get_drone_self_vel(int _id) const {
        return swarm_frame.velocity(_id);
    }

    inline Eigen::Quaterniond get_drone_self_quat(int _id) const {
        return swarm_frame.attitude(_id);
    }

    int id2index(int _id) const {
        return id_index_map.at(_id);
    }

    int index2id(int index) const {
        assert((unsigned int) index < all_nodes.size() && "A is not equal to B");
        return all_nodes.at(index);
    }

    inline double dis_measure_idj2i(int idj, int idi) const {
        return swarm_frame.distance(idj, idi);
    }

    SwarmFrame swarm_frame;

    std::vector<int> &all_nodes;
    std::map<int, int> &id_index_map;

    int drone_num_total() const {
        return all_nodes.size();
    }

    int num_params() const //This is number of parameters except bias
    {
        return (drone_num_total() - 1) * 4 + (drone_num_total() - 1) * drone_num_total() / 2;
    }

    int frame_node_num() const
    {
        return swarm_frame.id2nodeframe.size();
    }
};
