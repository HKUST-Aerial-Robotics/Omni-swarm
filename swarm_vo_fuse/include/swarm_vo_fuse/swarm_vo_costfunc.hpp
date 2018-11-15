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

using ceres::CostFunction;
using ceres::Problem;
using ceres::Solver;
using ceres::Solve;
using ceres::SizedCostFunction;
using ceres::Covariance;

using namespace Eigen;

typedef std::vector<Vector3d> vec_array;
typedef std::vector<Quaterniond> quat_array;


class SwarmDistanceResidual : public CostFunction {
    Vector3d Anntenna; 
    inline int param_index(int i) const
    {
        //Convert index of matrix and vector to id
        int j= ids[i];

        //Conver id to param index
        int k = id_to_index.at(j);

        return k - 1;
    }

    inline void Zik_the_ik(int i, Vector3d& Zik, double&thetaik, double const * Zxyzth) const
    {
        Zik.x() = 0;
        Zik.y() = 0;
        Zik.z() = 0;
        thetaik = 0;
        i = param_index(i);
        if (i >= 0)
        {
            Zik.x() = Zxyzth[i*4];
            Zik.y() = Zxyzth[i*4 + 1];
            Zik.z() = Zxyzth[i*4 + 2];
            thetaik = Zxyzth[i*4 + 3];
        }

    }

    inline double bias_ij(int i, int j)
    {
        if (i == j)
            return 0;
        
        if (i > j)
        {
            int tmp = j;
            j = i;
            i = tmp;
        }

        int no_of_bias = (2*n - 1 - i)*i/2 + j - i - 1;
        return Zxyzth[drone_num_total +   no_of_bias ]
    }

    // inline double bias_ij(doubl)

    inline Matrix3d rho_mat(double th) const
    {
        Matrix3d rho;
        rho <<  cos(th), sin(th), 0,
                -sin(th), cos(th), 0,
                0, 0, 1;

        return rho;
    }

    inline Matrix3d partial_rho_by_theta(double th) const
    {
        Matrix3d rho;
        rho <<  -sin(th), cos(th), 0,
                -cos(th), -sin(th), 0,
                0, 0, 0;

        return rho;
    }


    inline void Zji_theji(int j, int i, Vector3d& Zji, double&thetaji, double const * Zxyzth) const
    {
        double ztheik = 0;
        double zthejk = 0;
        Vector3d Zik;
        Vector3d Zjk;
        Zik_the_ik(i, Zik, ztheik, Zxyzth);
        Zik_the_ik(j, Zjk, zthejk, Zxyzth);
        
        Matrix3d rho_inv = rho_mat(-ztheik);
        Zji = rho_inv*(Zjk - Zik);
        thetaji = zthejk - ztheik;
    }

    

    inline Eigen::Vector3d partial_Zji_by_Zdelta_k(int j, int i, int delta, int m, double const * Zxyzth) const
    {

        double ztheik = 0;
        double zthejk = 0;
        Vector3d Zik;
        Vector3d Zjk;
        Zik_the_ik(i, Zik, ztheik, Zxyzth);
        Zik_the_ik(j, Zjk, zthejk, Zxyzth);
        Vector3d ret(0,0,0);

        if (delta == i)
        {
            if(m == 3) // Is theta axis
            {
                Eigen::Matrix3d prhoT = partial_rho_by_theta(ztheik).transpose();
                return prhoT * (Zjk - Zik);
            }
            else
            {
                ret(m) = 1;
            }
            return -rho_mat(-ztheik)*ret;
        }

        if (delta == j)
        {
            if (m==3)
            {
                return Eigen::Vector3d(0,0,0);
            }
            else{
                ret(m) = 1;
            }
            return rho_mat(-ztheik)*ret;
        }
        return Eigen::Vector3d(0,0,0);
    }


    inline double Jacobian_y_ij_by_Z_delta_m(int i, int j, int delta, int m, Eigen::Vector3d _rel_dir, double const * Zxyzth) const
    {
        if(delta != i && delta != j )
            return 0;
        double Ztheji = 0;
        Eigen::Vector3d Zji;
        Zji_theji(j, i, Zji, Ztheji, Zxyzth);
        Eigen::Vector3d ret = partial_Zji_by_Zdelta_k(j, i, delta, m, Zxyzth);
        if (m==3)
        {
            if (i == delta)
            {
                ret =  ret - partial_rho_by_theta(Ztheji) *  (self_pos[j] + self_quat[j] * Anntenna);
            }

            if (j == delta)
            {
                ret =  ret + partial_rho_by_theta(Ztheji) * (self_pos[j] + self_quat[j] * Anntenna);
            }
        }
        return _rel_dir.dot(ret);
    }


    virtual bool Evaluate(double const* const* _Zxyzth, double* residual, double** jacobians) const {

        double const * Zxyzth = *_Zxyzth;

        int count = 0;
        int drone_num_now = this->drone_num();


        //Set jacobian all zero
        // printf("Jacobian size will be %d of double\n",  num_params() * num_residuals() );
        if (jacobians != NULL && jacobians[0] != NULL) {
            memset(jacobians[0], 0 , sizeof(double) * num_params() * num_residuals());
        }

        int residual_num = drone_num_now * (drone_num_now - 1) / 2;

        for (int i = 0; i < drone_num_now; i++)
        {
            for (int j = i + 1; j < drone_num_now ; j++)
            {


                Eigen::Vector3d _rel = distance_j_i(j, i, Zxyzth);
                // Because dis_matrix(i,j) != dis_matrix(j, i), so we use average instead
                double d_bar = (dis_matrix(i,j) + dis_matrix(j,i)) * 0.5; 
                residual[count] = (_rel.norm() - d_bar - bias_ij(i, j));

                if (jacobians != NULL && jacobians[0] != NULL) {
                    for (int m =0; m<4 ; m++)
                    {
                        if (param_index(i) >= 0)
                        {
                            double jac_im = Jacobian_y_ij_by_Z_delta_m(i, j, i, m, _rel/_rel.norm(), Zxyzth);
                            jacobians[0][count*num_params() + param_index(i)*4+m] = jac_im;
                        }

                        if (param_index(j) >= 0)
                        {
                            double jac_jm = Jacobian_y_ij_by_Z_delta_m(i, j, j, m, _rel/_rel.norm(), Zxyzth);
                            jacobians[0][count*num_params() + param_index(j)*4+m] = jac_jm;
                        }

                    }
                }

                count ++;
            }
        }


        //Jacbian for bias is always -1
        //i array
        //The question is when drone num changes, we can't still use last Zxyth
        for (int i = 0; i < residual_num; i++)
        {
            jacobians[0][(drone_num_total-1)*4 + i] = -1;
        }
        return true;
    }
public:
    SwarmDistanceResidual(const Eigen::MatrixXd & _dis_matrix,
                const vec_array & _self_pos, 
                const vec_array & _self_vel, 
                const quat_array & _self_quat, 
                const std::vector<unsigned int>& _ids,
                std::map<int, int> _id2index,
                Eigen::Vector3d anntena_pos,
                int _drone_num_total
                ): 
            dis_matrix(_dis_matrix), 
            self_vel(_self_vel), 
            self_pos(_self_pos), 
            self_quat(_self_quat), 
            id_to_index(_id2index), 
            ids(_ids),
            Anntenna(anntena_pos),
            drone_num_total(_drone_num_total)
        {

            int drone_num_now = _ids.size();
            set_num_residuals(drone_num_now * (drone_num_now-1) / 2);
            mutable_parameter_block_sizes()->push_back(num_params());
        }

    Vector3d est_id_pose_in_k(int j, int i, double const * Zxyzth, bool estimate_antenna = false) const
    {
        double Ztheji = 0;
        Eigen::Vector3d Zji;
        Zji_theji(j, i, Zji, Ztheji, Zxyzth);
        
        Matrix3d rho_ji = rho_mat(Ztheji);
        Vector3d  _rel(0,0,0);
        if (estimate_antenna)
        {
            _rel =  Zji + rho_ji * (self_pos[j] + self_quat[j] * Anntenna);
        }
        else {
            _rel = Zji + rho_ji * self_pos[j];
        }
        return _rel;
    }

    Vector3d est_id_vel_in_k(int j, int i, double const * Zxyzth) const
    {
        double Ztheji = 0;
        Eigen::Vector3d Zji;
        Zji_theji(j, i, Zji, Ztheji, Zxyzth);
        
        Matrix3d rho_ji = rho_mat(Ztheji);
        Vector3d  _rel = rho_ji * self_vel[j];

        return _rel;
    }

    Quaterniond est_id_quat_in_k(int j, int i, double const * Zxyzth) const
    {
        double Ztheji = 0;
        Eigen::Vector3d Zji;
        Zji_theji(j, i, Zji, Ztheji, Zxyzth);
        
        Quaterniond _quat = AngleAxisd(-Ztheji, Vector3d::UnitZ()) * self_quat[j];
        return _quat;
    }
    
    inline Eigen::Vector3d distance_j_i(int j, int i, double const* Zxyzth) const
    {
        Eigen::Vector3d  _rel = est_id_pose_in_k(j, i, Zxyzth, true) - (self_pos[i] + self_quat[i] * Anntenna);
        return _rel;
    }

private:
    Eigen::MatrixXd dis_matrix;
    vec_array self_pos;
    vec_array self_vel;
    quat_array self_quat;

    std::vector<unsigned int> ids;
    std::map<int, int> id_to_index;


    int drone_num_total = -1;
    int num_params() const
    {
        return (id_to_index.size()-1)*4 + (id_to_index.size()-1) * id_to_index.size() / 2;
    }
    int drone_num() const
    {
        return ids.size();
    }
};
