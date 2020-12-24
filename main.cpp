//
// Created by chengdinga on 2020/8/5.
//
#include <iostream>
#include "joint.h"
#include <vector>
#include "configDir.h"
#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/info_parser.hpp>
#include <boost/property_tree/json_parser.hpp>


std::fstream target_joint_position_file;
std::fstream actual_joint_position_file;
std::fstream target_joint_velocity_file;
std::fstream actual_joint_velocity_file;
std::fstream target_joint_torque_file;
std::fstream actual_joint_torque_file;
std::fstream error_file;
std::fstream DC_time_file;


int main()
{
    std::string dataDir = "/home/cda/code/examples/zero_conf/test_data";
    target_joint_position_file.open(dataDir + "/target_joint_position.csv", std::ios::out);
    actual_joint_position_file.open(dataDir + "/actual_joint_position.csv", std::ios::out);
    target_joint_velocity_file.open(dataDir + "/target_joint_velocity.csv", std::ios::out);
    actual_joint_velocity_file.open(dataDir + "/actual_joint_velocity.csv", std::ios::out);
    target_joint_torque_file.open(dataDir + "/target_joint_torque.csv", std::ios::out);
    actual_joint_torque_file.open(dataDir + "/actual_joint_torque.csv", std::ios::out);
    std::string table_head = "1,2,3,4,5,6,7,8,9,10,11,12,time\n";
    target_joint_position_file << table_head;
    actual_joint_position_file << table_head;
    target_joint_velocity_file << table_head;
    actual_joint_velocity_file << table_head;
    target_joint_torque_file << table_head;
    actual_joint_torque_file << table_head;
     // 初始化代码 start
    double kp = 66;
    double kd = 6;
    Vector3d rf_theta;
    Vector3d rh_theta;
    Vector3d lh_theta;
    Vector3d lf_theta;

    Vector3d rf_foot_end;
    Vector3d rh_foot_end;
    Vector3d lh_foot_end;
    Vector3d lf_foot_end;

    Vector3d rf_target_q;
    Vector3d rh_target_q;
    Vector3d lh_target_q;
    Vector3d lf_target_q;
    Eigen::Matrix<double, 12, 1> initial_q_j;
    Eigen::Matrix<double, 12, 1> target_q_j;
    Eigen::Matrix<double, 12, 1> target_q_dot_j;
    Eigen::Matrix<double, 12, 1> actual_q_j;
    Eigen::Matrix<double, 12, 1> actual_q_dot_j;
    Eigen::Matrix<double, 12, 1> original_q_j;
    Eigen::Matrix<double, 12, 1> target_u;
    for (int i=0;i<12;i++)
    {
        original_q_j(i, 0) = joints[i]->getActualAngular();
    }
    rf_foot_end << 0, -0.050, -0.20;
    rh_foot_end << 0, -0.050, -0.20;
    lh_foot_end << 0, 0.050, -0.20;
    lf_foot_end << 0, 0.050, -0.20;
    inverse_kinematics(rf_target_q, rf_foot_end, -1);
    inverse_kinematics(rh_target_q, rh_foot_end, -1);
    inverse_kinematics(lh_target_q, lh_foot_end,  1);
    inverse_kinematics(lf_target_q, lf_foot_end,  1);

    initial_q_j.block<3, 1>(0, 0) = rf_target_q;
    initial_q_j.block<3, 1>(3, 0) = rh_target_q;
    initial_q_j.block<3, 1>(6, 0) = lh_target_q;
    initial_q_j.block<3, 1>(9, 0) = lf_target_q;

    auto initial_traj = ct::core::linspace<ct::core::StateVectorArray<12, double>>(original_q_j, initial_q_j, 5000);
    target_q_dot_j = (initial_q_j - original_q_j) / 5.0;
    target_q_j = original_q_j;
    // 初始化代码 end
    
    for (int i=0;i<12;i++)
    {
        joints[i]->enable(true);

    }

    int64 last_DCtime = 0;
    int joint_num = 0;
    int DC_TimeErrorCode = 0;
    
    /* acyclic loop 1000ms */
    while(true) {
        sem_wait(&bin_sem);
        if(last_DCtime != 0) 
        {
            if (ec_DCtime - last_DCtime < (1000000 - 100000))
            {
                DC_TimeErrorCode = -1;
            }
            
            else if (ec_DCtime - last_DCtime > (1000000 + 100000))
            {
                DC_TimeErrorCode = -2;
            }
            else 
            {
                DC_TimeErrorCode = 0;
            }
           
        }
        last_DCtime = ec_DCtime;


        // 输出DC_time错误到error文件
        if (DC_TimeErrorCode == -1)
        {
            
            error_file << "----------------DC time ERROR!------------------\n";
            error_file << "DC time 间隔过小\n";
            error_file << "last DC time = " << last_DCtime;
            error_file << "now DC time = " << ec_DCtime;
            DCtimeErrorTimes ++;
            error_file << "----------------DC time ERROR!------------------\n";
        }
        if (DC_TimeErrorCode == -2)
        {
            error_file << "----------------DC time ERROR!------------------\n";
            error_file << "DC time 间隔过大\n";
            error_file << "last DC time = " << last_DCtime << std::endl;
            error_file << "now DC time = " << ec_DCtime << std::endl;
            DCtimeErrorTimes ++;
            error_file << "----------------DC time ERROR!------------------\n";
        }
        else 
        {

        }

        // user code
        time = cnt * 0.001;

        for(int i=0;i<12;i++)
        {
            actual_q_j(i) = joints[i]->getActualAngular();
            actual_q_dot_j(i) = joints[i]->getActualVelocity();
        }
        


        std::cout << ec_DCtime << std::endl;
        

        // 时间
        if (cnt < 5000)
        {
            // std::cout << "进入if" << std::endl;
            target_q_j = initial_traj[cnt];
            // std::cout << "target_q_j = " << target_q_j.transpose() << std::endl;
        }
        else
        {
            target_q_j = initial_q_j;
            target_q_dot_j.setZero();
        }



        // 记录出错
        for (auto i: input_data) {
            if ((i->status_word & 0x004fU) == 0x0008U) {
                error_file << "---------- elmo fault start ----------" << std::endl;
                error_file << "elmo fault code: " << std::endl;
                error_file << "---------- elmo fault end ----------" << std::endl;
            }
        }


        // 计算力矩值
        for(int leg=0;leg<4;leg++)
        {
            for (int leg_joint=0;leg_joint < 3; leg_joint ++)
            {
                joint_num = leg * 3  + leg_joint;
                target_u(joint_num) = (target_q_j(joint_num) - actual_q_j(joint_num)) * kp +
                        (target_q_dot_j(joint_num) - actual_q_dot_j(joint_num)) * kd;
            }
        }


        

        // 发送关节力矩
        for (int i=0;i<12;i++)
        {
            joints[i]->torque_cmd(0);
        }


        // 输出数据到文件
        for (int i=0;i<12;i++)
        {
            target_joint_position_file << target_q_j(i) << ",";
            actual_joint_position_file << actual_q_j(i) << ",";
            target_joint_velocity_file << target_q_dot_j(i) << ",";
            actual_joint_velocity_file << actual_q_dot_j(i) << ",";
            target_joint_torque_file << target_u(i) << ",";
            actual_joint_torque_file << joints[i]->getActualTorque() << ",";
        }
        target_joint_position_file << time << std::endl;
        actual_joint_position_file << time << std::endl;
        target_joint_velocity_file << time << std::endl;
        actual_joint_velocity_file << time << std::endl;
        target_joint_torque_file << time << std::endl;
        actual_joint_torque_file << time << std::endl;


        
        // check if get the end program signal
        cnt++;
    }

    target_joint_position_file.close();
    actual_joint_position_file.close();
    target_joint_velocity_file.close();
    actual_joint_velocity_file.close();
    target_joint_torque_file.close();
    actual_joint_torque_file.close();
    DC_time_file.close();
    return 0;
}