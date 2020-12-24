
#include "soem.h"
#include "my_type.h"
#include "motor.h"
#include "joint.h"
#include "transform.h"
#include "io.h"
#include <ct/core/core.h>
#include <ct/rbd/rbd.h>
#include <chrono>
#include <kindr/Core>
#include     <termios.h>
#include <iostream>
#include <stdlib.h>
#include <sys/time.h>
#include <unistd.h>
#include <sched.h>
#include <string.h>
#include <time.h>
#include <pthread.h>
#include <signal.h>
#include <unistd.h>
#include <stdlib.h>
#include <ethercat.h>
#include <Eigen/Eigen>
#include     <sys/types.h>
#include     <sys/stat.h>
#include     <fcntl.h>
#include     <termios.h>
#include <kindr/Core>
#include     <errno.h>
#include <semaphore.h>
#include <ct/core/core.h>
#include "queue.h"
#include "pegasus2.h"

#include "dir.h"
bool end_program = false;
void ctrl_c_function(int sig);

/*
 * User define start
*/
#define NSEC_PER_SEC 1000000000

// sample interval in ns, here 8us -> 125kHz
// maximum data rate for E/BOX v1.0.1 is around 150kHz
#define SYNC0TIME 8000
/*
 * User define end
*/
constexpr double pi = 3.141592653589793;

constexpr double hip_length = 0.050;
constexpr double upperleg_length = 0.38;
constexpr double lowerleg_length = 0.3615;
int DCtimeErrorTimes = 0;
static double sign(const double & x)
{
    if (x < 0)
    {
        return -1;
    }
    else if (x > 0)
    {
        return 1;
    }
    else 
    {
        return 0;
    }
}
static int elmo_count = 0;
char IOmap[4096];
int dorun = 0;
sem_t bin_sem;
int64 time_DC = 0;

std::fstream target_joint_position_file;
std::fstream actual_joint_position_file;
std::fstream target_joint_velocity_file;
std::fstream actual_joint_velocity_file;
std::fstream target_joint_torque_file;
std::fstream actual_joint_torque_file;
std::fstream error_file;
std::fstream DC_time_file;
std::fstream ground_force_file;
std::fstream grf_estimation_file;
pthread_t imu_thread;

bool read_imu = true;

kindr::EulerAnglesZyx<double> actual_imu_data;
ct::core::StateVector<3> base_acc;
ct::core::StateVector<3> base_angular_vel;





pthread_cond_t  cond  = PTHREAD_COND_INITIALIZER;
pthread_mutex_t mutex = PTHREAD_MUTEX_INITIALIZER;
int64 integral=0;
uint32 cyclecount = 0;

// user typedef

typedef elmo::Motor Motor_t;
typedef Motor_t * MotorPtr_t;
typedef elmo::Joint Joint_t;
typedef Joint_t * JointPtr_t;
typedef ct::rbd::pegasus2::Kinematics::Position3Tpl FootEndPosition;
using Vector3d =  Eigen::Matrix<double, 3, 1>;

static void inverse_kinematics(Vector3d & theta, const FootEndPosition & footend, const double & side)
{
    theta(0) = side * sign(footend(2)) * acos(hip_length/sqrt(footend(1)*footend(1)+footend(2)*footend(2))) -  atan(footend(2)/footend(1));
    double y_tmp = footend(2)*cos(theta(0)) +footend(1)*sin(theta(0));
    double x_tmp = -footend(0);
    double alpha = atan2(y_tmp, x_tmp);
    double phi = acos ((upperleg_length*upperleg_length + footend(0)*footend(0) + y_tmp*y_tmp - lowerleg_length* lowerleg_length)/(2 * hypot(x_tmp, y_tmp) * upperleg_length));
    theta(1) = -(alpha + phi + pi/2);
    theta(2) = pi - asin(sin(phi) * hypot(x_tmp, y_tmp) / lowerleg_length);
}


void IMU_Data_Receive_Anl(uint8 read_buf1[50],uint8 num);

/////////////////////////////////////////////////////////////////////////////////////
//Data_Receive_Prepare函数是协议预解析，根据协议的格式，将收到的数据进行一次格式性解析，格式正确的话再进行数据解析
//移植时，此函数应由用户根据自身使用的通信方式自行调用，比如串口每收到一字节数据，则调用此函数一次
//此函数解析出符合格式的数据帧后，会自行调用数据解析函数
void IMU_Data_Receive_Prepare(uint8 data)
{
    static uint8 RxBuffer[200];
    static uint8 _data_len = 50,_data_cnt = 0;//注意协议包里的 数据长度设定
    static uint8 state = 0;

    if(state==0&&data==0xFA)
    {
        state=1;
        RxBuffer[0]=data;
    }
    else if(state==1&&data==0xFF)
    {
        state=2;
        RxBuffer[1]=data;
    }
    else if(state==2&&data==0x36)
    {
        state=3;
        RxBuffer[2]=data;
    }
    else if(state==3&&data==0x2d)//IMU 配置的有效数据长度 （除去帧头）
    {
        state = 4;
        RxBuffer[3]=data;
        _data_len = 46;//data-4;
        _data_cnt = 0;
    }
    else if(state==4&&_data_len>0)
    {
        _data_len--;
        RxBuffer[4+_data_cnt++]=data;
        if(_data_len==0)
        {
            state = 0;
            IMU_Data_Receive_Anl(RxBuffer,_data_cnt+4);
        }
    }
    else
        state = 0;
    //pthread_mutex_unlock(&mutex_imu);

}

/////////////////////////////////////////////////////////////////////////////////////
//IMU_Data_Receive_Anl函数是 IMU-300 协议数据解析函数，函数参数是符合协议格式的一个数据帧，该函数会首先对协议数据进行校验
//校验通过后对数据进行解析，实现相应功能
//此函数可以不用用户自行调用，由函数IMU_Data_Receive_Prepare自动调用
void IMU_Data_Receive_Anl(uint8 * read_buf1 ,uint8 num)
{

    if(!(read_buf1[0]==0xFA && read_buf1[1]==0xFF && read_buf1[2]==0x36 && read_buf1[3]==0x2d))	 return;	 //判断帧头

    if(read_buf1[4]==0x20&&read_buf1[5]==0x30)
    {
        float Roll=0.0f;
        float Pitch=0.0f;
        float Yaw=0.0f;
        // int EulerAngles_size;
        // EulerAngles_size = read_buf1[6];
        unsigned char buf[]={read_buf1[10],read_buf1[9],read_buf1[8],read_buf1[7]};
        memcpy(&Roll,buf,4);
        actual_imu_data.setRoll(Roll * pi /180);


        unsigned char buf1[]={read_buf1[14],read_buf1[13],read_buf1[12],read_buf1[11]};
        memcpy(&Pitch,buf1,4);
        actual_imu_data.setPitch(Pitch * pi /180);


        unsigned char buf2[]={read_buf1[18],read_buf1[17],read_buf1[16],read_buf1[15]};
        memcpy(&Yaw,buf2,4);
        actual_imu_data.setYaw(Yaw * pi /180);


        if(read_buf1[19]==0x40&&read_buf1[20]==0x20)
        {
            // int Acc_size;
            // Acc_size = read_buf1[21];

            unsigned char buff[]={read_buf1[25],read_buf1[24],read_buf1[23],read_buf1[22]};
            float accX=0.0f;
            memcpy(&accX,buff,4);

            unsigned char buff1[]={read_buf1[29],read_buf1[28],read_buf1[27],read_buf1[26]};
            float accY=0.0f;
            memcpy(&accY,buff1,4);

            unsigned char buff2[]={read_buf1[33],read_buf1[32],read_buf1[31],read_buf1[30]};
            float accZ=0.0f;
            memcpy(&accZ,buff2,4);

            base_acc(0) = accX;
            base_acc(1) = accX;
            base_acc(2) = accX;


            if(read_buf1[34]==0x80&&read_buf1[35]==0x20)
            {
                // int Gyr_size;
                // Gyr_size = read_buf1[36];

                unsigned char bufff[]={read_buf1[40],read_buf1[39],read_buf1[38],read_buf1[37]};
                float gyrX=0.0f;
                memcpy(&gyrX,bufff,4);

                unsigned char bufff1[]={read_buf1[44],read_buf1[43],read_buf1[42],read_buf1[41]};
                float gyrY=0.0f;
                memcpy(&gyrY,bufff1,4);

                unsigned char bufff2[]={read_buf1[48],read_buf1[47],read_buf1[46],read_buf1[45]};
                float gyrZ=0.0f;
                memcpy(&gyrZ,bufff2,4);

                base_angular_vel(0) = gyrX;
                base_angular_vel(1) = gyrY;
                base_angular_vel(2) = gyrZ;
            }
        }
    }

}



void* com_read(void* pstatu)//IMU M-300
{



    unsigned char read_buf1[54];


    //串口初始化
    struct termios opt;
    int fd = open("/dev/ttyUSB0", O_RDWR | O_NOCTTY );    //默认为阻塞读方式
    if(fd == -1)
    {
        perror("Can't Open Serial Port - IMU\n");
        exit(0);
    }
    tcgetattr(fd, &opt);
    cfsetispeed(&opt, B115200);     //输入波特率
    cfsetospeed(&opt, B115200);     //输出波特率
    opt.c_cflag &= ~CSIZE;

    opt.c_cflag |= CS8;     //8位数据位

    opt.c_cflag |= CSTOPB;  //2位停止位

    opt.c_cflag &= ~PARENB;
    opt.c_iflag=0;
    opt.c_cflag |= (CLOCAL | CREAD);  	//Enable the receiver and set local mode
    opt.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);  //选择原始输入
    opt.c_oflag &= ~OPOST;  //选择原始输出

    opt.c_cc[VTIME] = 1;    //等待超时时间(等待几个0.1S)
    opt.c_cc[VMIN]  = 1;    //最小接收字符
    tcflush(fd, TCIOFLUSH);	//Flush input and output buffers and make the change
    printf("\n IMU - Configure Complete\n");
    if(tcsetattr(fd, TCSANOW, &opt) != 0)
    {
        perror("\nIMU -serial error\n");
        exit(0);
    }


    while(read_imu)
    {

        //time_in= get_time();

        fd_set fdset;
        FD_ZERO(&fdset);
        FD_SET(fd, &fdset);
        struct timeval tTime;
        tTime.tv_sec = 0;	tTime.tv_usec = 0;
        select(fd + 1, &fdset, NULL, NULL, &tTime);
        int isset = FD_ISSET(fd, &fdset);
        if(isset)
        {
            printf("isset is: %d\n",isset);
            while(read(fd,read_buf1,1)>0)
            {
                IMU_Data_Receive_Prepare(read_buf1[0]);
            }
            //else usleep(1983);
            usleep(1000);

        }
        else usleep(1983);

        usleep(50);//for other process
    }
    printf("close...-IMU \n");
    close(fd);
    /*恢复旧的通信端口参数*/
    //tcsetattr(fd,TCSANOW,&oldtio);
    return nullptr;
}

template<typename DataValueType>
class LegDataMap{
public:

    enum LegIdentifier{
        RF=0,
        RH,
        LH,
        LF
    };
    LegDataMap()= default;

    DataValueType & operator[](LegIdentifier which)
    {
        return this->data[which];
    }

protected:
    DataValueType data[4];
};



void control_loop()
{
    /* Init code start */

    std::vector<InputData_t *> input_data(elmo_count, nullptr);
    std::vector<OutputData_t *> output_data(elmo_count, nullptr);
    for(int i=0;i<12;i++)
    {
        input_data[i] = (InputData_t * )ec_slave[i+1].inputs;
        output_data[i] = (OutputData_t * )ec_slave[i+1].outputs;
    }

    std::vector<MotorPtr_t> motors(12, nullptr);
    std::vector<JointPtr_t> joints(12, nullptr);
    for(int i=0;i<12;i++)
    {
        motors[i] = new elmo::Motor(input_data[i], output_data[i]);
        joints[i] = new Joint_t(motors[i]);
    }
    
    // 设置关节配置目录路径
    elmo::Joint::config_dir = configDir + "/joint";

    // 加载关节配置文件
    joints[0]->loadConfigFile("RF.HAA");
    joints[1]->loadConfigFile("RF.HFE");
    joints[2]->loadConfigFile("RF.KFE");

    joints[3]->loadConfigFile("RH.HAA");
    joints[4]->loadConfigFile("RH.HFE");
    joints[5]->loadConfigFile("RH.KFE");

    joints[6]->loadConfigFile("LH.HAA");
    joints[7]->loadConfigFile("LH.HFE");
    joints[8]->loadConfigFile("LH.KFE");

    joints[9]->loadConfigFile("LF.HAA");
    joints[10]->loadConfigFile("LF.HFE");
    joints[11]->loadConfigFile("LF.KFE");

    // 人工检查配置信息是否有误！
    if(elmo::Joint::check_config())
    {
        dorun = 0;
        end_program = true;
        return;
    }

    
    /* Init code end */
    /*
     * 关闭电机
     * */
    for (int i=0;i<12;i++)
    {
        joints[i]->enable(false);

    }


    std::cout << ">>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>";
    std::cout << std::endl;


    // 初始化代码 start
    // 定义变量
    double kp = 66;
    double kd = 6;



    double original_grf[4] = {};
    double relative_grf[4] = {};

    Vector3d rf_target_q;                       // 右前腿关节角度
    Vector3d rh_target_q;                       // 右后腿关节角度
    Vector3d lh_target_q;                       // 左后腿关节角度
    Vector3d lf_target_q;                       // 左前腿关节角度
    ct::core::StateVector<12> initial_q_j;      // 初始化关节角度
    ct::core::StateVector<12> target_q_j;       // 关节期望角度
    ct::core::StateVector<12> target_q_dot_j;   // 关节期望速度
    ct::core::StateVector<12> actual_q_j;       // 实际关节角度
    ct::core::StateVector<12> actual_q_dot_j;   // 实际关节速度
    ct::core::StateVector<12> original_q_j;     // 上电时刻关节角度
    ct::core::StateVector<12> target_u;         // 关节力矩
    ct::core::StateVector<12> stand_q_j;        // 站立时刻关节角度

    ct::core::StateVector<12> actual_torque;

    // iit::pegasus2::Jacobians jacobians;

    // ct::rbd::RigidBodyPose original_base_pose;
    // ct::rbd::RigidBodyPose initial_base_pose;

    // ct::rbd::RigidBodyPose ;


    /*
    * 机器人上电时的与地磁场之间的旋转矩阵
    * */
    kindr::RotationMatrix<double> original_base_pose;


    /*
     * 机器人相对于上电时姿态的旋转矩阵（需要实时更新）
     * */
    kindr::RotationMatrix<double> relative_base_pose;


    /*
     * 机器人当前绝对姿态（与地磁场之间的旋转矩阵，需要实时更新）
     * */
    kindr::RotationMatrix<double> absolute_base_pose;


    /*
     * 定义机器人惯性属性，运动变换，逆动力学
     * */
    iit::pegasus2::dyn::InertiaProperties inertiaProperties;
    iit::pegasus2::MotionTransforms motionTransforms;
    iit::pegasus2::HomogeneousTransforms homogeneousTransforms;

    iit::pegasus2::dyn::InverseDynamics inverseDynamics(inertiaProperties, motionTransforms);
    iit::pegasus2::dyn::InverseDynamics::ExtForces force_ext; //定义外力
    force_ext[iit::pegasus2::TORSO].setZero();
    force_ext[iit::pegasus2::RF_HIP].setZero();
    force_ext[iit::pegasus2::RF_UPPERLEG].setZero();
    force_ext[iit::pegasus2::RH_HIP].setZero();
    force_ext[iit::pegasus2::RH_UPPERLEG].setZero();
    force_ext[iit::pegasus2::LH_HIP].setZero();
    force_ext[iit::pegasus2::LH_UPPERLEG].setZero();
    force_ext[iit::pegasus2::LF_HIP].setZero();
    force_ext[iit::pegasus2::LF_UPPERLEG].setZero();

    ct::rbd::pegasus2::Kinematics kinematics;


    ct::rbd::pegasus2::Kinematics::Vector3Tpl rf_w_force;
    ct::rbd::pegasus2::Kinematics::Vector3Tpl rh_w_force;
    ct::rbd::pegasus2::Kinematics::Vector3Tpl lh_w_force;
    ct::rbd::pegasus2::Kinematics::Vector3Tpl lf_w_force;



    ct::rbd::RigidBodyPose basePose;
    basePose.setIdentity();
    // basePose;
    // auto rf_eeforce = kinematics.mapForceFromWorldToLink3d(rf_w_force, basePose, actual_q_j, 0);
    // auto rh_eeforce = kinematics.mapForceFromWorldToLink3d(rh_w_force, basePose, actual_q_j, 1);
    // auto lh_eeforce = kinematics.mapForceFromWorldToLink3d(lh_w_force, basePose, actual_q_j, 2);
    // auto lf_eeforce = kinematics.mapForceFromWorldToLink3d(lf_w_force, basePose, actual_q_j, 3);

    // force_ext[iit::pegasus2::RF_LOWERLEG] = rf_eeforce;
    // force_ext[iit::pegasus2::RH_LOWERLEG] = rh_eeforce;
    // force_ext[iit::pegasus2::LH_LOWERLEG] = lh_eeforce;
    // force_ext[iit::pegasus2::LF_LOWERLEG] = lf_eeforce;

    iit::pegasus2::dyn::InverseDynamics::Acceleration torso_a;
    iit::pegasus2::dyn::InverseDynamics::Velocity torso_v;

    iit::pegasus2::dyn::InverseDynamics::Acceleration gravity_acc;
    gravity_acc.setZero();
    gravity_acc[iit::rbd::LZ] = -9.81;              //重力加速度

    iit::pegasus2::JointState u_dynamics;           //逆动力学得到的关节力矩
    iit::pegasus2::JointState u_G_term;             //重力项关节力矩，腾空时使用
    iit::pegasus2::dyn::InverseDynamics::Force base_pending; //吊起机器人所需要的力，一般不使用
    iit::pegasus2::JointState target_q_dotdot_j;    //期望关节加速度
    target_q_dotdot_j.setZero();



    iit::pegasus2::Jacobians jacobians;

    Eigen::Matrix<double, 3, 1> rf_grf_estimated;
    Eigen::Matrix<double, 3, 1> rh_grf_estimated;
    Eigen::Matrix<double, 3, 1> lh_grf_estimated;
    Eigen::Matrix<double, 3, 1> lf_grf_estimated;

    /*
     * 压力传感器滤波器,
     * */
    Queue<double> RF_grf_filter(10);
    Queue<double> RH_grf_filter(10);
    Queue<double> LH_grf_filter(10);
    Queue<double> LF_grf_filter(10);

    /*
     * 等待初始化时间，读取编码器值，防止突变，单位：ms
     * */
    int initial_span = 5000;
    sem_init(&bin_sem, 0, 0);
    for(int cnt_initial=0;cnt_initial<initial_span;cnt_initial++)
    {
        sem_wait(&bin_sem);
        for (int i=0;i<12;i++)
        {
            joints[i]->enable(false);
            joints[i]->torque_cmd(0);
        }
        std::cout << "正在读取传感器信息，剩余：" << std::fixed << std::setprecision(1) << (initial_span - cnt_initial) * 0.001 << " s\r";
    }
    std::cout << std::endl << "初始化完成" << std::endl;

    // 计算初始化轨迹
    for (int i=0;i<12;i++)
    {
        original_q_j(i, 0) = joints[i]->getActualAngular(); // 记录上电时刻的关节角度
    }


    // 初始化足端位置
    double x_offset = -0.0325;

    FootEndPosition rf_foot_end_initial(x_offset, -0.050, -0.20);
    FootEndPosition rh_foot_end_initial(x_offset, -0.050, -0.20);
    FootEndPosition lh_foot_end_initial(x_offset, 0.050, -0.20);
    FootEndPosition lf_foot_end_initial(x_offset, 0.050, -0.20);


    // 逆解计算初始化关节角度
    inverse_kinematics(rf_target_q, rf_foot_end_initial, -1);
    inverse_kinematics(rh_target_q, rh_foot_end_initial, -1);
    inverse_kinematics(lh_target_q, lh_foot_end_initial,  1);
    inverse_kinematics(lf_target_q, lf_foot_end_initial,  1);

    // 转换为12维向量
    initial_q_j.block<3, 1>(0, 0) = rf_target_q;
    initial_q_j.block<3, 1>(3, 0) = rh_target_q;
    initial_q_j.block<3, 1>(6, 0) = lh_target_q;
    initial_q_j.block<3, 1>(9, 0) = lf_target_q;

    // 生成初始化轨迹
    auto initial_traj_data = ct::core::linspace<ct::core::StateVectorArray<12, double>>(original_q_j, initial_q_j, 5000);
    ct::core::StateTrajectory<12, double> initial_trajectory(initial_traj_data, 0.001, 0.0);

    // 站立足端位置
    FootEndPosition rf_foot_end_stand(x_offset, -0.050, -0.48);
    FootEndPosition rh_foot_end_stand(x_offset, -0.050, -0.48);
    FootEndPosition lh_foot_end_stand(x_offset, 0.050, -0.48);
    FootEndPosition lf_foot_end_stand(x_offset, 0.050, -0.48);

    /*
     * 生成站立足端轨迹
     * */
    auto rf_footend_stand_up_trajectory_data =
            ct::core::linspace<ct::core::DiscreteArray<ct::rbd::pegasus2::Kinematics::Position3Tpl>>(rf_foot_end_initial, rf_foot_end_stand, 5000);
    auto rh_footend_stand_up_trajectory_data =
            ct::core::linspace<ct::core::DiscreteArray<ct::rbd::pegasus2::Kinematics::Position3Tpl>>(rh_foot_end_initial, rh_foot_end_stand, 5000);
    auto lh_footend_stand_up_trajectory_data =
            ct::core::linspace<ct::core::DiscreteArray<ct::rbd::pegasus2::Kinematics::Position3Tpl>>(lh_foot_end_initial, lh_foot_end_stand, 5000);
    auto lf_footend_stand_up_trajectory_data =
            ct::core::linspace<ct::core::DiscreteArray<ct::rbd::pegasus2::Kinematics::Position3Tpl>>(lf_foot_end_initial, lf_foot_end_stand, 5000);

    auto rf_stand_up_traj_data =
            ct::core::linspace<ct::core::DiscreteArray<Vector3d>>(Vector3d::Zero(), Vector3d::Zero(), 5000);
    auto rh_stand_up_traj_data =
            ct::core::linspace<ct::core::DiscreteArray<Vector3d>>(Vector3d::Zero(), Vector3d::Zero(), 5000);
    auto lh_stand_up_traj_data =
            ct::core::linspace<ct::core::DiscreteArray<Vector3d>>(Vector3d::Zero(), Vector3d::Zero(), 5000);
    auto lf_stand_up_traj_data =
            ct::core::linspace<ct::core::DiscreteArray<Vector3d>>(Vector3d::Zero(), Vector3d::Zero(), 5000);

    /*
     * 逆解各腿站立时的关节轨迹
     * */
    for(int i=0;i<5000;i++)
    {
        inverse_kinematics(rf_stand_up_traj_data[i], rf_footend_stand_up_trajectory_data[i], -1);
        inverse_kinematics(rh_stand_up_traj_data[i], rh_footend_stand_up_trajectory_data[i], -1);
        inverse_kinematics(lh_stand_up_traj_data[i], lh_footend_stand_up_trajectory_data[i], 1);
        inverse_kinematics(lf_stand_up_traj_data[i], lf_footend_stand_up_trajectory_data[i], 1);
    }

    /*
     * 转换为12维向量
     * */
    ct::core::StateVectorArray<12, double> stand_up_traj_data;
    for (int i=0;i<5000;i++)
    {
        stand_up_traj_data[i].block<3, 1>(0, 0) = rf_stand_up_traj_data[i];
        stand_up_traj_data[i].block<3, 1>(3, 0) = rh_stand_up_traj_data[i];
        stand_up_traj_data[i].block<3, 1>(6, 0) = lh_stand_up_traj_data[i];
        stand_up_traj_data[i].block<3, 1>(9, 0) = lf_stand_up_traj_data[i];
    }

    /*
     * 构建站立关节轨迹
     * */
    ct::core::StateTrajectory<12, double> stand_up_trajectory(stand_up_traj_data, 0.001, 5.0);


    target_q_dot_j = (initial_q_j - original_q_j) / 5.0;
    target_q_j = original_q_j;
    // 初始化代码 end

    
    for (int i=0;i<12;i++)
    {
        joints[i]->enable(false);

    }

    
    
    int joint_num = 0;

    double time_now = 0;
    std::chrono::high_resolution_clock::time_point current_time;
    std::chrono::high_resolution_clock::time_point start_time;

    std::chrono::duration<double> time_span{0};

    std::cout << "--------------------------------------------" << std::endl;
    sem_init(&bin_sem, 0, 0);
    initial_span = 3000;
    for(int cnt_initial=0;cnt_initial<initial_span && !end_program;cnt_initial++)
    {
        sem_wait(&bin_sem);
        for (int i=0;i<12;i++)
        {
            // 使能电机，力矩为零
            joints[i]->enable(true);
            joints[i]->setOpMode(10);
            joints[i]->torque_cmd(0);
        }
        std::cout << "正在初始化，剩余：" << std::fixed << std::setprecision(1) << (initial_span - cnt_initial) * 0.001 << " s\r";

        absolute_base_pose = actual_imu_data;
        original_base_pose = absolute_base_pose;

        original_grf[0] += input_data[0]->ground_force / static_cast<double>(initial_span);
        original_grf[1] += input_data[3]->ground_force / static_cast<double>(initial_span);
        original_grf[2] += input_data[8]->ground_force / static_cast<double>(initial_span);
        original_grf[3] += input_data[9]->ground_force / static_cast<double>(initial_span);


        // 检查关节是否超过限制
        for (int i=0;i<elmo_count;i++)
        {
            Joint_t::ErrorCode errorCode;
            errorCode = joints[i]->limit_check();
            if (errorCode != elmo::Joint::NoError)
            {
                std::cout << "errorCode = " << errorCode << std::endl;
                std::cout << joints[i]->name() << " 关节超出限制，角度为：" << rad2deg(joints[i]->getActualAngular()) << "度" << std::endl;
                std::cout << "max = " << rad2deg(joints[i]->limit_max()) << ",  min = " << rad2deg(joints[i]->limit_min()) << std::endl;
                std::cout << joints[i]->name() << " 编码器值为：" << motors[i]->getActualPosition() << std::endl;
                end_program = true;
                joints[i]->enable(false);
                break;
            }
            else
            {
                joints[i]->send_control_word();
            }
        }
    }


    /*
     * 将之前的累加值归零
     * */
    sem_init(&bin_sem, 0, 0);

    /*
     * 获取开始时刻的时间
     * */
    start_time = std::chrono::high_resolution_clock::now();


    /*
     * 进入主循环
     * 周期：1ms
     * */
    while(!end_program) {
        sem_wait(&bin_sem);
        current_time = std::chrono::high_resolution_clock::now();
        time_span = std::chrono::duration_cast<std::chrono::duration<double>>(current_time - start_time);
        time_now = time_span.count();


        /*
         * 更新传感器信息包括关节位置，速度，机身姿态
         * */
        for(int i=0;i<12;i++) // 更新12个关节角度和速度
        {
            actual_q_j(i) = joints[i]->getActualAngular();
            actual_q_dot_j(i) = joints[i]->getActualVelocity();
            actual_torque(i) = joints[i]->getActualTorque();

        }
        // 更新足底压力传感器的值
        /*
         * 将压力传感器信号滤波，并获取相对上电时的值
         * */
        RF_grf_filter.push(input_data[0]->ground_force);
        RH_grf_filter.push(input_data[3]->ground_force);
        LH_grf_filter.push(input_data[8]->ground_force);
        LF_grf_filter.push(input_data[9]->ground_force);


        relative_grf[0] = RF_grf_filter.mean() - original_grf[0];
        relative_grf[1] = RH_grf_filter.mean() - original_grf[1];
        relative_grf[2] = LH_grf_filter.mean() - original_grf[2];
        relative_grf[3] = LF_grf_filter.mean() - original_grf[3];

        // 更新绝对姿态和相对上电时的姿态
        absolute_base_pose = actual_imu_data;
        relative_base_pose = original_base_pose.inverseRotate(absolute_base_pose);
        basePose.setFromRotationMatrix(relative_base_pose);


        /*
         * 更新足端力估计值
         * */
        if(relative_grf[0] > 50)
        {
            jacobians.fr_torso_J_fr_RF_foot.update(actual_q_j);
            rf_grf_estimated = - jacobians.fr_torso_J_fr_RF_foot.block<3, 3>(3, 0).transpose().inverse() * actual_torque.block<3, 1>(0, 0);
        } else{
            rf_grf_estimated.setZero();
        }
        if(relative_grf[1] > 50)
        {
            jacobians.fr_torso_J_fr_RH_foot.update(actual_q_j);
            rh_grf_estimated = - jacobians.fr_torso_J_fr_RH_foot.block<3, 3>(3, 0).transpose().inverse() * actual_torque.block<3, 1>(3, 0);
        } else{
            rh_grf_estimated.setZero();
        }
        if(relative_grf[2] > 50)
        {
            jacobians.fr_torso_J_fr_LH_foot.update(actual_q_j);
            lh_grf_estimated = - jacobians.fr_torso_J_fr_LH_foot.block<3, 3>(3, 0).transpose().inverse() * actual_torque.block<3, 1>(6, 0);
        } else{
            lh_grf_estimated.setZero();
        }
        if(relative_grf[3] > 50)
        {
            jacobians.fr_torso_J_fr_LF_foot.update(actual_q_j);
            lf_grf_estimated = - jacobians.fr_torso_J_fr_LF_foot.block<3, 3>(3, 0).transpose().inverse() * actual_torque.block<3, 1>(9, 0);
        } else{
            lf_grf_estimated.setZero();
        }

        /*
         * 计算机器人被吊起时平衡腿部重力所需要的关节力矩
         * */
        inverseDynamics.G_terms_fully_actuated(base_pending, u_G_term, gravity_acc, actual_q_j);

        /*
         * 小腿受力赋值
         * */
        force_ext[iit::pegasus2::RF_LOWERLEG] = kinematics.mapForceFromWorldToLink3d(rf_grf_estimated, basePose, actual_q_j, 0);
        force_ext[iit::pegasus2::RH_LOWERLEG] = kinematics.mapForceFromWorldToLink3d(rh_grf_estimated, basePose, actual_q_j, 1);
        force_ext[iit::pegasus2::LH_LOWERLEG] = kinematics.mapForceFromWorldToLink3d(lh_grf_estimated, basePose, actual_q_j, 2);
        force_ext[iit::pegasus2::LF_LOWERLEG] = kinematics.mapForceFromWorldToLink3d(lf_grf_estimated, basePose, actual_q_j, 3);


        /*
         * 逆动力学
         * u_dynamics：输出动力学关节力矩
         * torso_a：输出机身加速度
         * gravity_acc：常量，重力加速度在世界坐标系下的坐标表示
         * torso_v：机身速度，暂时设置为零
         * actual_q_j：关节角度，实时更新
         * actual_q_dot_j：关节速度，实时更新
         * target_q_dotdot_j：期望关节加速度
         * force_ext：外力，每个连杆所受外力的集合，除小腿外所有连杆外力都设为零
         * */
        inverseDynamics.id(u_dynamics, torso_a, gravity_acc, torso_v, actual_q_j, actual_q_dot_j, target_q_dotdot_j, force_ext);


        /*
         * 检查关节是否超过限制
         * */
        for (int i=0;i<elmo_count;i++)
        {
            /*
             * 建议改为返回错误码，关闭Ethercat通信再打印信息
             * */
            Joint_t::ErrorCode errorCode;
            errorCode = joints[i]->limit_check();
            if (errorCode != elmo::Joint::NoError)
            {
                std::cout << "errorCode = " << errorCode << std::endl;
                std::cout << joints[i]->name() << " 关节超出限制，角度为：" << rad2deg(joints[i]->getActualAngular()) << "度" << std::endl;
                std::cout << "max = " << rad2deg(joints[i]->limit_max()) << ",  min = " << rad2deg(joints[i]->limit_min()) << std::endl;
                std::cout << joints[i]->name() << " 编码器值为：" << motors[i]->getActualPosition() << std::endl;
                end_program = true;
                joints[i]->enable(false);
            } else{
                joints[i]->send_control_word();
            }
        }


        /*
         * 时间驱动
         * */
        if (time_now < 5.0)
        {
            
            target_q_j = initial_trajectory.eval(time_now);
            
        }
        else if (time_now < 10.0)
        {
            target_q_j = stand_up_trajectory.eval(time_now);
            target_q_dot_j = (stand_q_j - initial_q_j) / 5.0;
        }
        else
        {

            target_q_j = stand_q_j;
            target_q_dot_j.setZero();
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

        /*
         * 发送关节力矩
         * */
        target_u += u_G_term;
        for (int i=0;i<12;i++)
        {
            joints[i]->torque_cmd(target_u(i));
            // joints[i]->torque_cmd(0);
        }

        /*
         * 记录出错
         * */
        for (auto i: input_data) {
            if ((i->status_word & 0x004fU) == 0x0008U) {
                error_file << "---------- elmo fault start ----------" << std::endl;
                error_file << "elmo fault code: " << std::endl;
                error_file << "---------- elmo fault end ----------" << std::endl;
            }
        }


        /*
         * 输出数据到文件
         */
        for (int i=0;i<12;i++)
        {
            target_joint_position_file << target_q_j(i) << ",";
            actual_joint_position_file << actual_q_j(i) << ",";
            target_joint_velocity_file << target_q_dot_j(i) << ",";
            actual_joint_velocity_file << actual_q_dot_j(i) << ",";
            target_joint_torque_file << target_u(i) << ",";
            actual_joint_torque_file << joints[i]->getActualTorque() << ",";

        }
        for(int i=0;i<4;i++)
        {
            ground_force_file << relative_grf[i] << ",";
        }
        target_joint_position_file << time_now << std::endl;
        actual_joint_position_file << time_now << std::endl;
        target_joint_velocity_file << time_now << std::endl;
        actual_joint_velocity_file << time_now << std::endl;
        target_joint_torque_file << time_now << std::endl;
        actual_joint_torque_file << time_now << std::endl;
        DC_time_file << time_now << std::endl;
        ground_force_file << time_now << std::endl;


        std::cout << "<<<<<<<<<<<" << std::setprecision(4) << std::endl;
        std::cout << "rf_grf_estimated = " << rf_grf_estimated.transpose() << std::endl;
        std::cout << "rh_grf_estimated = " << rh_grf_estimated.transpose() << std::endl;
        std::cout << "lh_grf_estimated = " << lh_grf_estimated.transpose() << std::endl;
        std::cout << "lh_grf_estimated = " << lf_grf_estimated.transpose() << std::endl;
        std::cout << "u_dynamics = " << u_dynamics.transpose() << std::endl;
        std::cout << "torso_a = " << torso_a.transpose() << std::endl;
        std::cout << "original_grf = ";
        for(int i=0;i<4;i++)
        {
            std::cout << original_grf[i] << ", ";
        }
        std::cout << std::endl;
        std::cout << "-----" << std::endl;
        homogeneousTransforms.fr_torso_X_fr_LF_foot.update(actual_q_j);
        std::cout << homogeneousTransforms.fr_torso_X_fr_LF_foot.block<3, 1>(0, 3) << std::endl;
        std::cout << "-----" << std::endl;
        /*
         * 检查结束标志，如果为真则退出！
         * */
        // check if get the end program signal
        if (end_program) {
            break;
        }
    }
}


void master_shutdown()
{
    for(int slave=1;slave<=ec_slavecount;slave++)
    {
        ec_dcsync0(slave, FALSE, 8000, 0); // SYNC0 off
    }
    std::cout << "Request safe operational state for all slaves\n";
    ec_slave[0].state = EC_STATE_SAFE_OP;
    /* request SAFE_OP state for all slaves */
    ec_writestate(0);
    /* wait for all slaves to reach state */
    ec_statecheck(0, EC_STATE_SAFE_OP,  EC_TIMEOUTSTATE);
    ec_slave[0].state = EC_STATE_PRE_OP;
    /* request SAFE_OP state for all slaves */
    ec_writestate(0);
    /* wait for all slaves to reach state */
    ec_statecheck(0, EC_STATE_PRE_OP,  EC_TIMEOUTSTATE);
}

static void slave_info()
{
    for(int cnt = 1; cnt <= ec_slavecount ; cnt++)
    {
        std::cout << "Slave:" << cnt << ", "
                  << "Name:" << ec_slave[cnt].name << ", "
                  << "Output size: " << ec_slave[cnt].Obits << "bits, "
                  << "Input size: " << ec_slave[cnt].Ibits << "bits, "
                  << "State: " << ec_slave[cnt].state << ", "
                  << "delay: " << ec_slave[cnt].pdelay << ", ";
        if(ec_slave[cnt].hasdc != 0) {
            std::cout << "has DC: " << "true" << std::endl;
        }
        else{
            std::cout << "has DC: " << "false" << std::endl;
        }

    }
}

/*
 * 请求所有从站进入目标状态
 * */
void write_slave_state(uint16 target_state)
{
    ec_slave[0].state = target_state;
    /* request OP state for all slaves */
    ec_writestate(0);
    /* wait for all slaves to reach OP state */
    ec_statecheck(0, target_state,  EC_TIMEOUTSTATE);
}

/*
 *
 * */
void elmo_config(const char * ifname)
{
    int result = 0;
    /* initialise SOEM, bind socket to ifname */
    result = ec_init(ifname);
    if(!result) {std::cerr << "No socket connection on " << ifname << "\n";return;}
    std::cout << "ec_init on " << ifname <<  " succeeded." << std::endl;
    /* find and auto-config slaves */

    result = ec_config_init(FALSE);
    if(result <= 0) {std::cout << "No slaves found!" << std::endl;ec_close();return;}
    std::cout << ec_slavecount << "slaves found and configured." << std::endl;
    do{}while(io_config());
    ec_config_map(&IOmap);
    ec_configdc();
    /* wait for all slaves to reach SAFE_OP state */
    ec_statecheck(0, EC_STATE_SAFE_OP,  EC_TIMEOUTSTATE);

    /* configure DC options for every DC capable slave found in the list */
    std::cout << "DC capable : " << ec_configdc() << std::endl;

    /* check configuration */
    if (ec_slavecount == elmo_count) {
        std::cout << elmo_count << " elmo drives found" << std::endl;
        /* connect struct pointers to slave I/O pointers */
        /* read indevidual slave state and store in ec_slave[] */
        ec_readstate();
        slave_info();
        std::cout << "Request operational state for all slaves\n";
        ec_send_processdata();
        ec_receive_processdata(EC_TIMEOUTRET);
        write_slave_state(EC_STATE_OPERATIONAL);
        if (ec_slave[0].state == EC_STATE_OPERATIONAL ) {
            std::cout << "Operational state reached for all slaves.\n";
            if(0 != sem_init(&bin_sem, 0, 0))perror("Semaphore initialization failed");
            dorun = 1;
            usleep(100000); // wait for linux to sync on DC
            for(int i=1;i<=ec_slavecount;i++)ec_dcsync0(i, TRUE, SYNC0TIME, 0); // SYNC0 on slave 1
            control_loop();
            dorun = 0;
        }
        else std::cout << "Not all slaves reached operational state.\n";
    }
    else if (ec_slavecount > elmo_count) std::cout << "elmo count is less than EtherCAT slave count.\n";
    else if (ec_slavecount < elmo_count) 
    {
        std::cout << "elmo count is more than EtherCAT slave count.\n";
        std::cout << "elmo count = " << elmo_count << std::endl;
        std::cout << "EtherCAT slave count = " << ec_slavecount << std::endl;
    }
    master_shutdown();
    ec_close();
}

/* add ns to timespec */
inline void add_timespec(struct timespec & ts, const int64 & addtime)
{
    int64 sec, nsec;

    nsec = addtime % NSEC_PER_SEC;
    sec = (addtime - nsec) / NSEC_PER_SEC;
    ts.tv_sec += sec;
    ts.tv_nsec += nsec;
    if ( ts.tv_nsec > NSEC_PER_SEC )
    {
        nsec = ts.tv_nsec % NSEC_PER_SEC;
        ts.tv_sec += (ts.tv_nsec - nsec) / NSEC_PER_SEC;
        ts.tv_nsec = nsec;
    }
}

/* PI calculation to get linux time synced to DC time */
inline void ec_sync(const int64 & reftime, const int64 & cycletime , int64 & offsettime)
{
    int64 delta;
    /* set linux sync point 50us later than DC sync, just as example */
    delta = (reftime - 50000) % cycletime;
    if(delta> (cycletime /2)) { delta= delta - cycletime; }
    if(delta>0){ integral++; }
    if(delta<0){ integral--; }
    offsettime = -(delta / 100) - (integral /20);
}

/* RT EtherCAT thread */
void * ecatthread( void *ptr )
{
    // std::chrono::high_resolution_clock::time_point current_time;

    // std::chrono::high_resolution_clock::time_point last_time;
    // std::chrono::duration<double> time_span;
    // int64 DC_time_now;
    struct timespec   ts;
    struct timeval    tp;
    int ht;
    // int i;

    int64 cycletime = 0;
    pthread_mutex_lock(&mutex);
    gettimeofday(&tp, nullptr);
    /* Convert from timeval to timespec */
    ts.tv_sec  = tp.tv_sec;
    ht = (tp.tv_usec / 1000) + 1; /* round to nearest ms */
    ts.tv_nsec = ht * 1000000;
    cycletime = *(int*)ptr * 1000; /* cycletime in ns */
    int64 toff = 0;
    dorun = 0;
    while(true)
    {
        /* calculate next cycle start */
        add_timespec(ts, cycletime + toff);
        /* wait to cycle start */
        
        pthread_cond_timedwait(&cond, &mutex, &ts);
        
        
        if (dorun>0)
        {
            gettimeofday(&tp, nullptr);
            ec_send_processdata();
            ec_receive_processdata(EC_TIMEOUTRET);
            // DC_time_now = ec_DCtime;
            
            cyclecount++;
            
            /* calculate toff to get linux time and DC synced */
            ec_sync(ec_DCtime, cycletime, toff);
            sem_post(&bin_sem);
        }
        if(end_program)
        {
            break;
        }
    }
    return nullptr;
}

int main(int argc, char *argv[])
{
    int result = 0;
    std::string config_file_name = "ethercat.json";
    std::string config_dir_name = configDir;
    std::string config_file_full_name = config_dir_name + "/" + config_file_name;
    if(argc > 1)
    {
        std::cerr << "Notice: the program parameters are invalid.\n";
        std::cerr << "Please write the configuration to the config file:\n";
        std::cerr << config_file_full_name << std::endl;
        exit(EXIT_FAILURE);
    }
    boost::property_tree::ptree pt;
    boost::property_tree::read_json(config_file_full_name, pt);
    std::string if_name;
    try{
        elmo_count = pt.get<int>("slave count");
        if_name = pt.get<std::string>("interface name");
        std::cout << "Interface name: [" << if_name << "]" << std::endl;
    }catch(...){
        std::cerr << "Please check the config file!\n";
        std::cerr << "Check" << config_file_full_name << std::endl;
    }
    (void)signal(SIGINT, ctrl_c_function);

    target_joint_position_file.open(dataDir + "/target_joint_position.csv", std::ios::out);
    actual_joint_position_file.open(dataDir + "/actual_joint_position.csv", std::ios::out);
    target_joint_velocity_file.open(dataDir + "/target_joint_velocity.csv", std::ios::out);
    actual_joint_velocity_file.open(dataDir + "/actual_joint_velocity.csv", std::ios::out);
    target_joint_torque_file.open(dataDir + "/target_joint_torque.csv", std::ios::out);
    actual_joint_torque_file.open(dataDir + "/actual_joint_torque.csv", std::ios::out);
    ground_force_file.open(dataDir + "/ground_force.csv", std::ios::out);
    error_file.open(dataDir + "/error.csv", std::ios::out);
    DC_time_file.open(dataDir + "/DC_time.csv", std::ios::out);
    std::string table_head = "1,2,3,4,5,6,7,8,9,10,11,12,time\n";
    ground_force_file << "rf,rh,lh,lf,time" << std::endl;
    target_joint_position_file << table_head;
    actual_joint_position_file << table_head;
    target_joint_velocity_file << table_head;
    actual_joint_velocity_file << table_head;
    target_joint_torque_file << table_head;
    actual_joint_torque_file << table_head;
    DC_time_file << "ec_DCtime" << std::endl;


    /* do not set priority above 49, otherwise sockets are starved */
    struct sched_param schedp;
    memset(&schedp, 0, sizeof(schedp));
    schedp.sched_priority = 30;
    result = sched_setscheduler(0, SCHED_FIFO, &schedp);
    if ( result != 0 )
    {
        std::cerr << "Set the scheduler policy to FIFO failed\n";
        std::cerr << "Please check if you executed this program as root\n";
        exit(EXIT_FAILURE);
    }

    do{
        usleep(1000);
    }while (dorun);

    dorun = 1;
    int ctime = 1000; // 1ms cycle time
    void * pVoid = &ctime;

    /* 
     * 创建EtherCAT线程
     * */
    pthread_t thread1;
    result = pthread_create( &thread1, nullptr, ecatthread, pVoid);
    if(result != 0)
    {
        std::cerr << "Thread created failed!\n";
        exit(EXIT_FAILURE);
    }
    
    /*
     * 创建IMU线程
     * */
    if(0 != pthread_create(&(imu_thread),nullptr,com_read, nullptr))
    {
        printf("线程创建失败!\n");
        exit(EXIT_FAILURE);
    }
    
    /* start acyclic part */
    elmo_config(if_name.data());
    /* end acyclic part */


    schedp.sched_priority = 0;
    sched_setscheduler(0, SCHED_OTHER, &schedp);

    std::cout << "End program\n";
    target_joint_position_file.close();
    actual_joint_position_file.close();
    target_joint_velocity_file.close();
    actual_joint_velocity_file.close();
    target_joint_torque_file.close();
    actual_joint_torque_file.close();
    ground_force_file.close();
    DC_time_file.close();
    error_file << "+++++++ Error Summary +++++++" << std::endl;
    error_file << "total DC time error times: " << DCtimeErrorTimes << std::endl;
    error_file.close();
    return (0);
}

void ctrl_c_function(int sig)
{
    end_program =true;
    (void)signal(SIGINT, SIG_DFL);
}
