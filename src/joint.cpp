#include "joint.h"
#include <iostream>
using namespace elmo;
std::string Joint::config_dir;


Joint::Joint(MotorPtr motor)
    : motor_(motor),
      ratio_(100.0),
      initial_offset_(300000),
      polarity_(1.0),
      limit_max_(10),
      limit_min_(-10)
{
    motor_->setOpMode(elmo::op_mode::CS::torque);


    revolution_ = static_cast<double>(1 << 14);
	// actual value
	actual_angular = 0;
	actual_angular_vel = 0;
	actual_torque = 0;
}

using namespace boost::property_tree;


bool Joint::check_config() 
{
    std::cout << "请仔细检查配置信息，确认无误后输入 1 ，若有错误输入 2 结束程序\n";
    int enter;
    std::cin >> enter;
    return (enter != 1);
}

const double & Joint::limit_max() const
{
    return this->limit_max_;
}

const double & Joint::limit_min() const
{
    return this->limit_min_;
}

// 检查关节位置是否超过限制，如果超过就返回真
Joint::ErrorCode Joint::limit_check()
{

    if (this->getActualAngular() < this->limit_min_)
    {
        return ErrorCode::LowerLimitExceeded;
    }
    else if(this->getActualAngular() > this->limit_max_)
    {
        return ErrorCode::UpperLimitExceeded;
    } else if (this->motor_->encoder_check()){
         return ErrorCode::EncoderError;
    }
    else
    {
        return ErrorCode::NoError;
    }
}



void Joint::send_control_word()
{
    this->motor_->status_control();
}
const std::string& Joint::name()
{
    return this->name_;
}

void Joint::loadConfigFile(const std::string & joint_name)
{
    this->name_ = joint_name;
    // 加载减速比和编码器分辨率等共同配置
    ptree common_pt;
    read_json(Joint::config_dir + "/common.json", common_pt);
    this->ratio_ = common_pt.get<double>("ratio");
    int encoder_bits = common_pt.get<int>("encoder bits");
    this->revolution_ = 1.;
    for (int i = 0; i < encoder_bits; i++)
    {
        this->revolution_ *= 2;
    }

    // 加载极性配置
    ptree polarity_pt;
    read_json(Joint::config_dir + "/polarity.json", polarity_pt);
    this->polarity_ = polarity_pt.get<double>(joint_name);

    // 加载初始位置（零位）配置
    ptree zero_position_pt;
    read_json(Joint::config_dir + "/zero_position.json", zero_position_pt);
    this->initial_offset_ = zero_position_pt.get<int32>(joint_name);

    // 加载位置限制范围
    ptree limit_pt;
    read_json(Joint::config_dir + "/limit.json", limit_pt);
    this->limit_max_ = deg2rad(limit_pt.get<double>(joint_name + ".max"));
    this->limit_min_ = deg2rad(limit_pt.get<double>(joint_name + ".min"));

    // 显示配置信息
        std::cout << joint_name << ": { polarity: " << std::setw(2) << polarity_;
        std::cout << ", zero position: "<< std::setw(7) << initial_offset_;
        std::cout << ", revolution: " << std::setw(6) << revolution_;
        std::cout << ", ratio: " << std::setw(3) << ratio_;
        std::cout << ", limit: [" << std::setw(3) << rad2deg(limit_min_) << "度 ~ " << std::setw(3) << rad2deg(limit_max_) << "度 ]";
        std::cout << " }\n";
}


const double pi = 3.14159265358979;

void Joint::setPolarity(double pol)
{
    this->polarity_ = pol;
}

void Joint::setInitPos(int32 pos)
{
    this->initial_offset_ = pos;
}


double Joint::getActualAngular()
{
    actual_angular = this->polarity_ * static_cast<double>(motor_->getActualPosition() - initial_offset_) / revolution_ / ratio_ * 2. * pi;
    return this->actual_angular;
}

double Joint::getActualVelocity()
{
    actual_angular_vel = this->polarity_ * static_cast<double>(motor_->getActualVelocity()) / revolution_ / ratio_ * 2. * pi;
	return this->actual_angular_vel;
}

double Joint::getActualTorque()
{
    // 47.3e-3 Nm/A * 7A / 1000 = 0.00033109999999999997;
    // torque整数值1000代表额定电流1A, 电机力矩常数为47.3 mNm/A
    actual_torque = this->polarity_ * static_cast<double>(motor_->getActualTorque()) * ratio_ * 3.310999999999999e-4;
	return this->actual_torque;
}



void Joint::angular_cmd(double target)
{
    auto result = initial_offset_ + static_cast<int32>(polarity_ * target / 2 / pi * revolution_ * ratio_);
	this->motor_->position_cmd(result);
}

double Joint::angular_cmd()
{
    return this->polarity_ * static_cast<double>(motor_->position_cmd() - initial_offset_) / revolution_ / ratio_ * 2. * pi;
}

void Joint::velocity_cmd(double target)
{
    auto result = static_cast<int32>(target / 2 / pi * revolution_ * ratio_ * polarity_);
	this->motor_->velocity_cmd(result);
}

void Joint::torque_cmd(double target)
{
    // 3020.235578375114 = 1 / 0.00033109999999999997
    // 0.00033109999999999997计算详见torque_cmd函数
	auto result = static_cast<short>(target / ratio_ *3020.235578375114 * polarity_);
	this->motor_->torque_cmd(result);
}

double Joint::torque_cmd()
{
    // 47.3e-3 Nm/A * 7A / 1000 = 0.00033109999999999997;
    // torque整数值1000代表额定电流1A, 电机力矩常数为47.3 mNm/A
    return (polarity_*static_cast<double>(motor_->torque_cmd()) * ratio_ * 3.310999999999999e-4);
    // return (this->polarity_ * static_cast<double>(motor_->getTargetPosition()-initial_offset_)/revolution_ /ratio_*2.*pi);
}

double Joint::velocity_cmd()
{
    return this->polarity_ * static_cast<double>(motor_->velocity_cmd()) / revolution_ / ratio_ * 2. * pi;
}


void Joint::setOpMode(int8 mode)
{
	motor_->setOpMode(mode);
}

void Joint::enable(bool e)
{
	motor_->enable(e);
}