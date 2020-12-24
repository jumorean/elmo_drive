#pragma once


#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/info_parser.hpp>
#include <boost/property_tree/json_parser.hpp>
#include "motor.h"
#include "transform.h"



namespace elmo {



class Joint {
public:
	enum ErrorCode{
		NoError = 0,
		LowerLimitExceeded,
		UpperLimitExceeded,
		EncoderError,
	};
    static std::string config_dir;
    static bool check_config();
	Joint() = delete;
	// typedef std::shared_ptr<elmo::Motor> MotorPtr;
	typedef Motor * MotorPtr;
    explicit Joint(MotorPtr motor);
	void angular_cmd(double target);
	void velocity_cmd(double target);
	void torque_cmd(double target);
    double angular_cmd();
    double velocity_cmd();
	double torque_cmd();

    void loadConfigFile(const std::string & joint_name);

	void setOpMode(int8 mode);
    void enable(bool enable_);



    void setPolarity(double pol);
    void setInitPos(int32 pos);

    void send_control_word();

    const double & limit_max() const;
    const double & limit_min() const;


	ErrorCode limit_check();



	// void update_state();

	double getActualAngular();
	double getActualVelocity();
	double getActualTorque();
    
    // actual value
    double actual_angular;
    double actual_angular_vel;
    double actual_torque;


    
    

    

    const std::string& name();

protected:
	MotorPtr motor_;

	std::string name_;
	double ratio_;
	double revolution_;

	int32 initial_offset_;
    double polarity_;

	double limit_max_;
    double limit_min_;
	
};


} //namespace ethercat


