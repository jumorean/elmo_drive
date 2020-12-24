#pragma once

#include "my_type.h"
#include "io.h"


namespace elmo {


enum class DriveState : uint8_t {
    NotReadyToSwitchOn,
    SwitchOnDisabled,
    ReadyToSwitchOn,
    SwitchedOn,
    OperationEnabled,
    QuickStopActive,
    FaultReactionActive,
    Fault,
    NA
};


class Motor {
public:
	Motor() = delete;
	Motor(InputData_t *, OutputData_t *);

	void setOpMode(int8 mode);
	void status_control();
	void position_cmd(int target);

    int32 position_cmd();

	void velocity_cmd(int target);
	int32 velocity_cmd();
	void torque_cmd(int16 target);

    int16 torque_cmd();

	void enable(bool);
	int32 getActualPosition() const;
	int32 getActualVelocity() const;
	short getActualTorque() const;

	char getOpMode();
	DriveState getDriveState();
	bool encoder_check();
	// void setPP_param(PP_param * param);
	~Motor()= default;
protected:
	InputData_t * in_;
	OutputData_t * out_;
    DriveState drive_state_;
	bool enabled_;
	//Op_mode mode;
	//int index;

};

} //namespace ethercat





