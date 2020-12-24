
template <typename TRAIT>
iit::pegasus2::tpl::HomogeneousTransforms<TRAIT>::HomogeneousTransforms():
	fr_RF_hip_X_fr_RF_upperleg(),
	fr_RF_upperleg_X_fr_RF_hip(),
	fr_RF_upperleg_X_fr_RF_lowerleg(),
	fr_RF_lowerleg_X_fr_RF_upperleg(),
	fr_RH_hip_X_fr_RH_upperleg(),
	fr_RH_upperleg_X_fr_RH_hip(),
	fr_RH_upperleg_X_fr_RH_lowerleg(),
	fr_RH_lowerleg_X_fr_RH_upperleg(),
	fr_LH_hip_X_fr_LH_upperleg(),
	fr_LH_upperleg_X_fr_LH_hip(),
	fr_LH_upperleg_X_fr_LH_lowerleg(),
	fr_LH_lowerleg_X_fr_LH_upperleg(),
	fr_LF_hip_X_fr_LF_upperleg(),
	fr_LF_upperleg_X_fr_LF_hip(),
	fr_LF_upperleg_X_fr_LF_lowerleg(),
	fr_LF_lowerleg_X_fr_LF_upperleg(),
	fr_torso_X_fr_RF_hip(),
	fr_RF_hip_X_fr_torso(),
	fr_torso_X_fr_RF_upperleg(),
	fr_RF_upperleg_X_fr_torso(),
	fr_torso_X_fr_RF_lowerleg(),
	fr_RF_lowerleg_X_fr_torso(),
	fr_torso_X_fr_RH_hip(),
	fr_RH_hip_X_fr_torso(),
	fr_torso_X_fr_RH_upperleg(),
	fr_RH_upperleg_X_fr_torso(),
	fr_torso_X_fr_RH_lowerleg(),
	fr_RH_lowerleg_X_fr_torso(),
	fr_torso_X_fr_LH_hip(),
	fr_LH_hip_X_fr_torso(),
	fr_torso_X_fr_LH_upperleg(),
	fr_LH_upperleg_X_fr_torso(),
	fr_torso_X_fr_LH_lowerleg(),
	fr_LH_lowerleg_X_fr_torso(),
	fr_torso_X_fr_LF_hip(),
	fr_LF_hip_X_fr_torso(),
	fr_torso_X_fr_LF_upperleg(),
	fr_LF_upperleg_X_fr_torso(),
	fr_torso_X_fr_LF_lowerleg(),
	fr_LF_lowerleg_X_fr_torso(),
	fr_torso_X_fr_RF_foot(),
	fr_RF_foot_X_fr_torso(),
	fr_torso_X_fr_RH_foot(),
	fr_RH_foot_X_fr_torso(),
	fr_torso_X_fr_LH_foot(),
	fr_LH_foot_X_fr_torso(),
	fr_torso_X_fr_LF_foot(),
	fr_LF_foot_X_fr_torso()
{
	updateParameters();
}

template <typename TRAIT>
void iit::pegasus2::tpl::HomogeneousTransforms<TRAIT>::updateParameters() {
}

template <typename TRAIT>
iit::pegasus2::tpl::MotionTransforms<TRAIT>::MotionTransforms():
	fr_RF_hip_X_fr_RF_upperleg(),
	fr_RF_upperleg_X_fr_RF_hip(),
	fr_RF_upperleg_X_fr_RF_lowerleg(),
	fr_RF_lowerleg_X_fr_RF_upperleg(),
	fr_RH_hip_X_fr_RH_upperleg(),
	fr_RH_upperleg_X_fr_RH_hip(),
	fr_RH_upperleg_X_fr_RH_lowerleg(),
	fr_RH_lowerleg_X_fr_RH_upperleg(),
	fr_LH_hip_X_fr_LH_upperleg(),
	fr_LH_upperleg_X_fr_LH_hip(),
	fr_LH_upperleg_X_fr_LH_lowerleg(),
	fr_LH_lowerleg_X_fr_LH_upperleg(),
	fr_LF_hip_X_fr_LF_upperleg(),
	fr_LF_upperleg_X_fr_LF_hip(),
	fr_LF_upperleg_X_fr_LF_lowerleg(),
	fr_LF_lowerleg_X_fr_LF_upperleg(),
	fr_torso_X_fr_RF_hip(),
	fr_RF_hip_X_fr_torso(),
	fr_torso_X_fr_RF_upperleg(),
	fr_RF_upperleg_X_fr_torso(),
	fr_torso_X_fr_RF_lowerleg(),
	fr_RF_lowerleg_X_fr_torso(),
	fr_torso_X_fr_RH_hip(),
	fr_RH_hip_X_fr_torso(),
	fr_torso_X_fr_RH_upperleg(),
	fr_RH_upperleg_X_fr_torso(),
	fr_torso_X_fr_RH_lowerleg(),
	fr_RH_lowerleg_X_fr_torso(),
	fr_torso_X_fr_LH_hip(),
	fr_LH_hip_X_fr_torso(),
	fr_torso_X_fr_LH_upperleg(),
	fr_LH_upperleg_X_fr_torso(),
	fr_torso_X_fr_LH_lowerleg(),
	fr_LH_lowerleg_X_fr_torso(),
	fr_torso_X_fr_LF_hip(),
	fr_LF_hip_X_fr_torso(),
	fr_torso_X_fr_LF_upperleg(),
	fr_LF_upperleg_X_fr_torso(),
	fr_torso_X_fr_LF_lowerleg(),
	fr_LF_lowerleg_X_fr_torso(),
	fr_torso_X_fr_RF_foot(),
	fr_RF_foot_X_fr_torso(),
	fr_torso_X_fr_RH_foot(),
	fr_RH_foot_X_fr_torso(),
	fr_torso_X_fr_LH_foot(),
	fr_LH_foot_X_fr_torso(),
	fr_torso_X_fr_LF_foot(),
	fr_LF_foot_X_fr_torso()
{
	updateParameters();
}

template <typename TRAIT>
void iit::pegasus2::tpl::MotionTransforms<TRAIT>::updateParameters() {
}

template <typename TRAIT>
iit::pegasus2::tpl::ForceTransforms<TRAIT>::ForceTransforms():
	fr_RF_hip_X_fr_RF_upperleg(),
	fr_RF_upperleg_X_fr_RF_hip(),
	fr_RF_upperleg_X_fr_RF_lowerleg(),
	fr_RF_lowerleg_X_fr_RF_upperleg(),
	fr_RH_hip_X_fr_RH_upperleg(),
	fr_RH_upperleg_X_fr_RH_hip(),
	fr_RH_upperleg_X_fr_RH_lowerleg(),
	fr_RH_lowerleg_X_fr_RH_upperleg(),
	fr_LH_hip_X_fr_LH_upperleg(),
	fr_LH_upperleg_X_fr_LH_hip(),
	fr_LH_upperleg_X_fr_LH_lowerleg(),
	fr_LH_lowerleg_X_fr_LH_upperleg(),
	fr_LF_hip_X_fr_LF_upperleg(),
	fr_LF_upperleg_X_fr_LF_hip(),
	fr_LF_upperleg_X_fr_LF_lowerleg(),
	fr_LF_lowerleg_X_fr_LF_upperleg(),
	fr_torso_X_fr_RF_hip(),
	fr_RF_hip_X_fr_torso(),
	fr_torso_X_fr_RF_upperleg(),
	fr_RF_upperleg_X_fr_torso(),
	fr_torso_X_fr_RF_lowerleg(),
	fr_RF_lowerleg_X_fr_torso(),
	fr_torso_X_fr_RH_hip(),
	fr_RH_hip_X_fr_torso(),
	fr_torso_X_fr_RH_upperleg(),
	fr_RH_upperleg_X_fr_torso(),
	fr_torso_X_fr_RH_lowerleg(),
	fr_RH_lowerleg_X_fr_torso(),
	fr_torso_X_fr_LH_hip(),
	fr_LH_hip_X_fr_torso(),
	fr_torso_X_fr_LH_upperleg(),
	fr_LH_upperleg_X_fr_torso(),
	fr_torso_X_fr_LH_lowerleg(),
	fr_LH_lowerleg_X_fr_torso(),
	fr_torso_X_fr_LF_hip(),
	fr_LF_hip_X_fr_torso(),
	fr_torso_X_fr_LF_upperleg(),
	fr_LF_upperleg_X_fr_torso(),
	fr_torso_X_fr_LF_lowerleg(),
	fr_LF_lowerleg_X_fr_torso(),
	fr_torso_X_fr_RF_foot(),
	fr_RF_foot_X_fr_torso(),
	fr_torso_X_fr_RH_foot(),
	fr_RH_foot_X_fr_torso(),
	fr_torso_X_fr_LH_foot(),
	fr_LH_foot_X_fr_torso(),
	fr_torso_X_fr_LF_foot(),
	fr_LF_foot_X_fr_torso()
{
	updateParameters();
}

template <typename TRAIT>
void iit::pegasus2::tpl::ForceTransforms<TRAIT>::updateParameters() {
}

template <typename TRAIT>
iit::pegasus2::tpl::HomogeneousTransforms<TRAIT>::Type_fr_RF_hip_X_fr_RF_upperleg::Type_fr_RF_hip_X_fr_RF_upperleg(){ 
	(*this)(0, 2) = 0;
	(*this)(0, 3) = 0;
	(*this)(1, 0) = 0;
	(*this)(1, 1) = 0;
	(*this)(1, 2) = -1;
	(*this)(1, 3) = -0.0500000000000000;
	(*this)(2, 2) = 0;
	(*this)(2, 3) = 0;
	(*this)(3, 0) = 0;
	(*this)(3, 1) = 0;
	(*this)(3, 2) = 0;
	(*this)(3, 3) = 1;
}

template <typename TRAIT>
const typename iit::pegasus2::tpl::HomogeneousTransforms<TRAIT>::Type_fr_RF_hip_X_fr_RF_upperleg& 
iit::pegasus2::tpl::HomogeneousTransforms<TRAIT>::Type_fr_RF_hip_X_fr_RF_upperleg::update(const JState& q){ 
	Scalar c__q_RF_HFE_joint__;
	Scalar s__q_RF_HFE_joint__;

	c__q_RF_HFE_joint__ = TRAIT::cos( q[RF_HFE_JOINT]);
	s__q_RF_HFE_joint__ = TRAIT::sin( q[RF_HFE_JOINT]);

	(*this)(0, 0) = -c__q_RF_HFE_joint__;
	(*this)(0, 1) = s__q_RF_HFE_joint__;
	(*this)(2, 0) = -s__q_RF_HFE_joint__;
	(*this)(2, 1) = -c__q_RF_HFE_joint__;
	return *this;
}

template <typename TRAIT>
iit::pegasus2::tpl::HomogeneousTransforms<TRAIT>::Type_fr_RF_upperleg_X_fr_RF_hip::Type_fr_RF_upperleg_X_fr_RF_hip(){ 
	(*this)(0, 1) = 0;
	(*this)(0, 3) = 0;
	(*this)(1, 1) = 0;
	(*this)(1, 3) = 0;
	(*this)(2, 0) = 0;
	(*this)(2, 1) = -1;
	(*this)(2, 2) = 0;
	(*this)(2, 3) = -0.0500000000000000;
	(*this)(3, 0) = 0;
	(*this)(3, 1) = 0;
	(*this)(3, 2) = 0;
	(*this)(3, 3) = 1;
}

template <typename TRAIT>
const typename iit::pegasus2::tpl::HomogeneousTransforms<TRAIT>::Type_fr_RF_upperleg_X_fr_RF_hip& 
iit::pegasus2::tpl::HomogeneousTransforms<TRAIT>::Type_fr_RF_upperleg_X_fr_RF_hip::update(const JState& q){ 
	Scalar c__q_RF_HFE_joint__;
	Scalar s__q_RF_HFE_joint__;

	c__q_RF_HFE_joint__ = TRAIT::cos( q[RF_HFE_JOINT]);
	s__q_RF_HFE_joint__ = TRAIT::sin( q[RF_HFE_JOINT]);

	(*this)(0, 0) = -c__q_RF_HFE_joint__;
	(*this)(0, 2) = -s__q_RF_HFE_joint__;
	(*this)(1, 0) = s__q_RF_HFE_joint__;
	(*this)(1, 2) = -c__q_RF_HFE_joint__;
	return *this;
}

template <typename TRAIT>
iit::pegasus2::tpl::HomogeneousTransforms<TRAIT>::Type_fr_RF_upperleg_X_fr_RF_lowerleg::Type_fr_RF_upperleg_X_fr_RF_lowerleg(){ 
	(*this)(0, 2) = 0;
	(*this)(0, 3) = 0.380000000000000;
	(*this)(1, 2) = 0;
	(*this)(1, 3) = 0;
	(*this)(2, 0) = 0;
	(*this)(2, 1) = 0;
	(*this)(2, 2) = 1;
	(*this)(2, 3) = 0;
	(*this)(3, 0) = 0;
	(*this)(3, 1) = 0;
	(*this)(3, 2) = 0;
	(*this)(3, 3) = 1;
}

template <typename TRAIT>
const typename iit::pegasus2::tpl::HomogeneousTransforms<TRAIT>::Type_fr_RF_upperleg_X_fr_RF_lowerleg& 
iit::pegasus2::tpl::HomogeneousTransforms<TRAIT>::Type_fr_RF_upperleg_X_fr_RF_lowerleg::update(const JState& q){ 
	Scalar c__q_RF_KFE_joint__;
	Scalar s__q_RF_KFE_joint__;

	c__q_RF_KFE_joint__ = TRAIT::cos( q[RF_KFE_JOINT]);
	s__q_RF_KFE_joint__ = TRAIT::sin( q[RF_KFE_JOINT]);

	(*this)(0, 0) = c__q_RF_KFE_joint__;
	(*this)(0, 1) = -s__q_RF_KFE_joint__;
	(*this)(1, 0) = s__q_RF_KFE_joint__;
	(*this)(1, 1) = c__q_RF_KFE_joint__;
	return *this;
}

template <typename TRAIT>
iit::pegasus2::tpl::HomogeneousTransforms<TRAIT>::Type_fr_RF_lowerleg_X_fr_RF_upperleg::Type_fr_RF_lowerleg_X_fr_RF_upperleg(){ 
	(*this)(0, 2) = 0;
	(*this)(1, 2) = 0;
	(*this)(2, 0) = 0;
	(*this)(2, 1) = 0;
	(*this)(2, 2) = 1;
	(*this)(2, 3) = 0;
	(*this)(3, 0) = 0;
	(*this)(3, 1) = 0;
	(*this)(3, 2) = 0;
	(*this)(3, 3) = 1;
}

template <typename TRAIT>
const typename iit::pegasus2::tpl::HomogeneousTransforms<TRAIT>::Type_fr_RF_lowerleg_X_fr_RF_upperleg& 
iit::pegasus2::tpl::HomogeneousTransforms<TRAIT>::Type_fr_RF_lowerleg_X_fr_RF_upperleg::update(const JState& q){ 
	Scalar c__q_RF_KFE_joint__;
	Scalar s__q_RF_KFE_joint__;

	c__q_RF_KFE_joint__ = TRAIT::cos( q[RF_KFE_JOINT]);
	s__q_RF_KFE_joint__ = TRAIT::sin( q[RF_KFE_JOINT]);

	(*this)(0, 0) = c__q_RF_KFE_joint__;
	(*this)(0, 1) = s__q_RF_KFE_joint__;
	(*this)(0, 3) = -0.38*c__q_RF_KFE_joint__;
	(*this)(1, 0) = -s__q_RF_KFE_joint__;
	(*this)(1, 1) = c__q_RF_KFE_joint__;
	(*this)(1, 3) = 0.38*s__q_RF_KFE_joint__;
	return *this;
}

template <typename TRAIT>
iit::pegasus2::tpl::HomogeneousTransforms<TRAIT>::Type_fr_RH_hip_X_fr_RH_upperleg::Type_fr_RH_hip_X_fr_RH_upperleg(){ 
	(*this)(0, 2) = 0;
	(*this)(0, 3) = 0;
	(*this)(1, 0) = 0;
	(*this)(1, 1) = 0;
	(*this)(1, 2) = -1;
	(*this)(1, 3) = -0.0500000000000000;
	(*this)(2, 2) = 0;
	(*this)(2, 3) = 0;
	(*this)(3, 0) = 0;
	(*this)(3, 1) = 0;
	(*this)(3, 2) = 0;
	(*this)(3, 3) = 1;
}

template <typename TRAIT>
const typename iit::pegasus2::tpl::HomogeneousTransforms<TRAIT>::Type_fr_RH_hip_X_fr_RH_upperleg& 
iit::pegasus2::tpl::HomogeneousTransforms<TRAIT>::Type_fr_RH_hip_X_fr_RH_upperleg::update(const JState& q){ 
	Scalar c__q_RH_HFE_joint__;
	Scalar s__q_RH_HFE_joint__;

	c__q_RH_HFE_joint__ = TRAIT::cos( q[RH_HFE_JOINT]);
	s__q_RH_HFE_joint__ = TRAIT::sin( q[RH_HFE_JOINT]);

	(*this)(0, 0) = -c__q_RH_HFE_joint__;
	(*this)(0, 1) = s__q_RH_HFE_joint__;
	(*this)(2, 0) = -s__q_RH_HFE_joint__;
	(*this)(2, 1) = -c__q_RH_HFE_joint__;
	return *this;
}

template <typename TRAIT>
iit::pegasus2::tpl::HomogeneousTransforms<TRAIT>::Type_fr_RH_upperleg_X_fr_RH_hip::Type_fr_RH_upperleg_X_fr_RH_hip(){ 
	(*this)(0, 1) = 0;
	(*this)(0, 3) = 0;
	(*this)(1, 1) = 0;
	(*this)(1, 3) = 0;
	(*this)(2, 0) = 0;
	(*this)(2, 1) = -1;
	(*this)(2, 2) = 0;
	(*this)(2, 3) = -0.0500000000000000;
	(*this)(3, 0) = 0;
	(*this)(3, 1) = 0;
	(*this)(3, 2) = 0;
	(*this)(3, 3) = 1;
}

template <typename TRAIT>
const typename iit::pegasus2::tpl::HomogeneousTransforms<TRAIT>::Type_fr_RH_upperleg_X_fr_RH_hip& 
iit::pegasus2::tpl::HomogeneousTransforms<TRAIT>::Type_fr_RH_upperleg_X_fr_RH_hip::update(const JState& q){ 
	Scalar c__q_RH_HFE_joint__;
	Scalar s__q_RH_HFE_joint__;

	c__q_RH_HFE_joint__ = TRAIT::cos( q[RH_HFE_JOINT]);
	s__q_RH_HFE_joint__ = TRAIT::sin( q[RH_HFE_JOINT]);

	(*this)(0, 0) = -c__q_RH_HFE_joint__;
	(*this)(0, 2) = -s__q_RH_HFE_joint__;
	(*this)(1, 0) = s__q_RH_HFE_joint__;
	(*this)(1, 2) = -c__q_RH_HFE_joint__;
	return *this;
}

template <typename TRAIT>
iit::pegasus2::tpl::HomogeneousTransforms<TRAIT>::Type_fr_RH_upperleg_X_fr_RH_lowerleg::Type_fr_RH_upperleg_X_fr_RH_lowerleg(){ 
	(*this)(0, 2) = 0;
	(*this)(0, 3) = 0.380000000000000;
	(*this)(1, 2) = 0;
	(*this)(1, 3) = 0;
	(*this)(2, 0) = 0;
	(*this)(2, 1) = 0;
	(*this)(2, 2) = 1;
	(*this)(2, 3) = 0;
	(*this)(3, 0) = 0;
	(*this)(3, 1) = 0;
	(*this)(3, 2) = 0;
	(*this)(3, 3) = 1;
}

template <typename TRAIT>
const typename iit::pegasus2::tpl::HomogeneousTransforms<TRAIT>::Type_fr_RH_upperleg_X_fr_RH_lowerleg& 
iit::pegasus2::tpl::HomogeneousTransforms<TRAIT>::Type_fr_RH_upperleg_X_fr_RH_lowerleg::update(const JState& q){ 
	Scalar c__q_RH_KFE_joint__;
	Scalar s__q_RH_KFE_joint__;

	c__q_RH_KFE_joint__ = TRAIT::cos( q[RH_KFE_JOINT]);
	s__q_RH_KFE_joint__ = TRAIT::sin( q[RH_KFE_JOINT]);

	(*this)(0, 0) = c__q_RH_KFE_joint__;
	(*this)(0, 1) = -s__q_RH_KFE_joint__;
	(*this)(1, 0) = s__q_RH_KFE_joint__;
	(*this)(1, 1) = c__q_RH_KFE_joint__;
	return *this;
}

template <typename TRAIT>
iit::pegasus2::tpl::HomogeneousTransforms<TRAIT>::Type_fr_RH_lowerleg_X_fr_RH_upperleg::Type_fr_RH_lowerleg_X_fr_RH_upperleg(){ 
	(*this)(0, 2) = 0;
	(*this)(1, 2) = 0;
	(*this)(2, 0) = 0;
	(*this)(2, 1) = 0;
	(*this)(2, 2) = 1;
	(*this)(2, 3) = 0;
	(*this)(3, 0) = 0;
	(*this)(3, 1) = 0;
	(*this)(3, 2) = 0;
	(*this)(3, 3) = 1;
}

template <typename TRAIT>
const typename iit::pegasus2::tpl::HomogeneousTransforms<TRAIT>::Type_fr_RH_lowerleg_X_fr_RH_upperleg& 
iit::pegasus2::tpl::HomogeneousTransforms<TRAIT>::Type_fr_RH_lowerleg_X_fr_RH_upperleg::update(const JState& q){ 
	Scalar c__q_RH_KFE_joint__;
	Scalar s__q_RH_KFE_joint__;

	c__q_RH_KFE_joint__ = TRAIT::cos( q[RH_KFE_JOINT]);
	s__q_RH_KFE_joint__ = TRAIT::sin( q[RH_KFE_JOINT]);

	(*this)(0, 0) = c__q_RH_KFE_joint__;
	(*this)(0, 1) = s__q_RH_KFE_joint__;
	(*this)(0, 3) = -0.38*c__q_RH_KFE_joint__;
	(*this)(1, 0) = -s__q_RH_KFE_joint__;
	(*this)(1, 1) = c__q_RH_KFE_joint__;
	(*this)(1, 3) = 0.38*s__q_RH_KFE_joint__;
	return *this;
}

template <typename TRAIT>
iit::pegasus2::tpl::HomogeneousTransforms<TRAIT>::Type_fr_LH_hip_X_fr_LH_upperleg::Type_fr_LH_hip_X_fr_LH_upperleg(){ 
	(*this)(0, 2) = 0;
	(*this)(0, 3) = 0;
	(*this)(1, 0) = 0;
	(*this)(1, 1) = 0;
	(*this)(1, 2) = -1;
	(*this)(1, 3) = 0.0500000000000000;
	(*this)(2, 2) = 0;
	(*this)(2, 3) = 0;
	(*this)(3, 0) = 0;
	(*this)(3, 1) = 0;
	(*this)(3, 2) = 0;
	(*this)(3, 3) = 1;
}

template <typename TRAIT>
const typename iit::pegasus2::tpl::HomogeneousTransforms<TRAIT>::Type_fr_LH_hip_X_fr_LH_upperleg& 
iit::pegasus2::tpl::HomogeneousTransforms<TRAIT>::Type_fr_LH_hip_X_fr_LH_upperleg::update(const JState& q){ 
	Scalar c__q_LH_HFE_joint__;
	Scalar s__q_LH_HFE_joint__;

	c__q_LH_HFE_joint__ = TRAIT::cos( q[LH_HFE_JOINT]);
	s__q_LH_HFE_joint__ = TRAIT::sin( q[LH_HFE_JOINT]);

	(*this)(0, 0) = -c__q_LH_HFE_joint__;
	(*this)(0, 1) = s__q_LH_HFE_joint__;
	(*this)(2, 0) = -s__q_LH_HFE_joint__;
	(*this)(2, 1) = -c__q_LH_HFE_joint__;
	return *this;
}

template <typename TRAIT>
iit::pegasus2::tpl::HomogeneousTransforms<TRAIT>::Type_fr_LH_upperleg_X_fr_LH_hip::Type_fr_LH_upperleg_X_fr_LH_hip(){ 
	(*this)(0, 1) = 0;
	(*this)(0, 3) = 0;
	(*this)(1, 1) = 0;
	(*this)(1, 3) = 0;
	(*this)(2, 0) = 0;
	(*this)(2, 1) = -1;
	(*this)(2, 2) = 0;
	(*this)(2, 3) = 0.0500000000000000;
	(*this)(3, 0) = 0;
	(*this)(3, 1) = 0;
	(*this)(3, 2) = 0;
	(*this)(3, 3) = 1;
}

template <typename TRAIT>
const typename iit::pegasus2::tpl::HomogeneousTransforms<TRAIT>::Type_fr_LH_upperleg_X_fr_LH_hip& 
iit::pegasus2::tpl::HomogeneousTransforms<TRAIT>::Type_fr_LH_upperleg_X_fr_LH_hip::update(const JState& q){ 
	Scalar c__q_LH_HFE_joint__;
	Scalar s__q_LH_HFE_joint__;

	c__q_LH_HFE_joint__ = TRAIT::cos( q[LH_HFE_JOINT]);
	s__q_LH_HFE_joint__ = TRAIT::sin( q[LH_HFE_JOINT]);

	(*this)(0, 0) = -c__q_LH_HFE_joint__;
	(*this)(0, 2) = -s__q_LH_HFE_joint__;
	(*this)(1, 0) = s__q_LH_HFE_joint__;
	(*this)(1, 2) = -c__q_LH_HFE_joint__;
	return *this;
}

template <typename TRAIT>
iit::pegasus2::tpl::HomogeneousTransforms<TRAIT>::Type_fr_LH_upperleg_X_fr_LH_lowerleg::Type_fr_LH_upperleg_X_fr_LH_lowerleg(){ 
	(*this)(0, 2) = 0;
	(*this)(0, 3) = 0.380000000000000;
	(*this)(1, 2) = 0;
	(*this)(1, 3) = 0;
	(*this)(2, 0) = 0;
	(*this)(2, 1) = 0;
	(*this)(2, 2) = 1;
	(*this)(2, 3) = 0;
	(*this)(3, 0) = 0;
	(*this)(3, 1) = 0;
	(*this)(3, 2) = 0;
	(*this)(3, 3) = 1;
}

template <typename TRAIT>
const typename iit::pegasus2::tpl::HomogeneousTransforms<TRAIT>::Type_fr_LH_upperleg_X_fr_LH_lowerleg& 
iit::pegasus2::tpl::HomogeneousTransforms<TRAIT>::Type_fr_LH_upperleg_X_fr_LH_lowerleg::update(const JState& q){ 
	Scalar c__q_LH_KFE_joint__;
	Scalar s__q_LH_KFE_joint__;

	c__q_LH_KFE_joint__ = TRAIT::cos( q[LH_KFE_JOINT]);
	s__q_LH_KFE_joint__ = TRAIT::sin( q[LH_KFE_JOINT]);

	(*this)(0, 0) = c__q_LH_KFE_joint__;
	(*this)(0, 1) = -s__q_LH_KFE_joint__;
	(*this)(1, 0) = s__q_LH_KFE_joint__;
	(*this)(1, 1) = c__q_LH_KFE_joint__;
	return *this;
}

template <typename TRAIT>
iit::pegasus2::tpl::HomogeneousTransforms<TRAIT>::Type_fr_LH_lowerleg_X_fr_LH_upperleg::Type_fr_LH_lowerleg_X_fr_LH_upperleg(){ 
	(*this)(0, 2) = 0;
	(*this)(1, 2) = 0;
	(*this)(2, 0) = 0;
	(*this)(2, 1) = 0;
	(*this)(2, 2) = 1;
	(*this)(2, 3) = 0;
	(*this)(3, 0) = 0;
	(*this)(3, 1) = 0;
	(*this)(3, 2) = 0;
	(*this)(3, 3) = 1;
}

template <typename TRAIT>
const typename iit::pegasus2::tpl::HomogeneousTransforms<TRAIT>::Type_fr_LH_lowerleg_X_fr_LH_upperleg& 
iit::pegasus2::tpl::HomogeneousTransforms<TRAIT>::Type_fr_LH_lowerleg_X_fr_LH_upperleg::update(const JState& q){ 
	Scalar c__q_LH_KFE_joint__;
	Scalar s__q_LH_KFE_joint__;

	c__q_LH_KFE_joint__ = TRAIT::cos( q[LH_KFE_JOINT]);
	s__q_LH_KFE_joint__ = TRAIT::sin( q[LH_KFE_JOINT]);

	(*this)(0, 0) = c__q_LH_KFE_joint__;
	(*this)(0, 1) = s__q_LH_KFE_joint__;
	(*this)(0, 3) = -0.38*c__q_LH_KFE_joint__;
	(*this)(1, 0) = -s__q_LH_KFE_joint__;
	(*this)(1, 1) = c__q_LH_KFE_joint__;
	(*this)(1, 3) = 0.38*s__q_LH_KFE_joint__;
	return *this;
}

template <typename TRAIT>
iit::pegasus2::tpl::HomogeneousTransforms<TRAIT>::Type_fr_LF_hip_X_fr_LF_upperleg::Type_fr_LF_hip_X_fr_LF_upperleg(){ 
	(*this)(0, 2) = 0;
	(*this)(0, 3) = 0;
	(*this)(1, 0) = 0;
	(*this)(1, 1) = 0;
	(*this)(1, 2) = -1;
	(*this)(1, 3) = 0.0500000000000000;
	(*this)(2, 2) = 0;
	(*this)(2, 3) = 0;
	(*this)(3, 0) = 0;
	(*this)(3, 1) = 0;
	(*this)(3, 2) = 0;
	(*this)(3, 3) = 1;
}

template <typename TRAIT>
const typename iit::pegasus2::tpl::HomogeneousTransforms<TRAIT>::Type_fr_LF_hip_X_fr_LF_upperleg& 
iit::pegasus2::tpl::HomogeneousTransforms<TRAIT>::Type_fr_LF_hip_X_fr_LF_upperleg::update(const JState& q){ 
	Scalar c__q_LF_HFE_joint__;
	Scalar s__q_LF_HFE_joint__;

	c__q_LF_HFE_joint__ = TRAIT::cos( q[LF_HFE_JOINT]);
	s__q_LF_HFE_joint__ = TRAIT::sin( q[LF_HFE_JOINT]);

	(*this)(0, 0) = -c__q_LF_HFE_joint__;
	(*this)(0, 1) = s__q_LF_HFE_joint__;
	(*this)(2, 0) = -s__q_LF_HFE_joint__;
	(*this)(2, 1) = -c__q_LF_HFE_joint__;
	return *this;
}

template <typename TRAIT>
iit::pegasus2::tpl::HomogeneousTransforms<TRAIT>::Type_fr_LF_upperleg_X_fr_LF_hip::Type_fr_LF_upperleg_X_fr_LF_hip(){ 
	(*this)(0, 1) = 0;
	(*this)(0, 3) = 0;
	(*this)(1, 1) = 0;
	(*this)(1, 3) = 0;
	(*this)(2, 0) = 0;
	(*this)(2, 1) = -1;
	(*this)(2, 2) = 0;
	(*this)(2, 3) = 0.0500000000000000;
	(*this)(3, 0) = 0;
	(*this)(3, 1) = 0;
	(*this)(3, 2) = 0;
	(*this)(3, 3) = 1;
}

template <typename TRAIT>
const typename iit::pegasus2::tpl::HomogeneousTransforms<TRAIT>::Type_fr_LF_upperleg_X_fr_LF_hip& 
iit::pegasus2::tpl::HomogeneousTransforms<TRAIT>::Type_fr_LF_upperleg_X_fr_LF_hip::update(const JState& q){ 
	Scalar c__q_LF_HFE_joint__;
	Scalar s__q_LF_HFE_joint__;

	c__q_LF_HFE_joint__ = TRAIT::cos( q[LF_HFE_JOINT]);
	s__q_LF_HFE_joint__ = TRAIT::sin( q[LF_HFE_JOINT]);

	(*this)(0, 0) = -c__q_LF_HFE_joint__;
	(*this)(0, 2) = -s__q_LF_HFE_joint__;
	(*this)(1, 0) = s__q_LF_HFE_joint__;
	(*this)(1, 2) = -c__q_LF_HFE_joint__;
	return *this;
}

template <typename TRAIT>
iit::pegasus2::tpl::HomogeneousTransforms<TRAIT>::Type_fr_LF_upperleg_X_fr_LF_lowerleg::Type_fr_LF_upperleg_X_fr_LF_lowerleg(){ 
	(*this)(0, 2) = 0;
	(*this)(0, 3) = 0.380000000000000;
	(*this)(1, 2) = 0;
	(*this)(1, 3) = 0;
	(*this)(2, 0) = 0;
	(*this)(2, 1) = 0;
	(*this)(2, 2) = 1;
	(*this)(2, 3) = 0;
	(*this)(3, 0) = 0;
	(*this)(3, 1) = 0;
	(*this)(3, 2) = 0;
	(*this)(3, 3) = 1;
}

template <typename TRAIT>
const typename iit::pegasus2::tpl::HomogeneousTransforms<TRAIT>::Type_fr_LF_upperleg_X_fr_LF_lowerleg& 
iit::pegasus2::tpl::HomogeneousTransforms<TRAIT>::Type_fr_LF_upperleg_X_fr_LF_lowerleg::update(const JState& q){ 
	Scalar c__q_LF_KFE_joint__;
	Scalar s__q_LF_KFE_joint__;

	c__q_LF_KFE_joint__ = TRAIT::cos( q[LF_KFE_JOINT]);
	s__q_LF_KFE_joint__ = TRAIT::sin( q[LF_KFE_JOINT]);

	(*this)(0, 0) = c__q_LF_KFE_joint__;
	(*this)(0, 1) = -s__q_LF_KFE_joint__;
	(*this)(1, 0) = s__q_LF_KFE_joint__;
	(*this)(1, 1) = c__q_LF_KFE_joint__;
	return *this;
}

template <typename TRAIT>
iit::pegasus2::tpl::HomogeneousTransforms<TRAIT>::Type_fr_LF_lowerleg_X_fr_LF_upperleg::Type_fr_LF_lowerleg_X_fr_LF_upperleg(){ 
	(*this)(0, 2) = 0;
	(*this)(1, 2) = 0;
	(*this)(2, 0) = 0;
	(*this)(2, 1) = 0;
	(*this)(2, 2) = 1;
	(*this)(2, 3) = 0;
	(*this)(3, 0) = 0;
	(*this)(3, 1) = 0;
	(*this)(3, 2) = 0;
	(*this)(3, 3) = 1;
}

template <typename TRAIT>
const typename iit::pegasus2::tpl::HomogeneousTransforms<TRAIT>::Type_fr_LF_lowerleg_X_fr_LF_upperleg& 
iit::pegasus2::tpl::HomogeneousTransforms<TRAIT>::Type_fr_LF_lowerleg_X_fr_LF_upperleg::update(const JState& q){ 
	Scalar c__q_LF_KFE_joint__;
	Scalar s__q_LF_KFE_joint__;

	c__q_LF_KFE_joint__ = TRAIT::cos( q[LF_KFE_JOINT]);
	s__q_LF_KFE_joint__ = TRAIT::sin( q[LF_KFE_JOINT]);

	(*this)(0, 0) = c__q_LF_KFE_joint__;
	(*this)(0, 1) = s__q_LF_KFE_joint__;
	(*this)(0, 3) = -0.38*c__q_LF_KFE_joint__;
	(*this)(1, 0) = -s__q_LF_KFE_joint__;
	(*this)(1, 1) = c__q_LF_KFE_joint__;
	(*this)(1, 3) = 0.38*s__q_LF_KFE_joint__;
	return *this;
}

template <typename TRAIT>
iit::pegasus2::tpl::HomogeneousTransforms<TRAIT>::Type_fr_torso_X_fr_RF_hip::Type_fr_torso_X_fr_RF_hip(){ 
	(*this)(0, 0) = 0;
	(*this)(0, 1) = 0;
	(*this)(0, 2) = -1;
	(*this)(0, 3) = 0.370000000000000;
	(*this)(1, 2) = 0;
	(*this)(1, 3) = -0.118500000000000;
	(*this)(2, 2) = 0;
	(*this)(2, 3) = 0;
	(*this)(3, 0) = 0;
	(*this)(3, 1) = 0;
	(*this)(3, 2) = 0;
	(*this)(3, 3) = 1;
}

template <typename TRAIT>
const typename iit::pegasus2::tpl::HomogeneousTransforms<TRAIT>::Type_fr_torso_X_fr_RF_hip& 
iit::pegasus2::tpl::HomogeneousTransforms<TRAIT>::Type_fr_torso_X_fr_RF_hip::update(const JState& q){ 
	Scalar c__q_RF_HAA_joint__;
	Scalar s__q_RF_HAA_joint__;

	c__q_RF_HAA_joint__ = TRAIT::cos( q[RF_HAA_JOINT]);
	s__q_RF_HAA_joint__ = TRAIT::sin( q[RF_HAA_JOINT]);

	(*this)(1, 0) = s__q_RF_HAA_joint__;
	(*this)(1, 1) = c__q_RF_HAA_joint__;
	(*this)(2, 0) = c__q_RF_HAA_joint__;
	(*this)(2, 1) = -s__q_RF_HAA_joint__;
	return *this;
}

template <typename TRAIT>
iit::pegasus2::tpl::HomogeneousTransforms<TRAIT>::Type_fr_RF_hip_X_fr_torso::Type_fr_RF_hip_X_fr_torso(){ 
	(*this)(0, 0) = 0;
	(*this)(1, 0) = 0;
	(*this)(2, 0) = -1;
	(*this)(2, 1) = 0;
	(*this)(2, 2) = 0;
	(*this)(2, 3) = 0.370000000000000;
	(*this)(3, 0) = 0;
	(*this)(3, 1) = 0;
	(*this)(3, 2) = 0;
	(*this)(3, 3) = 1;
}

template <typename TRAIT>
const typename iit::pegasus2::tpl::HomogeneousTransforms<TRAIT>::Type_fr_RF_hip_X_fr_torso& 
iit::pegasus2::tpl::HomogeneousTransforms<TRAIT>::Type_fr_RF_hip_X_fr_torso::update(const JState& q){ 
	Scalar c__q_RF_HAA_joint__;
	Scalar s__q_RF_HAA_joint__;

	c__q_RF_HAA_joint__ = TRAIT::cos( q[RF_HAA_JOINT]);
	s__q_RF_HAA_joint__ = TRAIT::sin( q[RF_HAA_JOINT]);

	(*this)(0, 1) = s__q_RF_HAA_joint__;
	(*this)(0, 2) = c__q_RF_HAA_joint__;
	(*this)(0, 3) = 0.1185*s__q_RF_HAA_joint__;
	(*this)(1, 1) = c__q_RF_HAA_joint__;
	(*this)(1, 2) = -s__q_RF_HAA_joint__;
	(*this)(1, 3) = 0.1185*c__q_RF_HAA_joint__;
	return *this;
}

template <typename TRAIT>
iit::pegasus2::tpl::HomogeneousTransforms<TRAIT>::Type_fr_torso_X_fr_RF_upperleg::Type_fr_torso_X_fr_RF_upperleg(){ 
	(*this)(0, 2) = 0;
	(*this)(0, 3) = 0.370000000000000;
	(*this)(3, 0) = 0;
	(*this)(3, 1) = 0;
	(*this)(3, 2) = 0;
	(*this)(3, 3) = 1;
}

template <typename TRAIT>
const typename iit::pegasus2::tpl::HomogeneousTransforms<TRAIT>::Type_fr_torso_X_fr_RF_upperleg& 
iit::pegasus2::tpl::HomogeneousTransforms<TRAIT>::Type_fr_torso_X_fr_RF_upperleg::update(const JState& q){ 
	Scalar c__q_RF_HAA_joint__;
	Scalar c__q_RF_HFE_joint__;
	Scalar s__q_RF_HFE_joint__;
	Scalar s__q_RF_HAA_joint__;

	c__q_RF_HAA_joint__ = TRAIT::cos( q[RF_HAA_JOINT]);
	c__q_RF_HFE_joint__ = TRAIT::cos( q[RF_HFE_JOINT]);
	s__q_RF_HFE_joint__ = TRAIT::sin( q[RF_HFE_JOINT]);
	s__q_RF_HAA_joint__ = TRAIT::sin( q[RF_HAA_JOINT]);

	(*this)(0, 0) = s__q_RF_HFE_joint__;
	(*this)(0, 1) = c__q_RF_HFE_joint__;
	(*this)(1, 0) = -s__q_RF_HAA_joint__*c__q_RF_HFE_joint__;
	(*this)(1, 1) = s__q_RF_HAA_joint__*s__q_RF_HFE_joint__;
	(*this)(1, 2) = -c__q_RF_HAA_joint__;
	(*this)(1, 3) = -0.05*c__q_RF_HAA_joint__ - 0.1185;
	(*this)(2, 0) = -c__q_RF_HAA_joint__*c__q_RF_HFE_joint__;
	(*this)(2, 1) = s__q_RF_HFE_joint__*c__q_RF_HAA_joint__;
	(*this)(2, 2) = s__q_RF_HAA_joint__;
	(*this)(2, 3) = 0.05*s__q_RF_HAA_joint__;
	return *this;
}

template <typename TRAIT>
iit::pegasus2::tpl::HomogeneousTransforms<TRAIT>::Type_fr_RF_upperleg_X_fr_torso::Type_fr_RF_upperleg_X_fr_torso(){ 
	(*this)(2, 0) = 0;
	(*this)(3, 0) = 0;
	(*this)(3, 1) = 0;
	(*this)(3, 2) = 0;
	(*this)(3, 3) = 1;
}

template <typename TRAIT>
const typename iit::pegasus2::tpl::HomogeneousTransforms<TRAIT>::Type_fr_RF_upperleg_X_fr_torso& 
iit::pegasus2::tpl::HomogeneousTransforms<TRAIT>::Type_fr_RF_upperleg_X_fr_torso::update(const JState& q){ 
	Scalar c__q_RF_HAA_joint__;
	Scalar c__q_RF_HFE_joint__;
	Scalar s__q_RF_HFE_joint__;
	Scalar s__q_RF_HAA_joint__;

	c__q_RF_HAA_joint__ = TRAIT::cos( q[RF_HAA_JOINT]);
	c__q_RF_HFE_joint__ = TRAIT::cos( q[RF_HFE_JOINT]);
	s__q_RF_HFE_joint__ = TRAIT::sin( q[RF_HFE_JOINT]);
	s__q_RF_HAA_joint__ = TRAIT::sin( q[RF_HAA_JOINT]);

	(*this)(0, 0) = s__q_RF_HFE_joint__;
	(*this)(0, 1) = -s__q_RF_HAA_joint__*c__q_RF_HFE_joint__;
	(*this)(0, 2) = -c__q_RF_HAA_joint__*c__q_RF_HFE_joint__;
	(*this)(0, 3) = -0.1185*s__q_RF_HAA_joint__*c__q_RF_HFE_joint__ - 0.37*s__q_RF_HFE_joint__;
	(*this)(1, 0) = c__q_RF_HFE_joint__;
	(*this)(1, 1) = s__q_RF_HAA_joint__*s__q_RF_HFE_joint__;
	(*this)(1, 2) = s__q_RF_HFE_joint__*c__q_RF_HAA_joint__;
	(*this)(1, 3) = 0.1185*s__q_RF_HAA_joint__*s__q_RF_HFE_joint__ - 0.37*c__q_RF_HFE_joint__;
	(*this)(2, 1) = -c__q_RF_HAA_joint__;
	(*this)(2, 2) = s__q_RF_HAA_joint__;
	(*this)(2, 3) = -0.1185*c__q_RF_HAA_joint__ - 0.05;
	return *this;
}

template <typename TRAIT>
iit::pegasus2::tpl::HomogeneousTransforms<TRAIT>::Type_fr_torso_X_fr_RF_lowerleg::Type_fr_torso_X_fr_RF_lowerleg(){ 
	(*this)(0, 2) = 0;
	(*this)(3, 0) = 0;
	(*this)(3, 1) = 0;
	(*this)(3, 2) = 0;
	(*this)(3, 3) = 1;
}

template <typename TRAIT>
const typename iit::pegasus2::tpl::HomogeneousTransforms<TRAIT>::Type_fr_torso_X_fr_RF_lowerleg& 
iit::pegasus2::tpl::HomogeneousTransforms<TRAIT>::Type_fr_torso_X_fr_RF_lowerleg::update(const JState& q){ 
	Scalar c__q_RF_KFE_joint__;
	Scalar c__q_RF_HAA_joint__;
	Scalar c__q_RF_HFE_joint__;
	Scalar s__q_RF_HFE_joint__;
	Scalar s__q_RF_HAA_joint__;
	Scalar s__q_RF_KFE_joint__;

	c__q_RF_KFE_joint__ = TRAIT::cos( q[RF_KFE_JOINT]);
	c__q_RF_HAA_joint__ = TRAIT::cos( q[RF_HAA_JOINT]);
	c__q_RF_HFE_joint__ = TRAIT::cos( q[RF_HFE_JOINT]);
	s__q_RF_HFE_joint__ = TRAIT::sin( q[RF_HFE_JOINT]);
	s__q_RF_HAA_joint__ = TRAIT::sin( q[RF_HAA_JOINT]);
	s__q_RF_KFE_joint__ = TRAIT::sin( q[RF_KFE_JOINT]);

	(*this)(0, 0) = s__q_RF_HFE_joint__*c__q_RF_KFE_joint__ + s__q_RF_KFE_joint__*c__q_RF_HFE_joint__;
	(*this)(0, 1) = -s__q_RF_HFE_joint__*s__q_RF_KFE_joint__ + c__q_RF_HFE_joint__*c__q_RF_KFE_joint__;
	(*this)(0, 3) = 0.38*s__q_RF_HFE_joint__ + 0.37;
	(*this)(1, 0) = -(-s__q_RF_HFE_joint__*s__q_RF_KFE_joint__ + c__q_RF_HFE_joint__*c__q_RF_KFE_joint__)*s__q_RF_HAA_joint__;
	(*this)(1, 1) = (s__q_RF_HFE_joint__*c__q_RF_KFE_joint__ + s__q_RF_KFE_joint__*c__q_RF_HFE_joint__)*s__q_RF_HAA_joint__;
	(*this)(1, 2) = -c__q_RF_HAA_joint__;
	(*this)(1, 3) = -0.38*s__q_RF_HAA_joint__*c__q_RF_HFE_joint__ - 0.05*c__q_RF_HAA_joint__ - 0.1185;
	(*this)(2, 0) = -(-s__q_RF_HFE_joint__*s__q_RF_KFE_joint__ + c__q_RF_HFE_joint__*c__q_RF_KFE_joint__)*c__q_RF_HAA_joint__;
	(*this)(2, 1) = (s__q_RF_HFE_joint__*c__q_RF_KFE_joint__ + s__q_RF_KFE_joint__*c__q_RF_HFE_joint__)*c__q_RF_HAA_joint__;
	(*this)(2, 2) = s__q_RF_HAA_joint__;
	(*this)(2, 3) = 0.05*s__q_RF_HAA_joint__ - 0.38*c__q_RF_HAA_joint__*c__q_RF_HFE_joint__;
	return *this;
}

template <typename TRAIT>
iit::pegasus2::tpl::HomogeneousTransforms<TRAIT>::Type_fr_RF_lowerleg_X_fr_torso::Type_fr_RF_lowerleg_X_fr_torso(){ 
	(*this)(2, 0) = 0;
	(*this)(3, 0) = 0;
	(*this)(3, 1) = 0;
	(*this)(3, 2) = 0;
	(*this)(3, 3) = 1;
}

template <typename TRAIT>
const typename iit::pegasus2::tpl::HomogeneousTransforms<TRAIT>::Type_fr_RF_lowerleg_X_fr_torso& 
iit::pegasus2::tpl::HomogeneousTransforms<TRAIT>::Type_fr_RF_lowerleg_X_fr_torso::update(const JState& q){ 
	Scalar c__q_RF_KFE_joint__;
	Scalar c__q_RF_HAA_joint__;
	Scalar c__q_RF_HFE_joint__;
	Scalar s__q_RF_HFE_joint__;
	Scalar s__q_RF_HAA_joint__;
	Scalar s__q_RF_KFE_joint__;

	c__q_RF_KFE_joint__ = TRAIT::cos( q[RF_KFE_JOINT]);
	c__q_RF_HAA_joint__ = TRAIT::cos( q[RF_HAA_JOINT]);
	c__q_RF_HFE_joint__ = TRAIT::cos( q[RF_HFE_JOINT]);
	s__q_RF_HFE_joint__ = TRAIT::sin( q[RF_HFE_JOINT]);
	s__q_RF_HAA_joint__ = TRAIT::sin( q[RF_HAA_JOINT]);
	s__q_RF_KFE_joint__ = TRAIT::sin( q[RF_KFE_JOINT]);

	(*this)(0, 0) = s__q_RF_HFE_joint__*c__q_RF_KFE_joint__ + s__q_RF_KFE_joint__*c__q_RF_HFE_joint__;
	(*this)(0, 1) = -(-s__q_RF_HFE_joint__*s__q_RF_KFE_joint__ + c__q_RF_HFE_joint__*c__q_RF_KFE_joint__)*s__q_RF_HAA_joint__;
	(*this)(0, 2) = -(-s__q_RF_HFE_joint__*s__q_RF_KFE_joint__ + c__q_RF_HFE_joint__*c__q_RF_KFE_joint__)*c__q_RF_HAA_joint__;
	(*this)(0, 3) = -0.1185*(-s__q_RF_HFE_joint__*s__q_RF_KFE_joint__ + c__q_RF_HFE_joint__*c__q_RF_KFE_joint__)*s__q_RF_HAA_joint__ - 0.37*s__q_RF_HFE_joint__*c__q_RF_KFE_joint__ - 0.37*s__q_RF_KFE_joint__*c__q_RF_HFE_joint__ - 0.38*c__q_RF_KFE_joint__;
	(*this)(1, 0) = -s__q_RF_HFE_joint__*s__q_RF_KFE_joint__ + c__q_RF_HFE_joint__*c__q_RF_KFE_joint__;
	(*this)(1, 1) = (s__q_RF_HFE_joint__*c__q_RF_KFE_joint__ + s__q_RF_KFE_joint__*c__q_RF_HFE_joint__)*s__q_RF_HAA_joint__;
	(*this)(1, 2) = (s__q_RF_HFE_joint__*c__q_RF_KFE_joint__ + s__q_RF_KFE_joint__*c__q_RF_HFE_joint__)*c__q_RF_HAA_joint__;
	(*this)(1, 3) = 0.1185*(s__q_RF_HFE_joint__*c__q_RF_KFE_joint__ + s__q_RF_KFE_joint__*c__q_RF_HFE_joint__)*s__q_RF_HAA_joint__ + 0.37*s__q_RF_HFE_joint__*s__q_RF_KFE_joint__ + 0.38*s__q_RF_KFE_joint__ - 0.37*c__q_RF_HFE_joint__*c__q_RF_KFE_joint__;
	(*this)(2, 1) = -c__q_RF_HAA_joint__;
	(*this)(2, 2) = s__q_RF_HAA_joint__;
	(*this)(2, 3) = -0.1185*c__q_RF_HAA_joint__ - 0.05;
	return *this;
}

template <typename TRAIT>
iit::pegasus2::tpl::HomogeneousTransforms<TRAIT>::Type_fr_torso_X_fr_RH_hip::Type_fr_torso_X_fr_RH_hip(){ 
	(*this)(0, 0) = 0;
	(*this)(0, 1) = 0;
	(*this)(0, 2) = -1;
	(*this)(0, 3) = -0.370000000000000;
	(*this)(1, 2) = 0;
	(*this)(1, 3) = -0.118500000000000;
	(*this)(2, 2) = 0;
	(*this)(2, 3) = 0;
	(*this)(3, 0) = 0;
	(*this)(3, 1) = 0;
	(*this)(3, 2) = 0;
	(*this)(3, 3) = 1;
}

template <typename TRAIT>
const typename iit::pegasus2::tpl::HomogeneousTransforms<TRAIT>::Type_fr_torso_X_fr_RH_hip& 
iit::pegasus2::tpl::HomogeneousTransforms<TRAIT>::Type_fr_torso_X_fr_RH_hip::update(const JState& q){ 
	Scalar c__q_RH_HAA_joint__;
	Scalar s__q_RH_HAA_joint__;

	c__q_RH_HAA_joint__ = TRAIT::cos( q[RH_HAA_JOINT]);
	s__q_RH_HAA_joint__ = TRAIT::sin( q[RH_HAA_JOINT]);

	(*this)(1, 0) = s__q_RH_HAA_joint__;
	(*this)(1, 1) = c__q_RH_HAA_joint__;
	(*this)(2, 0) = c__q_RH_HAA_joint__;
	(*this)(2, 1) = -s__q_RH_HAA_joint__;
	return *this;
}

template <typename TRAIT>
iit::pegasus2::tpl::HomogeneousTransforms<TRAIT>::Type_fr_RH_hip_X_fr_torso::Type_fr_RH_hip_X_fr_torso(){ 
	(*this)(0, 0) = 0;
	(*this)(1, 0) = 0;
	(*this)(2, 0) = -1;
	(*this)(2, 1) = 0;
	(*this)(2, 2) = 0;
	(*this)(2, 3) = -0.370000000000000;
	(*this)(3, 0) = 0;
	(*this)(3, 1) = 0;
	(*this)(3, 2) = 0;
	(*this)(3, 3) = 1;
}

template <typename TRAIT>
const typename iit::pegasus2::tpl::HomogeneousTransforms<TRAIT>::Type_fr_RH_hip_X_fr_torso& 
iit::pegasus2::tpl::HomogeneousTransforms<TRAIT>::Type_fr_RH_hip_X_fr_torso::update(const JState& q){ 
	Scalar c__q_RH_HAA_joint__;
	Scalar s__q_RH_HAA_joint__;

	c__q_RH_HAA_joint__ = TRAIT::cos( q[RH_HAA_JOINT]);
	s__q_RH_HAA_joint__ = TRAIT::sin( q[RH_HAA_JOINT]);

	(*this)(0, 1) = s__q_RH_HAA_joint__;
	(*this)(0, 2) = c__q_RH_HAA_joint__;
	(*this)(0, 3) = 0.1185*s__q_RH_HAA_joint__;
	(*this)(1, 1) = c__q_RH_HAA_joint__;
	(*this)(1, 2) = -s__q_RH_HAA_joint__;
	(*this)(1, 3) = 0.1185*c__q_RH_HAA_joint__;
	return *this;
}

template <typename TRAIT>
iit::pegasus2::tpl::HomogeneousTransforms<TRAIT>::Type_fr_torso_X_fr_RH_upperleg::Type_fr_torso_X_fr_RH_upperleg(){ 
	(*this)(0, 2) = 0;
	(*this)(0, 3) = -0.370000000000000;
	(*this)(3, 0) = 0;
	(*this)(3, 1) = 0;
	(*this)(3, 2) = 0;
	(*this)(3, 3) = 1;
}

template <typename TRAIT>
const typename iit::pegasus2::tpl::HomogeneousTransforms<TRAIT>::Type_fr_torso_X_fr_RH_upperleg& 
iit::pegasus2::tpl::HomogeneousTransforms<TRAIT>::Type_fr_torso_X_fr_RH_upperleg::update(const JState& q){ 
	Scalar c__q_RH_HFE_joint__;
	Scalar c__q_RH_HAA_joint__;
	Scalar s__q_RH_HAA_joint__;
	Scalar s__q_RH_HFE_joint__;

	c__q_RH_HFE_joint__ = TRAIT::cos( q[RH_HFE_JOINT]);
	c__q_RH_HAA_joint__ = TRAIT::cos( q[RH_HAA_JOINT]);
	s__q_RH_HAA_joint__ = TRAIT::sin( q[RH_HAA_JOINT]);
	s__q_RH_HFE_joint__ = TRAIT::sin( q[RH_HFE_JOINT]);

	(*this)(0, 0) = s__q_RH_HFE_joint__;
	(*this)(0, 1) = c__q_RH_HFE_joint__;
	(*this)(1, 0) = -s__q_RH_HAA_joint__*c__q_RH_HFE_joint__;
	(*this)(1, 1) = s__q_RH_HAA_joint__*s__q_RH_HFE_joint__;
	(*this)(1, 2) = -c__q_RH_HAA_joint__;
	(*this)(1, 3) = -0.05*c__q_RH_HAA_joint__ - 0.1185;
	(*this)(2, 0) = -c__q_RH_HAA_joint__*c__q_RH_HFE_joint__;
	(*this)(2, 1) = s__q_RH_HFE_joint__*c__q_RH_HAA_joint__;
	(*this)(2, 2) = s__q_RH_HAA_joint__;
	(*this)(2, 3) = 0.05*s__q_RH_HAA_joint__;
	return *this;
}

template <typename TRAIT>
iit::pegasus2::tpl::HomogeneousTransforms<TRAIT>::Type_fr_RH_upperleg_X_fr_torso::Type_fr_RH_upperleg_X_fr_torso(){ 
	(*this)(2, 0) = 0;
	(*this)(3, 0) = 0;
	(*this)(3, 1) = 0;
	(*this)(3, 2) = 0;
	(*this)(3, 3) = 1;
}

template <typename TRAIT>
const typename iit::pegasus2::tpl::HomogeneousTransforms<TRAIT>::Type_fr_RH_upperleg_X_fr_torso& 
iit::pegasus2::tpl::HomogeneousTransforms<TRAIT>::Type_fr_RH_upperleg_X_fr_torso::update(const JState& q){ 
	Scalar c__q_RH_HFE_joint__;
	Scalar c__q_RH_HAA_joint__;
	Scalar s__q_RH_HAA_joint__;
	Scalar s__q_RH_HFE_joint__;

	c__q_RH_HFE_joint__ = TRAIT::cos( q[RH_HFE_JOINT]);
	c__q_RH_HAA_joint__ = TRAIT::cos( q[RH_HAA_JOINT]);
	s__q_RH_HAA_joint__ = TRAIT::sin( q[RH_HAA_JOINT]);
	s__q_RH_HFE_joint__ = TRAIT::sin( q[RH_HFE_JOINT]);

	(*this)(0, 0) = s__q_RH_HFE_joint__;
	(*this)(0, 1) = -s__q_RH_HAA_joint__*c__q_RH_HFE_joint__;
	(*this)(0, 2) = -c__q_RH_HAA_joint__*c__q_RH_HFE_joint__;
	(*this)(0, 3) = -0.1185*s__q_RH_HAA_joint__*c__q_RH_HFE_joint__ + 0.37*s__q_RH_HFE_joint__;
	(*this)(1, 0) = c__q_RH_HFE_joint__;
	(*this)(1, 1) = s__q_RH_HAA_joint__*s__q_RH_HFE_joint__;
	(*this)(1, 2) = s__q_RH_HFE_joint__*c__q_RH_HAA_joint__;
	(*this)(1, 3) = 0.1185*s__q_RH_HAA_joint__*s__q_RH_HFE_joint__ + 0.37*c__q_RH_HFE_joint__;
	(*this)(2, 1) = -c__q_RH_HAA_joint__;
	(*this)(2, 2) = s__q_RH_HAA_joint__;
	(*this)(2, 3) = -0.1185*c__q_RH_HAA_joint__ - 0.05;
	return *this;
}

template <typename TRAIT>
iit::pegasus2::tpl::HomogeneousTransforms<TRAIT>::Type_fr_torso_X_fr_RH_lowerleg::Type_fr_torso_X_fr_RH_lowerleg(){ 
	(*this)(0, 2) = 0;
	(*this)(3, 0) = 0;
	(*this)(3, 1) = 0;
	(*this)(3, 2) = 0;
	(*this)(3, 3) = 1;
}

template <typename TRAIT>
const typename iit::pegasus2::tpl::HomogeneousTransforms<TRAIT>::Type_fr_torso_X_fr_RH_lowerleg& 
iit::pegasus2::tpl::HomogeneousTransforms<TRAIT>::Type_fr_torso_X_fr_RH_lowerleg::update(const JState& q){ 
	Scalar c__q_RH_HFE_joint__;
	Scalar c__q_RH_HAA_joint__;
	Scalar c__q_RH_KFE_joint__;
	Scalar s__q_RH_KFE_joint__;
	Scalar s__q_RH_HAA_joint__;
	Scalar s__q_RH_HFE_joint__;

	c__q_RH_HFE_joint__ = TRAIT::cos( q[RH_HFE_JOINT]);
	c__q_RH_HAA_joint__ = TRAIT::cos( q[RH_HAA_JOINT]);
	c__q_RH_KFE_joint__ = TRAIT::cos( q[RH_KFE_JOINT]);
	s__q_RH_KFE_joint__ = TRAIT::sin( q[RH_KFE_JOINT]);
	s__q_RH_HAA_joint__ = TRAIT::sin( q[RH_HAA_JOINT]);
	s__q_RH_HFE_joint__ = TRAIT::sin( q[RH_HFE_JOINT]);

	(*this)(0, 0) = s__q_RH_HFE_joint__*c__q_RH_KFE_joint__ + s__q_RH_KFE_joint__*c__q_RH_HFE_joint__;
	(*this)(0, 1) = -s__q_RH_HFE_joint__*s__q_RH_KFE_joint__ + c__q_RH_HFE_joint__*c__q_RH_KFE_joint__;
	(*this)(0, 3) = 0.38*s__q_RH_HFE_joint__ - 0.37;
	(*this)(1, 0) = -(-s__q_RH_HFE_joint__*s__q_RH_KFE_joint__ + c__q_RH_HFE_joint__*c__q_RH_KFE_joint__)*s__q_RH_HAA_joint__;
	(*this)(1, 1) = (s__q_RH_HFE_joint__*c__q_RH_KFE_joint__ + s__q_RH_KFE_joint__*c__q_RH_HFE_joint__)*s__q_RH_HAA_joint__;
	(*this)(1, 2) = -c__q_RH_HAA_joint__;
	(*this)(1, 3) = -0.38*s__q_RH_HAA_joint__*c__q_RH_HFE_joint__ - 0.05*c__q_RH_HAA_joint__ - 0.1185;
	(*this)(2, 0) = -(-s__q_RH_HFE_joint__*s__q_RH_KFE_joint__ + c__q_RH_HFE_joint__*c__q_RH_KFE_joint__)*c__q_RH_HAA_joint__;
	(*this)(2, 1) = (s__q_RH_HFE_joint__*c__q_RH_KFE_joint__ + s__q_RH_KFE_joint__*c__q_RH_HFE_joint__)*c__q_RH_HAA_joint__;
	(*this)(2, 2) = s__q_RH_HAA_joint__;
	(*this)(2, 3) = 0.05*s__q_RH_HAA_joint__ - 0.38*c__q_RH_HAA_joint__*c__q_RH_HFE_joint__;
	return *this;
}

template <typename TRAIT>
iit::pegasus2::tpl::HomogeneousTransforms<TRAIT>::Type_fr_RH_lowerleg_X_fr_torso::Type_fr_RH_lowerleg_X_fr_torso(){ 
	(*this)(2, 0) = 0;
	(*this)(3, 0) = 0;
	(*this)(3, 1) = 0;
	(*this)(3, 2) = 0;
	(*this)(3, 3) = 1;
}

template <typename TRAIT>
const typename iit::pegasus2::tpl::HomogeneousTransforms<TRAIT>::Type_fr_RH_lowerleg_X_fr_torso& 
iit::pegasus2::tpl::HomogeneousTransforms<TRAIT>::Type_fr_RH_lowerleg_X_fr_torso::update(const JState& q){ 
	Scalar c__q_RH_HFE_joint__;
	Scalar c__q_RH_HAA_joint__;
	Scalar c__q_RH_KFE_joint__;
	Scalar s__q_RH_KFE_joint__;
	Scalar s__q_RH_HAA_joint__;
	Scalar s__q_RH_HFE_joint__;

	c__q_RH_HFE_joint__ = TRAIT::cos( q[RH_HFE_JOINT]);
	c__q_RH_HAA_joint__ = TRAIT::cos( q[RH_HAA_JOINT]);
	c__q_RH_KFE_joint__ = TRAIT::cos( q[RH_KFE_JOINT]);
	s__q_RH_KFE_joint__ = TRAIT::sin( q[RH_KFE_JOINT]);
	s__q_RH_HAA_joint__ = TRAIT::sin( q[RH_HAA_JOINT]);
	s__q_RH_HFE_joint__ = TRAIT::sin( q[RH_HFE_JOINT]);

	(*this)(0, 0) = s__q_RH_HFE_joint__*c__q_RH_KFE_joint__ + s__q_RH_KFE_joint__*c__q_RH_HFE_joint__;
	(*this)(0, 1) = -(-s__q_RH_HFE_joint__*s__q_RH_KFE_joint__ + c__q_RH_HFE_joint__*c__q_RH_KFE_joint__)*s__q_RH_HAA_joint__;
	(*this)(0, 2) = -(-s__q_RH_HFE_joint__*s__q_RH_KFE_joint__ + c__q_RH_HFE_joint__*c__q_RH_KFE_joint__)*c__q_RH_HAA_joint__;
	(*this)(0, 3) = -0.1185*(-s__q_RH_HFE_joint__*s__q_RH_KFE_joint__ + c__q_RH_HFE_joint__*c__q_RH_KFE_joint__)*s__q_RH_HAA_joint__ + 0.37*s__q_RH_HFE_joint__*c__q_RH_KFE_joint__ + 0.37*s__q_RH_KFE_joint__*c__q_RH_HFE_joint__ - 0.38*c__q_RH_KFE_joint__;
	(*this)(1, 0) = -s__q_RH_HFE_joint__*s__q_RH_KFE_joint__ + c__q_RH_HFE_joint__*c__q_RH_KFE_joint__;
	(*this)(1, 1) = (s__q_RH_HFE_joint__*c__q_RH_KFE_joint__ + s__q_RH_KFE_joint__*c__q_RH_HFE_joint__)*s__q_RH_HAA_joint__;
	(*this)(1, 2) = (s__q_RH_HFE_joint__*c__q_RH_KFE_joint__ + s__q_RH_KFE_joint__*c__q_RH_HFE_joint__)*c__q_RH_HAA_joint__;
	(*this)(1, 3) = 0.1185*(s__q_RH_HFE_joint__*c__q_RH_KFE_joint__ + s__q_RH_KFE_joint__*c__q_RH_HFE_joint__)*s__q_RH_HAA_joint__ - 0.37*s__q_RH_HFE_joint__*s__q_RH_KFE_joint__ + 0.38*s__q_RH_KFE_joint__ + 0.37*c__q_RH_HFE_joint__*c__q_RH_KFE_joint__;
	(*this)(2, 1) = -c__q_RH_HAA_joint__;
	(*this)(2, 2) = s__q_RH_HAA_joint__;
	(*this)(2, 3) = -0.1185*c__q_RH_HAA_joint__ - 0.05;
	return *this;
}

template <typename TRAIT>
iit::pegasus2::tpl::HomogeneousTransforms<TRAIT>::Type_fr_torso_X_fr_LH_hip::Type_fr_torso_X_fr_LH_hip(){ 
	(*this)(0, 0) = 0;
	(*this)(0, 1) = 0;
	(*this)(0, 2) = -1;
	(*this)(0, 3) = -0.370000000000000;
	(*this)(1, 2) = 0;
	(*this)(1, 3) = 0.118500000000000;
	(*this)(2, 2) = 0;
	(*this)(2, 3) = 0;
	(*this)(3, 0) = 0;
	(*this)(3, 1) = 0;
	(*this)(3, 2) = 0;
	(*this)(3, 3) = 1;
}

template <typename TRAIT>
const typename iit::pegasus2::tpl::HomogeneousTransforms<TRAIT>::Type_fr_torso_X_fr_LH_hip& 
iit::pegasus2::tpl::HomogeneousTransforms<TRAIT>::Type_fr_torso_X_fr_LH_hip::update(const JState& q){ 
	Scalar c__q_LH_HAA_joint__;
	Scalar s__q_LH_HAA_joint__;

	c__q_LH_HAA_joint__ = TRAIT::cos( q[LH_HAA_JOINT]);
	s__q_LH_HAA_joint__ = TRAIT::sin( q[LH_HAA_JOINT]);

	(*this)(1, 0) = s__q_LH_HAA_joint__;
	(*this)(1, 1) = c__q_LH_HAA_joint__;
	(*this)(2, 0) = c__q_LH_HAA_joint__;
	(*this)(2, 1) = -s__q_LH_HAA_joint__;
	return *this;
}

template <typename TRAIT>
iit::pegasus2::tpl::HomogeneousTransforms<TRAIT>::Type_fr_LH_hip_X_fr_torso::Type_fr_LH_hip_X_fr_torso(){ 
	(*this)(0, 0) = 0;
	(*this)(1, 0) = 0;
	(*this)(2, 0) = -1;
	(*this)(2, 1) = 0;
	(*this)(2, 2) = 0;
	(*this)(2, 3) = -0.370000000000000;
	(*this)(3, 0) = 0;
	(*this)(3, 1) = 0;
	(*this)(3, 2) = 0;
	(*this)(3, 3) = 1;
}

template <typename TRAIT>
const typename iit::pegasus2::tpl::HomogeneousTransforms<TRAIT>::Type_fr_LH_hip_X_fr_torso& 
iit::pegasus2::tpl::HomogeneousTransforms<TRAIT>::Type_fr_LH_hip_X_fr_torso::update(const JState& q){ 
	Scalar c__q_LH_HAA_joint__;
	Scalar s__q_LH_HAA_joint__;

	c__q_LH_HAA_joint__ = TRAIT::cos( q[LH_HAA_JOINT]);
	s__q_LH_HAA_joint__ = TRAIT::sin( q[LH_HAA_JOINT]);

	(*this)(0, 1) = s__q_LH_HAA_joint__;
	(*this)(0, 2) = c__q_LH_HAA_joint__;
	(*this)(0, 3) = -0.1185*s__q_LH_HAA_joint__;
	(*this)(1, 1) = c__q_LH_HAA_joint__;
	(*this)(1, 2) = -s__q_LH_HAA_joint__;
	(*this)(1, 3) = -0.1185*c__q_LH_HAA_joint__;
	return *this;
}

template <typename TRAIT>
iit::pegasus2::tpl::HomogeneousTransforms<TRAIT>::Type_fr_torso_X_fr_LH_upperleg::Type_fr_torso_X_fr_LH_upperleg(){ 
	(*this)(0, 2) = 0;
	(*this)(0, 3) = -0.370000000000000;
	(*this)(3, 0) = 0;
	(*this)(3, 1) = 0;
	(*this)(3, 2) = 0;
	(*this)(3, 3) = 1;
}

template <typename TRAIT>
const typename iit::pegasus2::tpl::HomogeneousTransforms<TRAIT>::Type_fr_torso_X_fr_LH_upperleg& 
iit::pegasus2::tpl::HomogeneousTransforms<TRAIT>::Type_fr_torso_X_fr_LH_upperleg::update(const JState& q){ 
	Scalar c__q_LH_HAA_joint__;
	Scalar c__q_LH_HFE_joint__;
	Scalar s__q_LH_HFE_joint__;
	Scalar s__q_LH_HAA_joint__;

	c__q_LH_HAA_joint__ = TRAIT::cos( q[LH_HAA_JOINT]);
	c__q_LH_HFE_joint__ = TRAIT::cos( q[LH_HFE_JOINT]);
	s__q_LH_HFE_joint__ = TRAIT::sin( q[LH_HFE_JOINT]);
	s__q_LH_HAA_joint__ = TRAIT::sin( q[LH_HAA_JOINT]);

	(*this)(0, 0) = s__q_LH_HFE_joint__;
	(*this)(0, 1) = c__q_LH_HFE_joint__;
	(*this)(1, 0) = -s__q_LH_HAA_joint__*c__q_LH_HFE_joint__;
	(*this)(1, 1) = s__q_LH_HAA_joint__*s__q_LH_HFE_joint__;
	(*this)(1, 2) = -c__q_LH_HAA_joint__;
	(*this)(1, 3) = 0.05*c__q_LH_HAA_joint__ + 0.1185;
	(*this)(2, 0) = -c__q_LH_HAA_joint__*c__q_LH_HFE_joint__;
	(*this)(2, 1) = s__q_LH_HFE_joint__*c__q_LH_HAA_joint__;
	(*this)(2, 2) = s__q_LH_HAA_joint__;
	(*this)(2, 3) = -0.05*s__q_LH_HAA_joint__;
	return *this;
}

template <typename TRAIT>
iit::pegasus2::tpl::HomogeneousTransforms<TRAIT>::Type_fr_LH_upperleg_X_fr_torso::Type_fr_LH_upperleg_X_fr_torso(){ 
	(*this)(2, 0) = 0;
	(*this)(3, 0) = 0;
	(*this)(3, 1) = 0;
	(*this)(3, 2) = 0;
	(*this)(3, 3) = 1;
}

template <typename TRAIT>
const typename iit::pegasus2::tpl::HomogeneousTransforms<TRAIT>::Type_fr_LH_upperleg_X_fr_torso& 
iit::pegasus2::tpl::HomogeneousTransforms<TRAIT>::Type_fr_LH_upperleg_X_fr_torso::update(const JState& q){ 
	Scalar c__q_LH_HAA_joint__;
	Scalar c__q_LH_HFE_joint__;
	Scalar s__q_LH_HFE_joint__;
	Scalar s__q_LH_HAA_joint__;

	c__q_LH_HAA_joint__ = TRAIT::cos( q[LH_HAA_JOINT]);
	c__q_LH_HFE_joint__ = TRAIT::cos( q[LH_HFE_JOINT]);
	s__q_LH_HFE_joint__ = TRAIT::sin( q[LH_HFE_JOINT]);
	s__q_LH_HAA_joint__ = TRAIT::sin( q[LH_HAA_JOINT]);

	(*this)(0, 0) = s__q_LH_HFE_joint__;
	(*this)(0, 1) = -s__q_LH_HAA_joint__*c__q_LH_HFE_joint__;
	(*this)(0, 2) = -c__q_LH_HAA_joint__*c__q_LH_HFE_joint__;
	(*this)(0, 3) = 0.1185*s__q_LH_HAA_joint__*c__q_LH_HFE_joint__ + 0.37*s__q_LH_HFE_joint__;
	(*this)(1, 0) = c__q_LH_HFE_joint__;
	(*this)(1, 1) = s__q_LH_HAA_joint__*s__q_LH_HFE_joint__;
	(*this)(1, 2) = s__q_LH_HFE_joint__*c__q_LH_HAA_joint__;
	(*this)(1, 3) = -0.1185*s__q_LH_HAA_joint__*s__q_LH_HFE_joint__ + 0.37*c__q_LH_HFE_joint__;
	(*this)(2, 1) = -c__q_LH_HAA_joint__;
	(*this)(2, 2) = s__q_LH_HAA_joint__;
	(*this)(2, 3) = 0.1185*c__q_LH_HAA_joint__ + 0.05;
	return *this;
}

template <typename TRAIT>
iit::pegasus2::tpl::HomogeneousTransforms<TRAIT>::Type_fr_torso_X_fr_LH_lowerleg::Type_fr_torso_X_fr_LH_lowerleg(){ 
	(*this)(0, 2) = 0;
	(*this)(3, 0) = 0;
	(*this)(3, 1) = 0;
	(*this)(3, 2) = 0;
	(*this)(3, 3) = 1;
}

template <typename TRAIT>
const typename iit::pegasus2::tpl::HomogeneousTransforms<TRAIT>::Type_fr_torso_X_fr_LH_lowerleg& 
iit::pegasus2::tpl::HomogeneousTransforms<TRAIT>::Type_fr_torso_X_fr_LH_lowerleg::update(const JState& q){ 
	Scalar c__q_LH_KFE_joint__;
	Scalar c__q_LH_HAA_joint__;
	Scalar c__q_LH_HFE_joint__;
	Scalar s__q_LH_HFE_joint__;
	Scalar s__q_LH_HAA_joint__;
	Scalar s__q_LH_KFE_joint__;

	c__q_LH_KFE_joint__ = TRAIT::cos( q[LH_KFE_JOINT]);
	c__q_LH_HAA_joint__ = TRAIT::cos( q[LH_HAA_JOINT]);
	c__q_LH_HFE_joint__ = TRAIT::cos( q[LH_HFE_JOINT]);
	s__q_LH_HFE_joint__ = TRAIT::sin( q[LH_HFE_JOINT]);
	s__q_LH_HAA_joint__ = TRAIT::sin( q[LH_HAA_JOINT]);
	s__q_LH_KFE_joint__ = TRAIT::sin( q[LH_KFE_JOINT]);

	(*this)(0, 0) = s__q_LH_HFE_joint__*c__q_LH_KFE_joint__ + s__q_LH_KFE_joint__*c__q_LH_HFE_joint__;
	(*this)(0, 1) = -s__q_LH_HFE_joint__*s__q_LH_KFE_joint__ + c__q_LH_HFE_joint__*c__q_LH_KFE_joint__;
	(*this)(0, 3) = 0.38*s__q_LH_HFE_joint__ - 0.37;
	(*this)(1, 0) = -(-s__q_LH_HFE_joint__*s__q_LH_KFE_joint__ + c__q_LH_HFE_joint__*c__q_LH_KFE_joint__)*s__q_LH_HAA_joint__;
	(*this)(1, 1) = (s__q_LH_HFE_joint__*c__q_LH_KFE_joint__ + s__q_LH_KFE_joint__*c__q_LH_HFE_joint__)*s__q_LH_HAA_joint__;
	(*this)(1, 2) = -c__q_LH_HAA_joint__;
	(*this)(1, 3) = -0.38*s__q_LH_HAA_joint__*c__q_LH_HFE_joint__ + 0.05*c__q_LH_HAA_joint__ + 0.1185;
	(*this)(2, 0) = -(-s__q_LH_HFE_joint__*s__q_LH_KFE_joint__ + c__q_LH_HFE_joint__*c__q_LH_KFE_joint__)*c__q_LH_HAA_joint__;
	(*this)(2, 1) = (s__q_LH_HFE_joint__*c__q_LH_KFE_joint__ + s__q_LH_KFE_joint__*c__q_LH_HFE_joint__)*c__q_LH_HAA_joint__;
	(*this)(2, 2) = s__q_LH_HAA_joint__;
	(*this)(2, 3) = -0.05*s__q_LH_HAA_joint__ - 0.38*c__q_LH_HAA_joint__*c__q_LH_HFE_joint__;
	return *this;
}

template <typename TRAIT>
iit::pegasus2::tpl::HomogeneousTransforms<TRAIT>::Type_fr_LH_lowerleg_X_fr_torso::Type_fr_LH_lowerleg_X_fr_torso(){ 
	(*this)(2, 0) = 0;
	(*this)(3, 0) = 0;
	(*this)(3, 1) = 0;
	(*this)(3, 2) = 0;
	(*this)(3, 3) = 1;
}

template <typename TRAIT>
const typename iit::pegasus2::tpl::HomogeneousTransforms<TRAIT>::Type_fr_LH_lowerleg_X_fr_torso& 
iit::pegasus2::tpl::HomogeneousTransforms<TRAIT>::Type_fr_LH_lowerleg_X_fr_torso::update(const JState& q){ 
	Scalar c__q_LH_KFE_joint__;
	Scalar c__q_LH_HAA_joint__;
	Scalar c__q_LH_HFE_joint__;
	Scalar s__q_LH_HFE_joint__;
	Scalar s__q_LH_HAA_joint__;
	Scalar s__q_LH_KFE_joint__;

	c__q_LH_KFE_joint__ = TRAIT::cos( q[LH_KFE_JOINT]);
	c__q_LH_HAA_joint__ = TRAIT::cos( q[LH_HAA_JOINT]);
	c__q_LH_HFE_joint__ = TRAIT::cos( q[LH_HFE_JOINT]);
	s__q_LH_HFE_joint__ = TRAIT::sin( q[LH_HFE_JOINT]);
	s__q_LH_HAA_joint__ = TRAIT::sin( q[LH_HAA_JOINT]);
	s__q_LH_KFE_joint__ = TRAIT::sin( q[LH_KFE_JOINT]);

	(*this)(0, 0) = s__q_LH_HFE_joint__*c__q_LH_KFE_joint__ + s__q_LH_KFE_joint__*c__q_LH_HFE_joint__;
	(*this)(0, 1) = -(-s__q_LH_HFE_joint__*s__q_LH_KFE_joint__ + c__q_LH_HFE_joint__*c__q_LH_KFE_joint__)*s__q_LH_HAA_joint__;
	(*this)(0, 2) = -(-s__q_LH_HFE_joint__*s__q_LH_KFE_joint__ + c__q_LH_HFE_joint__*c__q_LH_KFE_joint__)*c__q_LH_HAA_joint__;
	(*this)(0, 3) = 0.1185*(-s__q_LH_HFE_joint__*s__q_LH_KFE_joint__ + c__q_LH_HFE_joint__*c__q_LH_KFE_joint__)*s__q_LH_HAA_joint__ + 0.37*s__q_LH_HFE_joint__*c__q_LH_KFE_joint__ + 0.37*s__q_LH_KFE_joint__*c__q_LH_HFE_joint__ - 0.38*c__q_LH_KFE_joint__;
	(*this)(1, 0) = -s__q_LH_HFE_joint__*s__q_LH_KFE_joint__ + c__q_LH_HFE_joint__*c__q_LH_KFE_joint__;
	(*this)(1, 1) = (s__q_LH_HFE_joint__*c__q_LH_KFE_joint__ + s__q_LH_KFE_joint__*c__q_LH_HFE_joint__)*s__q_LH_HAA_joint__;
	(*this)(1, 2) = (s__q_LH_HFE_joint__*c__q_LH_KFE_joint__ + s__q_LH_KFE_joint__*c__q_LH_HFE_joint__)*c__q_LH_HAA_joint__;
	(*this)(1, 3) = -0.1185*(s__q_LH_HFE_joint__*c__q_LH_KFE_joint__ + s__q_LH_KFE_joint__*c__q_LH_HFE_joint__)*s__q_LH_HAA_joint__ - 0.37*s__q_LH_HFE_joint__*s__q_LH_KFE_joint__ + 0.38*s__q_LH_KFE_joint__ + 0.37*c__q_LH_HFE_joint__*c__q_LH_KFE_joint__;
	(*this)(2, 1) = -c__q_LH_HAA_joint__;
	(*this)(2, 2) = s__q_LH_HAA_joint__;
	(*this)(2, 3) = 0.1185*c__q_LH_HAA_joint__ + 0.05;
	return *this;
}

template <typename TRAIT>
iit::pegasus2::tpl::HomogeneousTransforms<TRAIT>::Type_fr_torso_X_fr_LF_hip::Type_fr_torso_X_fr_LF_hip(){ 
	(*this)(0, 0) = 0;
	(*this)(0, 1) = 0;
	(*this)(0, 2) = -1;
	(*this)(0, 3) = 0.370000000000000;
	(*this)(1, 2) = 0;
	(*this)(1, 3) = 0.118500000000000;
	(*this)(2, 2) = 0;
	(*this)(2, 3) = 0;
	(*this)(3, 0) = 0;
	(*this)(3, 1) = 0;
	(*this)(3, 2) = 0;
	(*this)(3, 3) = 1;
}

template <typename TRAIT>
const typename iit::pegasus2::tpl::HomogeneousTransforms<TRAIT>::Type_fr_torso_X_fr_LF_hip& 
iit::pegasus2::tpl::HomogeneousTransforms<TRAIT>::Type_fr_torso_X_fr_LF_hip::update(const JState& q){ 
	Scalar c__q_LF_HAA_joint__;
	Scalar s__q_LF_HAA_joint__;

	c__q_LF_HAA_joint__ = TRAIT::cos( q[LF_HAA_JOINT]);
	s__q_LF_HAA_joint__ = TRAIT::sin( q[LF_HAA_JOINT]);

	(*this)(1, 0) = s__q_LF_HAA_joint__;
	(*this)(1, 1) = c__q_LF_HAA_joint__;
	(*this)(2, 0) = c__q_LF_HAA_joint__;
	(*this)(2, 1) = -s__q_LF_HAA_joint__;
	return *this;
}

template <typename TRAIT>
iit::pegasus2::tpl::HomogeneousTransforms<TRAIT>::Type_fr_LF_hip_X_fr_torso::Type_fr_LF_hip_X_fr_torso(){ 
	(*this)(0, 0) = 0;
	(*this)(1, 0) = 0;
	(*this)(2, 0) = -1;
	(*this)(2, 1) = 0;
	(*this)(2, 2) = 0;
	(*this)(2, 3) = 0.370000000000000;
	(*this)(3, 0) = 0;
	(*this)(3, 1) = 0;
	(*this)(3, 2) = 0;
	(*this)(3, 3) = 1;
}

template <typename TRAIT>
const typename iit::pegasus2::tpl::HomogeneousTransforms<TRAIT>::Type_fr_LF_hip_X_fr_torso& 
iit::pegasus2::tpl::HomogeneousTransforms<TRAIT>::Type_fr_LF_hip_X_fr_torso::update(const JState& q){ 
	Scalar c__q_LF_HAA_joint__;
	Scalar s__q_LF_HAA_joint__;

	c__q_LF_HAA_joint__ = TRAIT::cos( q[LF_HAA_JOINT]);
	s__q_LF_HAA_joint__ = TRAIT::sin( q[LF_HAA_JOINT]);

	(*this)(0, 1) = s__q_LF_HAA_joint__;
	(*this)(0, 2) = c__q_LF_HAA_joint__;
	(*this)(0, 3) = -0.1185*s__q_LF_HAA_joint__;
	(*this)(1, 1) = c__q_LF_HAA_joint__;
	(*this)(1, 2) = -s__q_LF_HAA_joint__;
	(*this)(1, 3) = -0.1185*c__q_LF_HAA_joint__;
	return *this;
}

template <typename TRAIT>
iit::pegasus2::tpl::HomogeneousTransforms<TRAIT>::Type_fr_torso_X_fr_LF_upperleg::Type_fr_torso_X_fr_LF_upperleg(){ 
	(*this)(0, 2) = 0;
	(*this)(0, 3) = 0.370000000000000;
	(*this)(3, 0) = 0;
	(*this)(3, 1) = 0;
	(*this)(3, 2) = 0;
	(*this)(3, 3) = 1;
}

template <typename TRAIT>
const typename iit::pegasus2::tpl::HomogeneousTransforms<TRAIT>::Type_fr_torso_X_fr_LF_upperleg& 
iit::pegasus2::tpl::HomogeneousTransforms<TRAIT>::Type_fr_torso_X_fr_LF_upperleg::update(const JState& q){ 
	Scalar c__q_LF_HFE_joint__;
	Scalar c__q_LF_HAA_joint__;
	Scalar s__q_LF_HAA_joint__;
	Scalar s__q_LF_HFE_joint__;

	c__q_LF_HFE_joint__ = TRAIT::cos( q[LF_HFE_JOINT]);
	c__q_LF_HAA_joint__ = TRAIT::cos( q[LF_HAA_JOINT]);
	s__q_LF_HAA_joint__ = TRAIT::sin( q[LF_HAA_JOINT]);
	s__q_LF_HFE_joint__ = TRAIT::sin( q[LF_HFE_JOINT]);

	(*this)(0, 0) = s__q_LF_HFE_joint__;
	(*this)(0, 1) = c__q_LF_HFE_joint__;
	(*this)(1, 0) = -s__q_LF_HAA_joint__*c__q_LF_HFE_joint__;
	(*this)(1, 1) = s__q_LF_HAA_joint__*s__q_LF_HFE_joint__;
	(*this)(1, 2) = -c__q_LF_HAA_joint__;
	(*this)(1, 3) = 0.05*c__q_LF_HAA_joint__ + 0.1185;
	(*this)(2, 0) = -c__q_LF_HAA_joint__*c__q_LF_HFE_joint__;
	(*this)(2, 1) = s__q_LF_HFE_joint__*c__q_LF_HAA_joint__;
	(*this)(2, 2) = s__q_LF_HAA_joint__;
	(*this)(2, 3) = -0.05*s__q_LF_HAA_joint__;
	return *this;
}

template <typename TRAIT>
iit::pegasus2::tpl::HomogeneousTransforms<TRAIT>::Type_fr_LF_upperleg_X_fr_torso::Type_fr_LF_upperleg_X_fr_torso(){ 
	(*this)(2, 0) = 0;
	(*this)(3, 0) = 0;
	(*this)(3, 1) = 0;
	(*this)(3, 2) = 0;
	(*this)(3, 3) = 1;
}

template <typename TRAIT>
const typename iit::pegasus2::tpl::HomogeneousTransforms<TRAIT>::Type_fr_LF_upperleg_X_fr_torso& 
iit::pegasus2::tpl::HomogeneousTransforms<TRAIT>::Type_fr_LF_upperleg_X_fr_torso::update(const JState& q){ 
	Scalar c__q_LF_HFE_joint__;
	Scalar c__q_LF_HAA_joint__;
	Scalar s__q_LF_HAA_joint__;
	Scalar s__q_LF_HFE_joint__;

	c__q_LF_HFE_joint__ = TRAIT::cos( q[LF_HFE_JOINT]);
	c__q_LF_HAA_joint__ = TRAIT::cos( q[LF_HAA_JOINT]);
	s__q_LF_HAA_joint__ = TRAIT::sin( q[LF_HAA_JOINT]);
	s__q_LF_HFE_joint__ = TRAIT::sin( q[LF_HFE_JOINT]);

	(*this)(0, 0) = s__q_LF_HFE_joint__;
	(*this)(0, 1) = -s__q_LF_HAA_joint__*c__q_LF_HFE_joint__;
	(*this)(0, 2) = -c__q_LF_HAA_joint__*c__q_LF_HFE_joint__;
	(*this)(0, 3) = 0.1185*s__q_LF_HAA_joint__*c__q_LF_HFE_joint__ - 0.37*s__q_LF_HFE_joint__;
	(*this)(1, 0) = c__q_LF_HFE_joint__;
	(*this)(1, 1) = s__q_LF_HAA_joint__*s__q_LF_HFE_joint__;
	(*this)(1, 2) = s__q_LF_HFE_joint__*c__q_LF_HAA_joint__;
	(*this)(1, 3) = -0.1185*s__q_LF_HAA_joint__*s__q_LF_HFE_joint__ - 0.37*c__q_LF_HFE_joint__;
	(*this)(2, 1) = -c__q_LF_HAA_joint__;
	(*this)(2, 2) = s__q_LF_HAA_joint__;
	(*this)(2, 3) = 0.1185*c__q_LF_HAA_joint__ + 0.05;
	return *this;
}

template <typename TRAIT>
iit::pegasus2::tpl::HomogeneousTransforms<TRAIT>::Type_fr_torso_X_fr_LF_lowerleg::Type_fr_torso_X_fr_LF_lowerleg(){ 
	(*this)(0, 2) = 0;
	(*this)(3, 0) = 0;
	(*this)(3, 1) = 0;
	(*this)(3, 2) = 0;
	(*this)(3, 3) = 1;
}

template <typename TRAIT>
const typename iit::pegasus2::tpl::HomogeneousTransforms<TRAIT>::Type_fr_torso_X_fr_LF_lowerleg& 
iit::pegasus2::tpl::HomogeneousTransforms<TRAIT>::Type_fr_torso_X_fr_LF_lowerleg::update(const JState& q){ 
	Scalar c__q_LF_KFE_joint__;
	Scalar c__q_LF_HFE_joint__;
	Scalar c__q_LF_HAA_joint__;
	Scalar s__q_LF_HAA_joint__;
	Scalar s__q_LF_KFE_joint__;
	Scalar s__q_LF_HFE_joint__;

	c__q_LF_KFE_joint__ = TRAIT::cos( q[LF_KFE_JOINT]);
	c__q_LF_HFE_joint__ = TRAIT::cos( q[LF_HFE_JOINT]);
	c__q_LF_HAA_joint__ = TRAIT::cos( q[LF_HAA_JOINT]);
	s__q_LF_HAA_joint__ = TRAIT::sin( q[LF_HAA_JOINT]);
	s__q_LF_KFE_joint__ = TRAIT::sin( q[LF_KFE_JOINT]);
	s__q_LF_HFE_joint__ = TRAIT::sin( q[LF_HFE_JOINT]);

	(*this)(0, 0) = s__q_LF_HFE_joint__*c__q_LF_KFE_joint__ + s__q_LF_KFE_joint__*c__q_LF_HFE_joint__;
	(*this)(0, 1) = -s__q_LF_HFE_joint__*s__q_LF_KFE_joint__ + c__q_LF_HFE_joint__*c__q_LF_KFE_joint__;
	(*this)(0, 3) = 0.38*s__q_LF_HFE_joint__ + 0.37;
	(*this)(1, 0) = -(-s__q_LF_HFE_joint__*s__q_LF_KFE_joint__ + c__q_LF_HFE_joint__*c__q_LF_KFE_joint__)*s__q_LF_HAA_joint__;
	(*this)(1, 1) = (s__q_LF_HFE_joint__*c__q_LF_KFE_joint__ + s__q_LF_KFE_joint__*c__q_LF_HFE_joint__)*s__q_LF_HAA_joint__;
	(*this)(1, 2) = -c__q_LF_HAA_joint__;
	(*this)(1, 3) = -0.38*s__q_LF_HAA_joint__*c__q_LF_HFE_joint__ + 0.05*c__q_LF_HAA_joint__ + 0.1185;
	(*this)(2, 0) = -(-s__q_LF_HFE_joint__*s__q_LF_KFE_joint__ + c__q_LF_HFE_joint__*c__q_LF_KFE_joint__)*c__q_LF_HAA_joint__;
	(*this)(2, 1) = (s__q_LF_HFE_joint__*c__q_LF_KFE_joint__ + s__q_LF_KFE_joint__*c__q_LF_HFE_joint__)*c__q_LF_HAA_joint__;
	(*this)(2, 2) = s__q_LF_HAA_joint__;
	(*this)(2, 3) = -0.05*s__q_LF_HAA_joint__ - 0.38*c__q_LF_HAA_joint__*c__q_LF_HFE_joint__;
	return *this;
}

template <typename TRAIT>
iit::pegasus2::tpl::HomogeneousTransforms<TRAIT>::Type_fr_LF_lowerleg_X_fr_torso::Type_fr_LF_lowerleg_X_fr_torso(){ 
	(*this)(2, 0) = 0;
	(*this)(3, 0) = 0;
	(*this)(3, 1) = 0;
	(*this)(3, 2) = 0;
	(*this)(3, 3) = 1;
}

template <typename TRAIT>
const typename iit::pegasus2::tpl::HomogeneousTransforms<TRAIT>::Type_fr_LF_lowerleg_X_fr_torso& 
iit::pegasus2::tpl::HomogeneousTransforms<TRAIT>::Type_fr_LF_lowerleg_X_fr_torso::update(const JState& q){ 
	Scalar c__q_LF_KFE_joint__;
	Scalar c__q_LF_HFE_joint__;
	Scalar c__q_LF_HAA_joint__;
	Scalar s__q_LF_HAA_joint__;
	Scalar s__q_LF_KFE_joint__;
	Scalar s__q_LF_HFE_joint__;

	c__q_LF_KFE_joint__ = TRAIT::cos( q[LF_KFE_JOINT]);
	c__q_LF_HFE_joint__ = TRAIT::cos( q[LF_HFE_JOINT]);
	c__q_LF_HAA_joint__ = TRAIT::cos( q[LF_HAA_JOINT]);
	s__q_LF_HAA_joint__ = TRAIT::sin( q[LF_HAA_JOINT]);
	s__q_LF_KFE_joint__ = TRAIT::sin( q[LF_KFE_JOINT]);
	s__q_LF_HFE_joint__ = TRAIT::sin( q[LF_HFE_JOINT]);

	(*this)(0, 0) = s__q_LF_HFE_joint__*c__q_LF_KFE_joint__ + s__q_LF_KFE_joint__*c__q_LF_HFE_joint__;
	(*this)(0, 1) = -(-s__q_LF_HFE_joint__*s__q_LF_KFE_joint__ + c__q_LF_HFE_joint__*c__q_LF_KFE_joint__)*s__q_LF_HAA_joint__;
	(*this)(0, 2) = -(-s__q_LF_HFE_joint__*s__q_LF_KFE_joint__ + c__q_LF_HFE_joint__*c__q_LF_KFE_joint__)*c__q_LF_HAA_joint__;
	(*this)(0, 3) = 0.1185*(-s__q_LF_HFE_joint__*s__q_LF_KFE_joint__ + c__q_LF_HFE_joint__*c__q_LF_KFE_joint__)*s__q_LF_HAA_joint__ - 0.37*s__q_LF_HFE_joint__*c__q_LF_KFE_joint__ - 0.37*s__q_LF_KFE_joint__*c__q_LF_HFE_joint__ - 0.38*c__q_LF_KFE_joint__;
	(*this)(1, 0) = -s__q_LF_HFE_joint__*s__q_LF_KFE_joint__ + c__q_LF_HFE_joint__*c__q_LF_KFE_joint__;
	(*this)(1, 1) = (s__q_LF_HFE_joint__*c__q_LF_KFE_joint__ + s__q_LF_KFE_joint__*c__q_LF_HFE_joint__)*s__q_LF_HAA_joint__;
	(*this)(1, 2) = (s__q_LF_HFE_joint__*c__q_LF_KFE_joint__ + s__q_LF_KFE_joint__*c__q_LF_HFE_joint__)*c__q_LF_HAA_joint__;
	(*this)(1, 3) = -0.1185*(s__q_LF_HFE_joint__*c__q_LF_KFE_joint__ + s__q_LF_KFE_joint__*c__q_LF_HFE_joint__)*s__q_LF_HAA_joint__ + 0.37*s__q_LF_HFE_joint__*s__q_LF_KFE_joint__ + 0.38*s__q_LF_KFE_joint__ - 0.37*c__q_LF_HFE_joint__*c__q_LF_KFE_joint__;
	(*this)(2, 1) = -c__q_LF_HAA_joint__;
	(*this)(2, 2) = s__q_LF_HAA_joint__;
	(*this)(2, 3) = 0.1185*c__q_LF_HAA_joint__ + 0.05;
	return *this;
}

template <typename TRAIT>
iit::pegasus2::tpl::HomogeneousTransforms<TRAIT>::Type_fr_torso_X_fr_RF_foot::Type_fr_torso_X_fr_RF_foot(){ 
	(*this)(0, 2) = 0;
	(*this)(3, 0) = 0;
	(*this)(3, 1) = 0;
	(*this)(3, 2) = 0;
	(*this)(3, 3) = 1;
}

template <typename TRAIT>
const typename iit::pegasus2::tpl::HomogeneousTransforms<TRAIT>::Type_fr_torso_X_fr_RF_foot& 
iit::pegasus2::tpl::HomogeneousTransforms<TRAIT>::Type_fr_torso_X_fr_RF_foot::update(const JState& q){ 
	Scalar c__q_RF_KFE_joint__;
	Scalar c__q_RF_HAA_joint__;
	Scalar c__q_RF_HFE_joint__;
	Scalar s__q_RF_HFE_joint__;
	Scalar s__q_RF_HAA_joint__;
	Scalar s__q_RF_KFE_joint__;

	c__q_RF_KFE_joint__ = TRAIT::cos( q[RF_KFE_JOINT]);
	c__q_RF_HAA_joint__ = TRAIT::cos( q[RF_HAA_JOINT]);
	c__q_RF_HFE_joint__ = TRAIT::cos( q[RF_HFE_JOINT]);
	s__q_RF_HFE_joint__ = TRAIT::sin( q[RF_HFE_JOINT]);
	s__q_RF_HAA_joint__ = TRAIT::sin( q[RF_HAA_JOINT]);
	s__q_RF_KFE_joint__ = TRAIT::sin( q[RF_KFE_JOINT]);

	(*this)(0, 0) = s__q_RF_HFE_joint__*c__q_RF_KFE_joint__ + s__q_RF_KFE_joint__*c__q_RF_HFE_joint__;
	(*this)(0, 1) = -s__q_RF_HFE_joint__*s__q_RF_KFE_joint__ + c__q_RF_HFE_joint__*c__q_RF_KFE_joint__;
	(*this)(0, 3) = 0.3612*s__q_RF_HFE_joint__*c__q_RF_KFE_joint__ + 0.38*s__q_RF_HFE_joint__ + 0.3612*s__q_RF_KFE_joint__*c__q_RF_HFE_joint__ + 0.37;
	(*this)(1, 0) = -(-s__q_RF_HFE_joint__*s__q_RF_KFE_joint__ + c__q_RF_HFE_joint__*c__q_RF_KFE_joint__)*s__q_RF_HAA_joint__;
	(*this)(1, 1) = (s__q_RF_HFE_joint__*c__q_RF_KFE_joint__ + s__q_RF_KFE_joint__*c__q_RF_HFE_joint__)*s__q_RF_HAA_joint__;
	(*this)(1, 2) = -c__q_RF_HAA_joint__;
	(*this)(1, 3) = -0.3612*(-s__q_RF_HFE_joint__*s__q_RF_KFE_joint__ + c__q_RF_HFE_joint__*c__q_RF_KFE_joint__)*s__q_RF_HAA_joint__ - 0.38*s__q_RF_HAA_joint__*c__q_RF_HFE_joint__ - 0.05*c__q_RF_HAA_joint__ - 0.1185;
	(*this)(2, 0) = -(-s__q_RF_HFE_joint__*s__q_RF_KFE_joint__ + c__q_RF_HFE_joint__*c__q_RF_KFE_joint__)*c__q_RF_HAA_joint__;
	(*this)(2, 1) = (s__q_RF_HFE_joint__*c__q_RF_KFE_joint__ + s__q_RF_KFE_joint__*c__q_RF_HFE_joint__)*c__q_RF_HAA_joint__;
	(*this)(2, 2) = s__q_RF_HAA_joint__;
	(*this)(2, 3) = -0.3612*(-s__q_RF_HFE_joint__*s__q_RF_KFE_joint__ + c__q_RF_HFE_joint__*c__q_RF_KFE_joint__)*c__q_RF_HAA_joint__ + 0.05*s__q_RF_HAA_joint__ - 0.38*c__q_RF_HAA_joint__*c__q_RF_HFE_joint__;
	return *this;
}

template <typename TRAIT>
iit::pegasus2::tpl::HomogeneousTransforms<TRAIT>::Type_fr_RF_foot_X_fr_torso::Type_fr_RF_foot_X_fr_torso(){ 
	(*this)(2, 0) = 0;
	(*this)(3, 0) = 0;
	(*this)(3, 1) = 0;
	(*this)(3, 2) = 0;
	(*this)(3, 3) = 1;
}

template <typename TRAIT>
const typename iit::pegasus2::tpl::HomogeneousTransforms<TRAIT>::Type_fr_RF_foot_X_fr_torso& 
iit::pegasus2::tpl::HomogeneousTransforms<TRAIT>::Type_fr_RF_foot_X_fr_torso::update(const JState& q){ 
	Scalar c__q_RF_KFE_joint__;
	Scalar c__q_RF_HAA_joint__;
	Scalar c__q_RF_HFE_joint__;
	Scalar s__q_RF_HFE_joint__;
	Scalar s__q_RF_HAA_joint__;
	Scalar s__q_RF_KFE_joint__;

	c__q_RF_KFE_joint__ = TRAIT::cos( q[RF_KFE_JOINT]);
	c__q_RF_HAA_joint__ = TRAIT::cos( q[RF_HAA_JOINT]);
	c__q_RF_HFE_joint__ = TRAIT::cos( q[RF_HFE_JOINT]);
	s__q_RF_HFE_joint__ = TRAIT::sin( q[RF_HFE_JOINT]);
	s__q_RF_HAA_joint__ = TRAIT::sin( q[RF_HAA_JOINT]);
	s__q_RF_KFE_joint__ = TRAIT::sin( q[RF_KFE_JOINT]);

	(*this)(0, 0) = s__q_RF_HFE_joint__*c__q_RF_KFE_joint__ + s__q_RF_KFE_joint__*c__q_RF_HFE_joint__;
	(*this)(0, 1) = -(-s__q_RF_HFE_joint__*s__q_RF_KFE_joint__ + c__q_RF_HFE_joint__*c__q_RF_KFE_joint__)*s__q_RF_HAA_joint__;
	(*this)(0, 2) = -(-s__q_RF_HFE_joint__*s__q_RF_KFE_joint__ + c__q_RF_HFE_joint__*c__q_RF_KFE_joint__)*c__q_RF_HAA_joint__;
	(*this)(0, 3) = -0.1185*(-s__q_RF_HFE_joint__*s__q_RF_KFE_joint__ + c__q_RF_HFE_joint__*c__q_RF_KFE_joint__)*s__q_RF_HAA_joint__ - 0.37*s__q_RF_HFE_joint__*c__q_RF_KFE_joint__ - 0.37*s__q_RF_KFE_joint__*c__q_RF_HFE_joint__ - 0.38*c__q_RF_KFE_joint__ + 0.3612;
	(*this)(1, 0) = -s__q_RF_HFE_joint__*s__q_RF_KFE_joint__ + c__q_RF_HFE_joint__*c__q_RF_KFE_joint__;
	(*this)(1, 1) = (s__q_RF_HFE_joint__*c__q_RF_KFE_joint__ + s__q_RF_KFE_joint__*c__q_RF_HFE_joint__)*s__q_RF_HAA_joint__;
	(*this)(1, 2) = (s__q_RF_HFE_joint__*c__q_RF_KFE_joint__ + s__q_RF_KFE_joint__*c__q_RF_HFE_joint__)*c__q_RF_HAA_joint__;
	(*this)(1, 3) = 0.1185*(s__q_RF_HFE_joint__*c__q_RF_KFE_joint__ + s__q_RF_KFE_joint__*c__q_RF_HFE_joint__)*s__q_RF_HAA_joint__ + 0.37*s__q_RF_HFE_joint__*s__q_RF_KFE_joint__ + 0.38*s__q_RF_KFE_joint__ - 0.37*c__q_RF_HFE_joint__*c__q_RF_KFE_joint__;
	(*this)(2, 1) = -c__q_RF_HAA_joint__;
	(*this)(2, 2) = s__q_RF_HAA_joint__;
	(*this)(2, 3) = -0.1185*c__q_RF_HAA_joint__ - 0.05;
	return *this;
}

template <typename TRAIT>
iit::pegasus2::tpl::HomogeneousTransforms<TRAIT>::Type_fr_torso_X_fr_RH_foot::Type_fr_torso_X_fr_RH_foot(){ 
	(*this)(0, 2) = 0;
	(*this)(3, 0) = 0;
	(*this)(3, 1) = 0;
	(*this)(3, 2) = 0;
	(*this)(3, 3) = 1;
}

template <typename TRAIT>
const typename iit::pegasus2::tpl::HomogeneousTransforms<TRAIT>::Type_fr_torso_X_fr_RH_foot& 
iit::pegasus2::tpl::HomogeneousTransforms<TRAIT>::Type_fr_torso_X_fr_RH_foot::update(const JState& q){ 
	Scalar c__q_RH_HFE_joint__;
	Scalar c__q_RH_HAA_joint__;
	Scalar c__q_RH_KFE_joint__;
	Scalar s__q_RH_KFE_joint__;
	Scalar s__q_RH_HAA_joint__;
	Scalar s__q_RH_HFE_joint__;

	c__q_RH_HFE_joint__ = TRAIT::cos( q[RH_HFE_JOINT]);
	c__q_RH_HAA_joint__ = TRAIT::cos( q[RH_HAA_JOINT]);
	c__q_RH_KFE_joint__ = TRAIT::cos( q[RH_KFE_JOINT]);
	s__q_RH_KFE_joint__ = TRAIT::sin( q[RH_KFE_JOINT]);
	s__q_RH_HAA_joint__ = TRAIT::sin( q[RH_HAA_JOINT]);
	s__q_RH_HFE_joint__ = TRAIT::sin( q[RH_HFE_JOINT]);

	(*this)(0, 0) = s__q_RH_HFE_joint__*c__q_RH_KFE_joint__ + s__q_RH_KFE_joint__*c__q_RH_HFE_joint__;
	(*this)(0, 1) = -s__q_RH_HFE_joint__*s__q_RH_KFE_joint__ + c__q_RH_HFE_joint__*c__q_RH_KFE_joint__;
	(*this)(0, 3) = 0.3612*s__q_RH_HFE_joint__*c__q_RH_KFE_joint__ + 0.38*s__q_RH_HFE_joint__ + 0.3612*s__q_RH_KFE_joint__*c__q_RH_HFE_joint__ - 0.37;
	(*this)(1, 0) = -(-s__q_RH_HFE_joint__*s__q_RH_KFE_joint__ + c__q_RH_HFE_joint__*c__q_RH_KFE_joint__)*s__q_RH_HAA_joint__;
	(*this)(1, 1) = (s__q_RH_HFE_joint__*c__q_RH_KFE_joint__ + s__q_RH_KFE_joint__*c__q_RH_HFE_joint__)*s__q_RH_HAA_joint__;
	(*this)(1, 2) = -c__q_RH_HAA_joint__;
	(*this)(1, 3) = -0.3612*(-s__q_RH_HFE_joint__*s__q_RH_KFE_joint__ + c__q_RH_HFE_joint__*c__q_RH_KFE_joint__)*s__q_RH_HAA_joint__ - 0.38*s__q_RH_HAA_joint__*c__q_RH_HFE_joint__ - 0.05*c__q_RH_HAA_joint__ - 0.1185;
	(*this)(2, 0) = -(-s__q_RH_HFE_joint__*s__q_RH_KFE_joint__ + c__q_RH_HFE_joint__*c__q_RH_KFE_joint__)*c__q_RH_HAA_joint__;
	(*this)(2, 1) = (s__q_RH_HFE_joint__*c__q_RH_KFE_joint__ + s__q_RH_KFE_joint__*c__q_RH_HFE_joint__)*c__q_RH_HAA_joint__;
	(*this)(2, 2) = s__q_RH_HAA_joint__;
	(*this)(2, 3) = -0.3612*(-s__q_RH_HFE_joint__*s__q_RH_KFE_joint__ + c__q_RH_HFE_joint__*c__q_RH_KFE_joint__)*c__q_RH_HAA_joint__ + 0.05*s__q_RH_HAA_joint__ - 0.38*c__q_RH_HAA_joint__*c__q_RH_HFE_joint__;
	return *this;
}

template <typename TRAIT>
iit::pegasus2::tpl::HomogeneousTransforms<TRAIT>::Type_fr_RH_foot_X_fr_torso::Type_fr_RH_foot_X_fr_torso(){ 
	(*this)(2, 0) = 0;
	(*this)(3, 0) = 0;
	(*this)(3, 1) = 0;
	(*this)(3, 2) = 0;
	(*this)(3, 3) = 1;
}

template <typename TRAIT>
const typename iit::pegasus2::tpl::HomogeneousTransforms<TRAIT>::Type_fr_RH_foot_X_fr_torso& 
iit::pegasus2::tpl::HomogeneousTransforms<TRAIT>::Type_fr_RH_foot_X_fr_torso::update(const JState& q){ 
	Scalar c__q_RH_HFE_joint__;
	Scalar c__q_RH_HAA_joint__;
	Scalar c__q_RH_KFE_joint__;
	Scalar s__q_RH_KFE_joint__;
	Scalar s__q_RH_HAA_joint__;
	Scalar s__q_RH_HFE_joint__;

	c__q_RH_HFE_joint__ = TRAIT::cos( q[RH_HFE_JOINT]);
	c__q_RH_HAA_joint__ = TRAIT::cos( q[RH_HAA_JOINT]);
	c__q_RH_KFE_joint__ = TRAIT::cos( q[RH_KFE_JOINT]);
	s__q_RH_KFE_joint__ = TRAIT::sin( q[RH_KFE_JOINT]);
	s__q_RH_HAA_joint__ = TRAIT::sin( q[RH_HAA_JOINT]);
	s__q_RH_HFE_joint__ = TRAIT::sin( q[RH_HFE_JOINT]);

	(*this)(0, 0) = s__q_RH_HFE_joint__*c__q_RH_KFE_joint__ + s__q_RH_KFE_joint__*c__q_RH_HFE_joint__;
	(*this)(0, 1) = -(-s__q_RH_HFE_joint__*s__q_RH_KFE_joint__ + c__q_RH_HFE_joint__*c__q_RH_KFE_joint__)*s__q_RH_HAA_joint__;
	(*this)(0, 2) = -(-s__q_RH_HFE_joint__*s__q_RH_KFE_joint__ + c__q_RH_HFE_joint__*c__q_RH_KFE_joint__)*c__q_RH_HAA_joint__;
	(*this)(0, 3) = -0.1185*(-s__q_RH_HFE_joint__*s__q_RH_KFE_joint__ + c__q_RH_HFE_joint__*c__q_RH_KFE_joint__)*s__q_RH_HAA_joint__ + 0.37*s__q_RH_HFE_joint__*c__q_RH_KFE_joint__ + 0.37*s__q_RH_KFE_joint__*c__q_RH_HFE_joint__ - 0.38*c__q_RH_KFE_joint__ + 0.3612;
	(*this)(1, 0) = -s__q_RH_HFE_joint__*s__q_RH_KFE_joint__ + c__q_RH_HFE_joint__*c__q_RH_KFE_joint__;
	(*this)(1, 1) = (s__q_RH_HFE_joint__*c__q_RH_KFE_joint__ + s__q_RH_KFE_joint__*c__q_RH_HFE_joint__)*s__q_RH_HAA_joint__;
	(*this)(1, 2) = (s__q_RH_HFE_joint__*c__q_RH_KFE_joint__ + s__q_RH_KFE_joint__*c__q_RH_HFE_joint__)*c__q_RH_HAA_joint__;
	(*this)(1, 3) = 0.1185*(s__q_RH_HFE_joint__*c__q_RH_KFE_joint__ + s__q_RH_KFE_joint__*c__q_RH_HFE_joint__)*s__q_RH_HAA_joint__ - 0.37*s__q_RH_HFE_joint__*s__q_RH_KFE_joint__ + 0.38*s__q_RH_KFE_joint__ + 0.37*c__q_RH_HFE_joint__*c__q_RH_KFE_joint__;
	(*this)(2, 1) = -c__q_RH_HAA_joint__;
	(*this)(2, 2) = s__q_RH_HAA_joint__;
	(*this)(2, 3) = -0.1185*c__q_RH_HAA_joint__ - 0.05;
	return *this;
}

template <typename TRAIT>
iit::pegasus2::tpl::HomogeneousTransforms<TRAIT>::Type_fr_torso_X_fr_LH_foot::Type_fr_torso_X_fr_LH_foot(){ 
	(*this)(0, 2) = 0;
	(*this)(3, 0) = 0;
	(*this)(3, 1) = 0;
	(*this)(3, 2) = 0;
	(*this)(3, 3) = 1;
}

template <typename TRAIT>
const typename iit::pegasus2::tpl::HomogeneousTransforms<TRAIT>::Type_fr_torso_X_fr_LH_foot& 
iit::pegasus2::tpl::HomogeneousTransforms<TRAIT>::Type_fr_torso_X_fr_LH_foot::update(const JState& q){ 
	Scalar c__q_LH_KFE_joint__;
	Scalar c__q_LH_HAA_joint__;
	Scalar c__q_LH_HFE_joint__;
	Scalar s__q_LH_HFE_joint__;
	Scalar s__q_LH_HAA_joint__;
	Scalar s__q_LH_KFE_joint__;

	c__q_LH_KFE_joint__ = TRAIT::cos( q[LH_KFE_JOINT]);
	c__q_LH_HAA_joint__ = TRAIT::cos( q[LH_HAA_JOINT]);
	c__q_LH_HFE_joint__ = TRAIT::cos( q[LH_HFE_JOINT]);
	s__q_LH_HFE_joint__ = TRAIT::sin( q[LH_HFE_JOINT]);
	s__q_LH_HAA_joint__ = TRAIT::sin( q[LH_HAA_JOINT]);
	s__q_LH_KFE_joint__ = TRAIT::sin( q[LH_KFE_JOINT]);

	(*this)(0, 0) = s__q_LH_HFE_joint__*c__q_LH_KFE_joint__ + s__q_LH_KFE_joint__*c__q_LH_HFE_joint__;
	(*this)(0, 1) = -s__q_LH_HFE_joint__*s__q_LH_KFE_joint__ + c__q_LH_HFE_joint__*c__q_LH_KFE_joint__;
	(*this)(0, 3) = 0.3612*s__q_LH_HFE_joint__*c__q_LH_KFE_joint__ + 0.38*s__q_LH_HFE_joint__ + 0.3612*s__q_LH_KFE_joint__*c__q_LH_HFE_joint__ - 0.37;
	(*this)(1, 0) = -(-s__q_LH_HFE_joint__*s__q_LH_KFE_joint__ + c__q_LH_HFE_joint__*c__q_LH_KFE_joint__)*s__q_LH_HAA_joint__;
	(*this)(1, 1) = (s__q_LH_HFE_joint__*c__q_LH_KFE_joint__ + s__q_LH_KFE_joint__*c__q_LH_HFE_joint__)*s__q_LH_HAA_joint__;
	(*this)(1, 2) = -c__q_LH_HAA_joint__;
	(*this)(1, 3) = -0.3612*(-s__q_LH_HFE_joint__*s__q_LH_KFE_joint__ + c__q_LH_HFE_joint__*c__q_LH_KFE_joint__)*s__q_LH_HAA_joint__ - 0.38*s__q_LH_HAA_joint__*c__q_LH_HFE_joint__ + 0.05*c__q_LH_HAA_joint__ + 0.1185;
	(*this)(2, 0) = -(-s__q_LH_HFE_joint__*s__q_LH_KFE_joint__ + c__q_LH_HFE_joint__*c__q_LH_KFE_joint__)*c__q_LH_HAA_joint__;
	(*this)(2, 1) = (s__q_LH_HFE_joint__*c__q_LH_KFE_joint__ + s__q_LH_KFE_joint__*c__q_LH_HFE_joint__)*c__q_LH_HAA_joint__;
	(*this)(2, 2) = s__q_LH_HAA_joint__;
	(*this)(2, 3) = -0.3612*(-s__q_LH_HFE_joint__*s__q_LH_KFE_joint__ + c__q_LH_HFE_joint__*c__q_LH_KFE_joint__)*c__q_LH_HAA_joint__ - 0.05*s__q_LH_HAA_joint__ - 0.38*c__q_LH_HAA_joint__*c__q_LH_HFE_joint__;
	return *this;
}

template <typename TRAIT>
iit::pegasus2::tpl::HomogeneousTransforms<TRAIT>::Type_fr_LH_foot_X_fr_torso::Type_fr_LH_foot_X_fr_torso(){ 
	(*this)(2, 0) = 0;
	(*this)(3, 0) = 0;
	(*this)(3, 1) = 0;
	(*this)(3, 2) = 0;
	(*this)(3, 3) = 1;
}

template <typename TRAIT>
const typename iit::pegasus2::tpl::HomogeneousTransforms<TRAIT>::Type_fr_LH_foot_X_fr_torso& 
iit::pegasus2::tpl::HomogeneousTransforms<TRAIT>::Type_fr_LH_foot_X_fr_torso::update(const JState& q){ 
	Scalar c__q_LH_KFE_joint__;
	Scalar c__q_LH_HAA_joint__;
	Scalar c__q_LH_HFE_joint__;
	Scalar s__q_LH_HFE_joint__;
	Scalar s__q_LH_HAA_joint__;
	Scalar s__q_LH_KFE_joint__;

	c__q_LH_KFE_joint__ = TRAIT::cos( q[LH_KFE_JOINT]);
	c__q_LH_HAA_joint__ = TRAIT::cos( q[LH_HAA_JOINT]);
	c__q_LH_HFE_joint__ = TRAIT::cos( q[LH_HFE_JOINT]);
	s__q_LH_HFE_joint__ = TRAIT::sin( q[LH_HFE_JOINT]);
	s__q_LH_HAA_joint__ = TRAIT::sin( q[LH_HAA_JOINT]);
	s__q_LH_KFE_joint__ = TRAIT::sin( q[LH_KFE_JOINT]);

	(*this)(0, 0) = s__q_LH_HFE_joint__*c__q_LH_KFE_joint__ + s__q_LH_KFE_joint__*c__q_LH_HFE_joint__;
	(*this)(0, 1) = -(-s__q_LH_HFE_joint__*s__q_LH_KFE_joint__ + c__q_LH_HFE_joint__*c__q_LH_KFE_joint__)*s__q_LH_HAA_joint__;
	(*this)(0, 2) = -(-s__q_LH_HFE_joint__*s__q_LH_KFE_joint__ + c__q_LH_HFE_joint__*c__q_LH_KFE_joint__)*c__q_LH_HAA_joint__;
	(*this)(0, 3) = 0.1185*(-s__q_LH_HFE_joint__*s__q_LH_KFE_joint__ + c__q_LH_HFE_joint__*c__q_LH_KFE_joint__)*s__q_LH_HAA_joint__ + 0.37*s__q_LH_HFE_joint__*c__q_LH_KFE_joint__ + 0.37*s__q_LH_KFE_joint__*c__q_LH_HFE_joint__ - 0.38*c__q_LH_KFE_joint__ + 0.3612;
	(*this)(1, 0) = -s__q_LH_HFE_joint__*s__q_LH_KFE_joint__ + c__q_LH_HFE_joint__*c__q_LH_KFE_joint__;
	(*this)(1, 1) = (s__q_LH_HFE_joint__*c__q_LH_KFE_joint__ + s__q_LH_KFE_joint__*c__q_LH_HFE_joint__)*s__q_LH_HAA_joint__;
	(*this)(1, 2) = (s__q_LH_HFE_joint__*c__q_LH_KFE_joint__ + s__q_LH_KFE_joint__*c__q_LH_HFE_joint__)*c__q_LH_HAA_joint__;
	(*this)(1, 3) = -0.1185*(s__q_LH_HFE_joint__*c__q_LH_KFE_joint__ + s__q_LH_KFE_joint__*c__q_LH_HFE_joint__)*s__q_LH_HAA_joint__ - 0.37*s__q_LH_HFE_joint__*s__q_LH_KFE_joint__ + 0.38*s__q_LH_KFE_joint__ + 0.37*c__q_LH_HFE_joint__*c__q_LH_KFE_joint__;
	(*this)(2, 1) = -c__q_LH_HAA_joint__;
	(*this)(2, 2) = s__q_LH_HAA_joint__;
	(*this)(2, 3) = 0.1185*c__q_LH_HAA_joint__ + 0.05;
	return *this;
}

template <typename TRAIT>
iit::pegasus2::tpl::HomogeneousTransforms<TRAIT>::Type_fr_torso_X_fr_LF_foot::Type_fr_torso_X_fr_LF_foot(){ 
	(*this)(0, 2) = 0;
	(*this)(3, 0) = 0;
	(*this)(3, 1) = 0;
	(*this)(3, 2) = 0;
	(*this)(3, 3) = 1;
}

template <typename TRAIT>
const typename iit::pegasus2::tpl::HomogeneousTransforms<TRAIT>::Type_fr_torso_X_fr_LF_foot& 
iit::pegasus2::tpl::HomogeneousTransforms<TRAIT>::Type_fr_torso_X_fr_LF_foot::update(const JState& q){ 
	Scalar c__q_LF_KFE_joint__;
	Scalar c__q_LF_HFE_joint__;
	Scalar c__q_LF_HAA_joint__;
	Scalar s__q_LF_HAA_joint__;
	Scalar s__q_LF_KFE_joint__;
	Scalar s__q_LF_HFE_joint__;

	c__q_LF_KFE_joint__ = TRAIT::cos( q[LF_KFE_JOINT]);
	c__q_LF_HFE_joint__ = TRAIT::cos( q[LF_HFE_JOINT]);
	c__q_LF_HAA_joint__ = TRAIT::cos( q[LF_HAA_JOINT]);
	s__q_LF_HAA_joint__ = TRAIT::sin( q[LF_HAA_JOINT]);
	s__q_LF_KFE_joint__ = TRAIT::sin( q[LF_KFE_JOINT]);
	s__q_LF_HFE_joint__ = TRAIT::sin( q[LF_HFE_JOINT]);

	(*this)(0, 0) = s__q_LF_HFE_joint__*c__q_LF_KFE_joint__ + s__q_LF_KFE_joint__*c__q_LF_HFE_joint__;
	(*this)(0, 1) = -s__q_LF_HFE_joint__*s__q_LF_KFE_joint__ + c__q_LF_HFE_joint__*c__q_LF_KFE_joint__;
	(*this)(0, 3) = 0.3612*s__q_LF_HFE_joint__*c__q_LF_KFE_joint__ + 0.38*s__q_LF_HFE_joint__ + 0.3612*s__q_LF_KFE_joint__*c__q_LF_HFE_joint__ + 0.37;
	(*this)(1, 0) = -(-s__q_LF_HFE_joint__*s__q_LF_KFE_joint__ + c__q_LF_HFE_joint__*c__q_LF_KFE_joint__)*s__q_LF_HAA_joint__;
	(*this)(1, 1) = (s__q_LF_HFE_joint__*c__q_LF_KFE_joint__ + s__q_LF_KFE_joint__*c__q_LF_HFE_joint__)*s__q_LF_HAA_joint__;
	(*this)(1, 2) = -c__q_LF_HAA_joint__;
	(*this)(1, 3) = -0.3612*(-s__q_LF_HFE_joint__*s__q_LF_KFE_joint__ + c__q_LF_HFE_joint__*c__q_LF_KFE_joint__)*s__q_LF_HAA_joint__ - 0.38*s__q_LF_HAA_joint__*c__q_LF_HFE_joint__ + 0.05*c__q_LF_HAA_joint__ + 0.1185;
	(*this)(2, 0) = -(-s__q_LF_HFE_joint__*s__q_LF_KFE_joint__ + c__q_LF_HFE_joint__*c__q_LF_KFE_joint__)*c__q_LF_HAA_joint__;
	(*this)(2, 1) = (s__q_LF_HFE_joint__*c__q_LF_KFE_joint__ + s__q_LF_KFE_joint__*c__q_LF_HFE_joint__)*c__q_LF_HAA_joint__;
	(*this)(2, 2) = s__q_LF_HAA_joint__;
	(*this)(2, 3) = -0.3612*(-s__q_LF_HFE_joint__*s__q_LF_KFE_joint__ + c__q_LF_HFE_joint__*c__q_LF_KFE_joint__)*c__q_LF_HAA_joint__ - 0.05*s__q_LF_HAA_joint__ - 0.38*c__q_LF_HAA_joint__*c__q_LF_HFE_joint__;
	return *this;
}

template <typename TRAIT>
iit::pegasus2::tpl::HomogeneousTransforms<TRAIT>::Type_fr_LF_foot_X_fr_torso::Type_fr_LF_foot_X_fr_torso(){ 
	(*this)(2, 0) = 0;
	(*this)(3, 0) = 0;
	(*this)(3, 1) = 0;
	(*this)(3, 2) = 0;
	(*this)(3, 3) = 1;
}

template <typename TRAIT>
const typename iit::pegasus2::tpl::HomogeneousTransforms<TRAIT>::Type_fr_LF_foot_X_fr_torso& 
iit::pegasus2::tpl::HomogeneousTransforms<TRAIT>::Type_fr_LF_foot_X_fr_torso::update(const JState& q){ 
	Scalar c__q_LF_KFE_joint__;
	Scalar c__q_LF_HFE_joint__;
	Scalar c__q_LF_HAA_joint__;
	Scalar s__q_LF_HAA_joint__;
	Scalar s__q_LF_KFE_joint__;
	Scalar s__q_LF_HFE_joint__;

	c__q_LF_KFE_joint__ = TRAIT::cos( q[LF_KFE_JOINT]);
	c__q_LF_HFE_joint__ = TRAIT::cos( q[LF_HFE_JOINT]);
	c__q_LF_HAA_joint__ = TRAIT::cos( q[LF_HAA_JOINT]);
	s__q_LF_HAA_joint__ = TRAIT::sin( q[LF_HAA_JOINT]);
	s__q_LF_KFE_joint__ = TRAIT::sin( q[LF_KFE_JOINT]);
	s__q_LF_HFE_joint__ = TRAIT::sin( q[LF_HFE_JOINT]);

	(*this)(0, 0) = s__q_LF_HFE_joint__*c__q_LF_KFE_joint__ + s__q_LF_KFE_joint__*c__q_LF_HFE_joint__;
	(*this)(0, 1) = -(-s__q_LF_HFE_joint__*s__q_LF_KFE_joint__ + c__q_LF_HFE_joint__*c__q_LF_KFE_joint__)*s__q_LF_HAA_joint__;
	(*this)(0, 2) = -(-s__q_LF_HFE_joint__*s__q_LF_KFE_joint__ + c__q_LF_HFE_joint__*c__q_LF_KFE_joint__)*c__q_LF_HAA_joint__;
	(*this)(0, 3) = 0.1185*(-s__q_LF_HFE_joint__*s__q_LF_KFE_joint__ + c__q_LF_HFE_joint__*c__q_LF_KFE_joint__)*s__q_LF_HAA_joint__ - 0.37*s__q_LF_HFE_joint__*c__q_LF_KFE_joint__ - 0.37*s__q_LF_KFE_joint__*c__q_LF_HFE_joint__ - 0.38*c__q_LF_KFE_joint__ + 0.3612;
	(*this)(1, 0) = -s__q_LF_HFE_joint__*s__q_LF_KFE_joint__ + c__q_LF_HFE_joint__*c__q_LF_KFE_joint__;
	(*this)(1, 1) = (s__q_LF_HFE_joint__*c__q_LF_KFE_joint__ + s__q_LF_KFE_joint__*c__q_LF_HFE_joint__)*s__q_LF_HAA_joint__;
	(*this)(1, 2) = (s__q_LF_HFE_joint__*c__q_LF_KFE_joint__ + s__q_LF_KFE_joint__*c__q_LF_HFE_joint__)*c__q_LF_HAA_joint__;
	(*this)(1, 3) = -0.1185*(s__q_LF_HFE_joint__*c__q_LF_KFE_joint__ + s__q_LF_KFE_joint__*c__q_LF_HFE_joint__)*s__q_LF_HAA_joint__ + 0.37*s__q_LF_HFE_joint__*s__q_LF_KFE_joint__ + 0.38*s__q_LF_KFE_joint__ - 0.37*c__q_LF_HFE_joint__*c__q_LF_KFE_joint__;
	(*this)(2, 1) = -c__q_LF_HAA_joint__;
	(*this)(2, 2) = s__q_LF_HAA_joint__;
	(*this)(2, 3) = 0.1185*c__q_LF_HAA_joint__ + 0.05;
	return *this;
}

template <typename TRAIT>
iit::pegasus2::tpl::MotionTransforms<TRAIT>::Type_fr_RF_hip_X_fr_RF_upperleg::Type_fr_RF_hip_X_fr_RF_upperleg(){ 
	(*this)(0, 2) = 0;
	(*this)(0, 3) = 0;
	(*this)(0, 4) = 0;
	(*this)(0, 5) = 0;
	(*this)(1, 0) = 0;
	(*this)(1, 1) = 0;
	(*this)(1, 2) = -1;
	(*this)(1, 3) = 0;
	(*this)(1, 4) = 0;
	(*this)(1, 5) = 0;
	(*this)(2, 2) = 0;
	(*this)(2, 3) = 0;
	(*this)(2, 4) = 0;
	(*this)(2, 5) = 0;
	(*this)(3, 2) = 0;
	(*this)(3, 5) = 0;
	(*this)(4, 0) = 0;
	(*this)(4, 1) = 0;
	(*this)(4, 2) = 0;
	(*this)(4, 3) = 0;
	(*this)(4, 4) = 0;
	(*this)(4, 5) = -1;
	(*this)(5, 2) = 0;
	(*this)(5, 5) = 0;
}

template <typename TRAIT>
const typename iit::pegasus2::tpl::MotionTransforms<TRAIT>::Type_fr_RF_hip_X_fr_RF_upperleg& 
iit::pegasus2::tpl::MotionTransforms<TRAIT>::Type_fr_RF_hip_X_fr_RF_upperleg::update(const JState& q){ 
	Scalar c__q_RF_HFE_joint__;
	Scalar s__q_RF_HFE_joint__;

	c__q_RF_HFE_joint__ = TRAIT::cos( q[RF_HFE_JOINT]);
	s__q_RF_HFE_joint__ = TRAIT::sin( q[RF_HFE_JOINT]);

	(*this)(0, 0) = -c__q_RF_HFE_joint__;
	(*this)(0, 1) = s__q_RF_HFE_joint__;
	(*this)(2, 0) = -s__q_RF_HFE_joint__;
	(*this)(2, 1) = -c__q_RF_HFE_joint__;
	(*this)(3, 0) = 0.05*s__q_RF_HFE_joint__;
	(*this)(3, 1) = 0.05*c__q_RF_HFE_joint__;
	(*this)(3, 3) = -c__q_RF_HFE_joint__;
	(*this)(3, 4) = s__q_RF_HFE_joint__;
	(*this)(5, 0) = -0.05*c__q_RF_HFE_joint__;
	(*this)(5, 1) = 0.05*s__q_RF_HFE_joint__;
	(*this)(5, 3) = -s__q_RF_HFE_joint__;
	(*this)(5, 4) = -c__q_RF_HFE_joint__;
	return *this;
}

template <typename TRAIT>
iit::pegasus2::tpl::MotionTransforms<TRAIT>::Type_fr_RF_upperleg_X_fr_RF_hip::Type_fr_RF_upperleg_X_fr_RF_hip(){ 
	(*this)(0, 1) = 0;
	(*this)(0, 3) = 0;
	(*this)(0, 4) = 0;
	(*this)(0, 5) = 0;
	(*this)(1, 1) = 0;
	(*this)(1, 3) = 0;
	(*this)(1, 4) = 0;
	(*this)(1, 5) = 0;
	(*this)(2, 0) = 0;
	(*this)(2, 1) = -1;
	(*this)(2, 2) = 0;
	(*this)(2, 3) = 0;
	(*this)(2, 4) = 0;
	(*this)(2, 5) = 0;
	(*this)(3, 1) = 0;
	(*this)(3, 4) = 0;
	(*this)(4, 1) = 0;
	(*this)(4, 4) = 0;
	(*this)(5, 0) = 0;
	(*this)(5, 1) = 0;
	(*this)(5, 2) = 0;
	(*this)(5, 3) = 0;
	(*this)(5, 4) = -1;
	(*this)(5, 5) = 0;
}

template <typename TRAIT>
const typename iit::pegasus2::tpl::MotionTransforms<TRAIT>::Type_fr_RF_upperleg_X_fr_RF_hip& 
iit::pegasus2::tpl::MotionTransforms<TRAIT>::Type_fr_RF_upperleg_X_fr_RF_hip::update(const JState& q){ 
	Scalar c__q_RF_HFE_joint__;
	Scalar s__q_RF_HFE_joint__;

	c__q_RF_HFE_joint__ = TRAIT::cos( q[RF_HFE_JOINT]);
	s__q_RF_HFE_joint__ = TRAIT::sin( q[RF_HFE_JOINT]);

	(*this)(0, 0) = -c__q_RF_HFE_joint__;
	(*this)(0, 2) = -s__q_RF_HFE_joint__;
	(*this)(1, 0) = s__q_RF_HFE_joint__;
	(*this)(1, 2) = -c__q_RF_HFE_joint__;
	(*this)(3, 0) = 0.05*s__q_RF_HFE_joint__;
	(*this)(3, 2) = -0.05*c__q_RF_HFE_joint__;
	(*this)(3, 3) = -c__q_RF_HFE_joint__;
	(*this)(3, 5) = -s__q_RF_HFE_joint__;
	(*this)(4, 0) = 0.05*c__q_RF_HFE_joint__;
	(*this)(4, 2) = 0.05*s__q_RF_HFE_joint__;
	(*this)(4, 3) = s__q_RF_HFE_joint__;
	(*this)(4, 5) = -c__q_RF_HFE_joint__;
	return *this;
}

template <typename TRAIT>
iit::pegasus2::tpl::MotionTransforms<TRAIT>::Type_fr_RF_upperleg_X_fr_RF_lowerleg::Type_fr_RF_upperleg_X_fr_RF_lowerleg(){ 
	(*this)(0, 2) = 0;
	(*this)(0, 3) = 0;
	(*this)(0, 4) = 0;
	(*this)(0, 5) = 0;
	(*this)(1, 2) = 0;
	(*this)(1, 3) = 0;
	(*this)(1, 4) = 0;
	(*this)(1, 5) = 0;
	(*this)(2, 0) = 0;
	(*this)(2, 1) = 0;
	(*this)(2, 2) = 1;
	(*this)(2, 3) = 0;
	(*this)(2, 4) = 0;
	(*this)(2, 5) = 0;
	(*this)(3, 0) = 0;
	(*this)(3, 1) = 0;
	(*this)(3, 2) = 0;
	(*this)(3, 5) = 0;
	(*this)(4, 0) = 0;
	(*this)(4, 1) = 0;
	(*this)(4, 2) = -0.380000000000000;
	(*this)(4, 5) = 0;
	(*this)(5, 2) = 0;
	(*this)(5, 3) = 0;
	(*this)(5, 4) = 0;
	(*this)(5, 5) = 1;
}

template <typename TRAIT>
const typename iit::pegasus2::tpl::MotionTransforms<TRAIT>::Type_fr_RF_upperleg_X_fr_RF_lowerleg& 
iit::pegasus2::tpl::MotionTransforms<TRAIT>::Type_fr_RF_upperleg_X_fr_RF_lowerleg::update(const JState& q){ 
	Scalar c__q_RF_KFE_joint__;
	Scalar s__q_RF_KFE_joint__;

	c__q_RF_KFE_joint__ = TRAIT::cos( q[RF_KFE_JOINT]);
	s__q_RF_KFE_joint__ = TRAIT::sin( q[RF_KFE_JOINT]);

	(*this)(0, 0) = c__q_RF_KFE_joint__;
	(*this)(0, 1) = -s__q_RF_KFE_joint__;
	(*this)(1, 0) = s__q_RF_KFE_joint__;
	(*this)(1, 1) = c__q_RF_KFE_joint__;
	(*this)(3, 3) = c__q_RF_KFE_joint__;
	(*this)(3, 4) = -s__q_RF_KFE_joint__;
	(*this)(4, 3) = s__q_RF_KFE_joint__;
	(*this)(4, 4) = c__q_RF_KFE_joint__;
	(*this)(5, 0) = 0.38*s__q_RF_KFE_joint__;
	(*this)(5, 1) = 0.38*c__q_RF_KFE_joint__;
	return *this;
}

template <typename TRAIT>
iit::pegasus2::tpl::MotionTransforms<TRAIT>::Type_fr_RF_lowerleg_X_fr_RF_upperleg::Type_fr_RF_lowerleg_X_fr_RF_upperleg(){ 
	(*this)(0, 2) = 0;
	(*this)(0, 3) = 0;
	(*this)(0, 4) = 0;
	(*this)(0, 5) = 0;
	(*this)(1, 2) = 0;
	(*this)(1, 3) = 0;
	(*this)(1, 4) = 0;
	(*this)(1, 5) = 0;
	(*this)(2, 0) = 0;
	(*this)(2, 1) = 0;
	(*this)(2, 2) = 1;
	(*this)(2, 3) = 0;
	(*this)(2, 4) = 0;
	(*this)(2, 5) = 0;
	(*this)(3, 0) = 0;
	(*this)(3, 1) = 0;
	(*this)(3, 5) = 0;
	(*this)(4, 0) = 0;
	(*this)(4, 1) = 0;
	(*this)(4, 5) = 0;
	(*this)(5, 0) = 0;
	(*this)(5, 1) = -0.380000000000000;
	(*this)(5, 2) = 0;
	(*this)(5, 3) = 0;
	(*this)(5, 4) = 0;
	(*this)(5, 5) = 1;
}

template <typename TRAIT>
const typename iit::pegasus2::tpl::MotionTransforms<TRAIT>::Type_fr_RF_lowerleg_X_fr_RF_upperleg& 
iit::pegasus2::tpl::MotionTransforms<TRAIT>::Type_fr_RF_lowerleg_X_fr_RF_upperleg::update(const JState& q){ 
	Scalar c__q_RF_KFE_joint__;
	Scalar s__q_RF_KFE_joint__;

	c__q_RF_KFE_joint__ = TRAIT::cos( q[RF_KFE_JOINT]);
	s__q_RF_KFE_joint__ = TRAIT::sin( q[RF_KFE_JOINT]);

	(*this)(0, 0) = c__q_RF_KFE_joint__;
	(*this)(0, 1) = s__q_RF_KFE_joint__;
	(*this)(1, 0) = -s__q_RF_KFE_joint__;
	(*this)(1, 1) = c__q_RF_KFE_joint__;
	(*this)(3, 2) = 0.38*s__q_RF_KFE_joint__;
	(*this)(3, 3) = c__q_RF_KFE_joint__;
	(*this)(3, 4) = s__q_RF_KFE_joint__;
	(*this)(4, 2) = 0.38*c__q_RF_KFE_joint__;
	(*this)(4, 3) = -s__q_RF_KFE_joint__;
	(*this)(4, 4) = c__q_RF_KFE_joint__;
	return *this;
}

template <typename TRAIT>
iit::pegasus2::tpl::MotionTransforms<TRAIT>::Type_fr_RH_hip_X_fr_RH_upperleg::Type_fr_RH_hip_X_fr_RH_upperleg(){ 
	(*this)(0, 2) = 0;
	(*this)(0, 3) = 0;
	(*this)(0, 4) = 0;
	(*this)(0, 5) = 0;
	(*this)(1, 0) = 0;
	(*this)(1, 1) = 0;
	(*this)(1, 2) = -1;
	(*this)(1, 3) = 0;
	(*this)(1, 4) = 0;
	(*this)(1, 5) = 0;
	(*this)(2, 2) = 0;
	(*this)(2, 3) = 0;
	(*this)(2, 4) = 0;
	(*this)(2, 5) = 0;
	(*this)(3, 2) = 0;
	(*this)(3, 5) = 0;
	(*this)(4, 0) = 0;
	(*this)(4, 1) = 0;
	(*this)(4, 2) = 0;
	(*this)(4, 3) = 0;
	(*this)(4, 4) = 0;
	(*this)(4, 5) = -1;
	(*this)(5, 2) = 0;
	(*this)(5, 5) = 0;
}

template <typename TRAIT>
const typename iit::pegasus2::tpl::MotionTransforms<TRAIT>::Type_fr_RH_hip_X_fr_RH_upperleg& 
iit::pegasus2::tpl::MotionTransforms<TRAIT>::Type_fr_RH_hip_X_fr_RH_upperleg::update(const JState& q){ 
	Scalar c__q_RH_HFE_joint__;
	Scalar s__q_RH_HFE_joint__;

	c__q_RH_HFE_joint__ = TRAIT::cos( q[RH_HFE_JOINT]);
	s__q_RH_HFE_joint__ = TRAIT::sin( q[RH_HFE_JOINT]);

	(*this)(0, 0) = -c__q_RH_HFE_joint__;
	(*this)(0, 1) = s__q_RH_HFE_joint__;
	(*this)(2, 0) = -s__q_RH_HFE_joint__;
	(*this)(2, 1) = -c__q_RH_HFE_joint__;
	(*this)(3, 0) = 0.05*s__q_RH_HFE_joint__;
	(*this)(3, 1) = 0.05*c__q_RH_HFE_joint__;
	(*this)(3, 3) = -c__q_RH_HFE_joint__;
	(*this)(3, 4) = s__q_RH_HFE_joint__;
	(*this)(5, 0) = -0.05*c__q_RH_HFE_joint__;
	(*this)(5, 1) = 0.05*s__q_RH_HFE_joint__;
	(*this)(5, 3) = -s__q_RH_HFE_joint__;
	(*this)(5, 4) = -c__q_RH_HFE_joint__;
	return *this;
}

template <typename TRAIT>
iit::pegasus2::tpl::MotionTransforms<TRAIT>::Type_fr_RH_upperleg_X_fr_RH_hip::Type_fr_RH_upperleg_X_fr_RH_hip(){ 
	(*this)(0, 1) = 0;
	(*this)(0, 3) = 0;
	(*this)(0, 4) = 0;
	(*this)(0, 5) = 0;
	(*this)(1, 1) = 0;
	(*this)(1, 3) = 0;
	(*this)(1, 4) = 0;
	(*this)(1, 5) = 0;
	(*this)(2, 0) = 0;
	(*this)(2, 1) = -1;
	(*this)(2, 2) = 0;
	(*this)(2, 3) = 0;
	(*this)(2, 4) = 0;
	(*this)(2, 5) = 0;
	(*this)(3, 1) = 0;
	(*this)(3, 4) = 0;
	(*this)(4, 1) = 0;
	(*this)(4, 4) = 0;
	(*this)(5, 0) = 0;
	(*this)(5, 1) = 0;
	(*this)(5, 2) = 0;
	(*this)(5, 3) = 0;
	(*this)(5, 4) = -1;
	(*this)(5, 5) = 0;
}

template <typename TRAIT>
const typename iit::pegasus2::tpl::MotionTransforms<TRAIT>::Type_fr_RH_upperleg_X_fr_RH_hip& 
iit::pegasus2::tpl::MotionTransforms<TRAIT>::Type_fr_RH_upperleg_X_fr_RH_hip::update(const JState& q){ 
	Scalar c__q_RH_HFE_joint__;
	Scalar s__q_RH_HFE_joint__;

	c__q_RH_HFE_joint__ = TRAIT::cos( q[RH_HFE_JOINT]);
	s__q_RH_HFE_joint__ = TRAIT::sin( q[RH_HFE_JOINT]);

	(*this)(0, 0) = -c__q_RH_HFE_joint__;
	(*this)(0, 2) = -s__q_RH_HFE_joint__;
	(*this)(1, 0) = s__q_RH_HFE_joint__;
	(*this)(1, 2) = -c__q_RH_HFE_joint__;
	(*this)(3, 0) = 0.05*s__q_RH_HFE_joint__;
	(*this)(3, 2) = -0.05*c__q_RH_HFE_joint__;
	(*this)(3, 3) = -c__q_RH_HFE_joint__;
	(*this)(3, 5) = -s__q_RH_HFE_joint__;
	(*this)(4, 0) = 0.05*c__q_RH_HFE_joint__;
	(*this)(4, 2) = 0.05*s__q_RH_HFE_joint__;
	(*this)(4, 3) = s__q_RH_HFE_joint__;
	(*this)(4, 5) = -c__q_RH_HFE_joint__;
	return *this;
}

template <typename TRAIT>
iit::pegasus2::tpl::MotionTransforms<TRAIT>::Type_fr_RH_upperleg_X_fr_RH_lowerleg::Type_fr_RH_upperleg_X_fr_RH_lowerleg(){ 
	(*this)(0, 2) = 0;
	(*this)(0, 3) = 0;
	(*this)(0, 4) = 0;
	(*this)(0, 5) = 0;
	(*this)(1, 2) = 0;
	(*this)(1, 3) = 0;
	(*this)(1, 4) = 0;
	(*this)(1, 5) = 0;
	(*this)(2, 0) = 0;
	(*this)(2, 1) = 0;
	(*this)(2, 2) = 1;
	(*this)(2, 3) = 0;
	(*this)(2, 4) = 0;
	(*this)(2, 5) = 0;
	(*this)(3, 0) = 0;
	(*this)(3, 1) = 0;
	(*this)(3, 2) = 0;
	(*this)(3, 5) = 0;
	(*this)(4, 0) = 0;
	(*this)(4, 1) = 0;
	(*this)(4, 2) = -0.380000000000000;
	(*this)(4, 5) = 0;
	(*this)(5, 2) = 0;
	(*this)(5, 3) = 0;
	(*this)(5, 4) = 0;
	(*this)(5, 5) = 1;
}

template <typename TRAIT>
const typename iit::pegasus2::tpl::MotionTransforms<TRAIT>::Type_fr_RH_upperleg_X_fr_RH_lowerleg& 
iit::pegasus2::tpl::MotionTransforms<TRAIT>::Type_fr_RH_upperleg_X_fr_RH_lowerleg::update(const JState& q){ 
	Scalar c__q_RH_KFE_joint__;
	Scalar s__q_RH_KFE_joint__;

	c__q_RH_KFE_joint__ = TRAIT::cos( q[RH_KFE_JOINT]);
	s__q_RH_KFE_joint__ = TRAIT::sin( q[RH_KFE_JOINT]);

	(*this)(0, 0) = c__q_RH_KFE_joint__;
	(*this)(0, 1) = -s__q_RH_KFE_joint__;
	(*this)(1, 0) = s__q_RH_KFE_joint__;
	(*this)(1, 1) = c__q_RH_KFE_joint__;
	(*this)(3, 3) = c__q_RH_KFE_joint__;
	(*this)(3, 4) = -s__q_RH_KFE_joint__;
	(*this)(4, 3) = s__q_RH_KFE_joint__;
	(*this)(4, 4) = c__q_RH_KFE_joint__;
	(*this)(5, 0) = 0.38*s__q_RH_KFE_joint__;
	(*this)(5, 1) = 0.38*c__q_RH_KFE_joint__;
	return *this;
}

template <typename TRAIT>
iit::pegasus2::tpl::MotionTransforms<TRAIT>::Type_fr_RH_lowerleg_X_fr_RH_upperleg::Type_fr_RH_lowerleg_X_fr_RH_upperleg(){ 
	(*this)(0, 2) = 0;
	(*this)(0, 3) = 0;
	(*this)(0, 4) = 0;
	(*this)(0, 5) = 0;
	(*this)(1, 2) = 0;
	(*this)(1, 3) = 0;
	(*this)(1, 4) = 0;
	(*this)(1, 5) = 0;
	(*this)(2, 0) = 0;
	(*this)(2, 1) = 0;
	(*this)(2, 2) = 1;
	(*this)(2, 3) = 0;
	(*this)(2, 4) = 0;
	(*this)(2, 5) = 0;
	(*this)(3, 0) = 0;
	(*this)(3, 1) = 0;
	(*this)(3, 5) = 0;
	(*this)(4, 0) = 0;
	(*this)(4, 1) = 0;
	(*this)(4, 5) = 0;
	(*this)(5, 0) = 0;
	(*this)(5, 1) = -0.380000000000000;
	(*this)(5, 2) = 0;
	(*this)(5, 3) = 0;
	(*this)(5, 4) = 0;
	(*this)(5, 5) = 1;
}

template <typename TRAIT>
const typename iit::pegasus2::tpl::MotionTransforms<TRAIT>::Type_fr_RH_lowerleg_X_fr_RH_upperleg& 
iit::pegasus2::tpl::MotionTransforms<TRAIT>::Type_fr_RH_lowerleg_X_fr_RH_upperleg::update(const JState& q){ 
	Scalar c__q_RH_KFE_joint__;
	Scalar s__q_RH_KFE_joint__;

	c__q_RH_KFE_joint__ = TRAIT::cos( q[RH_KFE_JOINT]);
	s__q_RH_KFE_joint__ = TRAIT::sin( q[RH_KFE_JOINT]);

	(*this)(0, 0) = c__q_RH_KFE_joint__;
	(*this)(0, 1) = s__q_RH_KFE_joint__;
	(*this)(1, 0) = -s__q_RH_KFE_joint__;
	(*this)(1, 1) = c__q_RH_KFE_joint__;
	(*this)(3, 2) = 0.38*s__q_RH_KFE_joint__;
	(*this)(3, 3) = c__q_RH_KFE_joint__;
	(*this)(3, 4) = s__q_RH_KFE_joint__;
	(*this)(4, 2) = 0.38*c__q_RH_KFE_joint__;
	(*this)(4, 3) = -s__q_RH_KFE_joint__;
	(*this)(4, 4) = c__q_RH_KFE_joint__;
	return *this;
}

template <typename TRAIT>
iit::pegasus2::tpl::MotionTransforms<TRAIT>::Type_fr_LH_hip_X_fr_LH_upperleg::Type_fr_LH_hip_X_fr_LH_upperleg(){ 
	(*this)(0, 2) = 0;
	(*this)(0, 3) = 0;
	(*this)(0, 4) = 0;
	(*this)(0, 5) = 0;
	(*this)(1, 0) = 0;
	(*this)(1, 1) = 0;
	(*this)(1, 2) = -1;
	(*this)(1, 3) = 0;
	(*this)(1, 4) = 0;
	(*this)(1, 5) = 0;
	(*this)(2, 2) = 0;
	(*this)(2, 3) = 0;
	(*this)(2, 4) = 0;
	(*this)(2, 5) = 0;
	(*this)(3, 2) = 0;
	(*this)(3, 5) = 0;
	(*this)(4, 0) = 0;
	(*this)(4, 1) = 0;
	(*this)(4, 2) = 0;
	(*this)(4, 3) = 0;
	(*this)(4, 4) = 0;
	(*this)(4, 5) = -1;
	(*this)(5, 2) = 0;
	(*this)(5, 5) = 0;
}

template <typename TRAIT>
const typename iit::pegasus2::tpl::MotionTransforms<TRAIT>::Type_fr_LH_hip_X_fr_LH_upperleg& 
iit::pegasus2::tpl::MotionTransforms<TRAIT>::Type_fr_LH_hip_X_fr_LH_upperleg::update(const JState& q){ 
	Scalar c__q_LH_HFE_joint__;
	Scalar s__q_LH_HFE_joint__;

	c__q_LH_HFE_joint__ = TRAIT::cos( q[LH_HFE_JOINT]);
	s__q_LH_HFE_joint__ = TRAIT::sin( q[LH_HFE_JOINT]);

	(*this)(0, 0) = -c__q_LH_HFE_joint__;
	(*this)(0, 1) = s__q_LH_HFE_joint__;
	(*this)(2, 0) = -s__q_LH_HFE_joint__;
	(*this)(2, 1) = -c__q_LH_HFE_joint__;
	(*this)(3, 0) = -0.05*s__q_LH_HFE_joint__;
	(*this)(3, 1) = -0.05*c__q_LH_HFE_joint__;
	(*this)(3, 3) = -c__q_LH_HFE_joint__;
	(*this)(3, 4) = s__q_LH_HFE_joint__;
	(*this)(5, 0) = 0.05*c__q_LH_HFE_joint__;
	(*this)(5, 1) = -0.05*s__q_LH_HFE_joint__;
	(*this)(5, 3) = -s__q_LH_HFE_joint__;
	(*this)(5, 4) = -c__q_LH_HFE_joint__;
	return *this;
}

template <typename TRAIT>
iit::pegasus2::tpl::MotionTransforms<TRAIT>::Type_fr_LH_upperleg_X_fr_LH_hip::Type_fr_LH_upperleg_X_fr_LH_hip(){ 
	(*this)(0, 1) = 0;
	(*this)(0, 3) = 0;
	(*this)(0, 4) = 0;
	(*this)(0, 5) = 0;
	(*this)(1, 1) = 0;
	(*this)(1, 3) = 0;
	(*this)(1, 4) = 0;
	(*this)(1, 5) = 0;
	(*this)(2, 0) = 0;
	(*this)(2, 1) = -1;
	(*this)(2, 2) = 0;
	(*this)(2, 3) = 0;
	(*this)(2, 4) = 0;
	(*this)(2, 5) = 0;
	(*this)(3, 1) = 0;
	(*this)(3, 4) = 0;
	(*this)(4, 1) = 0;
	(*this)(4, 4) = 0;
	(*this)(5, 0) = 0;
	(*this)(5, 1) = 0;
	(*this)(5, 2) = 0;
	(*this)(5, 3) = 0;
	(*this)(5, 4) = -1;
	(*this)(5, 5) = 0;
}

template <typename TRAIT>
const typename iit::pegasus2::tpl::MotionTransforms<TRAIT>::Type_fr_LH_upperleg_X_fr_LH_hip& 
iit::pegasus2::tpl::MotionTransforms<TRAIT>::Type_fr_LH_upperleg_X_fr_LH_hip::update(const JState& q){ 
	Scalar c__q_LH_HFE_joint__;
	Scalar s__q_LH_HFE_joint__;

	c__q_LH_HFE_joint__ = TRAIT::cos( q[LH_HFE_JOINT]);
	s__q_LH_HFE_joint__ = TRAIT::sin( q[LH_HFE_JOINT]);

	(*this)(0, 0) = -c__q_LH_HFE_joint__;
	(*this)(0, 2) = -s__q_LH_HFE_joint__;
	(*this)(1, 0) = s__q_LH_HFE_joint__;
	(*this)(1, 2) = -c__q_LH_HFE_joint__;
	(*this)(3, 0) = -0.05*s__q_LH_HFE_joint__;
	(*this)(3, 2) = 0.05*c__q_LH_HFE_joint__;
	(*this)(3, 3) = -c__q_LH_HFE_joint__;
	(*this)(3, 5) = -s__q_LH_HFE_joint__;
	(*this)(4, 0) = -0.05*c__q_LH_HFE_joint__;
	(*this)(4, 2) = -0.05*s__q_LH_HFE_joint__;
	(*this)(4, 3) = s__q_LH_HFE_joint__;
	(*this)(4, 5) = -c__q_LH_HFE_joint__;
	return *this;
}

template <typename TRAIT>
iit::pegasus2::tpl::MotionTransforms<TRAIT>::Type_fr_LH_upperleg_X_fr_LH_lowerleg::Type_fr_LH_upperleg_X_fr_LH_lowerleg(){ 
	(*this)(0, 2) = 0;
	(*this)(0, 3) = 0;
	(*this)(0, 4) = 0;
	(*this)(0, 5) = 0;
	(*this)(1, 2) = 0;
	(*this)(1, 3) = 0;
	(*this)(1, 4) = 0;
	(*this)(1, 5) = 0;
	(*this)(2, 0) = 0;
	(*this)(2, 1) = 0;
	(*this)(2, 2) = 1;
	(*this)(2, 3) = 0;
	(*this)(2, 4) = 0;
	(*this)(2, 5) = 0;
	(*this)(3, 0) = 0;
	(*this)(3, 1) = 0;
	(*this)(3, 2) = 0;
	(*this)(3, 5) = 0;
	(*this)(4, 0) = 0;
	(*this)(4, 1) = 0;
	(*this)(4, 2) = -0.380000000000000;
	(*this)(4, 5) = 0;
	(*this)(5, 2) = 0;
	(*this)(5, 3) = 0;
	(*this)(5, 4) = 0;
	(*this)(5, 5) = 1;
}

template <typename TRAIT>
const typename iit::pegasus2::tpl::MotionTransforms<TRAIT>::Type_fr_LH_upperleg_X_fr_LH_lowerleg& 
iit::pegasus2::tpl::MotionTransforms<TRAIT>::Type_fr_LH_upperleg_X_fr_LH_lowerleg::update(const JState& q){ 
	Scalar c__q_LH_KFE_joint__;
	Scalar s__q_LH_KFE_joint__;

	c__q_LH_KFE_joint__ = TRAIT::cos( q[LH_KFE_JOINT]);
	s__q_LH_KFE_joint__ = TRAIT::sin( q[LH_KFE_JOINT]);

	(*this)(0, 0) = c__q_LH_KFE_joint__;
	(*this)(0, 1) = -s__q_LH_KFE_joint__;
	(*this)(1, 0) = s__q_LH_KFE_joint__;
	(*this)(1, 1) = c__q_LH_KFE_joint__;
	(*this)(3, 3) = c__q_LH_KFE_joint__;
	(*this)(3, 4) = -s__q_LH_KFE_joint__;
	(*this)(4, 3) = s__q_LH_KFE_joint__;
	(*this)(4, 4) = c__q_LH_KFE_joint__;
	(*this)(5, 0) = 0.38*s__q_LH_KFE_joint__;
	(*this)(5, 1) = 0.38*c__q_LH_KFE_joint__;
	return *this;
}

template <typename TRAIT>
iit::pegasus2::tpl::MotionTransforms<TRAIT>::Type_fr_LH_lowerleg_X_fr_LH_upperleg::Type_fr_LH_lowerleg_X_fr_LH_upperleg(){ 
	(*this)(0, 2) = 0;
	(*this)(0, 3) = 0;
	(*this)(0, 4) = 0;
	(*this)(0, 5) = 0;
	(*this)(1, 2) = 0;
	(*this)(1, 3) = 0;
	(*this)(1, 4) = 0;
	(*this)(1, 5) = 0;
	(*this)(2, 0) = 0;
	(*this)(2, 1) = 0;
	(*this)(2, 2) = 1;
	(*this)(2, 3) = 0;
	(*this)(2, 4) = 0;
	(*this)(2, 5) = 0;
	(*this)(3, 0) = 0;
	(*this)(3, 1) = 0;
	(*this)(3, 5) = 0;
	(*this)(4, 0) = 0;
	(*this)(4, 1) = 0;
	(*this)(4, 5) = 0;
	(*this)(5, 0) = 0;
	(*this)(5, 1) = -0.380000000000000;
	(*this)(5, 2) = 0;
	(*this)(5, 3) = 0;
	(*this)(5, 4) = 0;
	(*this)(5, 5) = 1;
}

template <typename TRAIT>
const typename iit::pegasus2::tpl::MotionTransforms<TRAIT>::Type_fr_LH_lowerleg_X_fr_LH_upperleg& 
iit::pegasus2::tpl::MotionTransforms<TRAIT>::Type_fr_LH_lowerleg_X_fr_LH_upperleg::update(const JState& q){ 
	Scalar c__q_LH_KFE_joint__;
	Scalar s__q_LH_KFE_joint__;

	c__q_LH_KFE_joint__ = TRAIT::cos( q[LH_KFE_JOINT]);
	s__q_LH_KFE_joint__ = TRAIT::sin( q[LH_KFE_JOINT]);

	(*this)(0, 0) = c__q_LH_KFE_joint__;
	(*this)(0, 1) = s__q_LH_KFE_joint__;
	(*this)(1, 0) = -s__q_LH_KFE_joint__;
	(*this)(1, 1) = c__q_LH_KFE_joint__;
	(*this)(3, 2) = 0.38*s__q_LH_KFE_joint__;
	(*this)(3, 3) = c__q_LH_KFE_joint__;
	(*this)(3, 4) = s__q_LH_KFE_joint__;
	(*this)(4, 2) = 0.38*c__q_LH_KFE_joint__;
	(*this)(4, 3) = -s__q_LH_KFE_joint__;
	(*this)(4, 4) = c__q_LH_KFE_joint__;
	return *this;
}

template <typename TRAIT>
iit::pegasus2::tpl::MotionTransforms<TRAIT>::Type_fr_LF_hip_X_fr_LF_upperleg::Type_fr_LF_hip_X_fr_LF_upperleg(){ 
	(*this)(0, 2) = 0;
	(*this)(0, 3) = 0;
	(*this)(0, 4) = 0;
	(*this)(0, 5) = 0;
	(*this)(1, 0) = 0;
	(*this)(1, 1) = 0;
	(*this)(1, 2) = -1;
	(*this)(1, 3) = 0;
	(*this)(1, 4) = 0;
	(*this)(1, 5) = 0;
	(*this)(2, 2) = 0;
	(*this)(2, 3) = 0;
	(*this)(2, 4) = 0;
	(*this)(2, 5) = 0;
	(*this)(3, 2) = 0;
	(*this)(3, 5) = 0;
	(*this)(4, 0) = 0;
	(*this)(4, 1) = 0;
	(*this)(4, 2) = 0;
	(*this)(4, 3) = 0;
	(*this)(4, 4) = 0;
	(*this)(4, 5) = -1;
	(*this)(5, 2) = 0;
	(*this)(5, 5) = 0;
}

template <typename TRAIT>
const typename iit::pegasus2::tpl::MotionTransforms<TRAIT>::Type_fr_LF_hip_X_fr_LF_upperleg& 
iit::pegasus2::tpl::MotionTransforms<TRAIT>::Type_fr_LF_hip_X_fr_LF_upperleg::update(const JState& q){ 
	Scalar c__q_LF_HFE_joint__;
	Scalar s__q_LF_HFE_joint__;

	c__q_LF_HFE_joint__ = TRAIT::cos( q[LF_HFE_JOINT]);
	s__q_LF_HFE_joint__ = TRAIT::sin( q[LF_HFE_JOINT]);

	(*this)(0, 0) = -c__q_LF_HFE_joint__;
	(*this)(0, 1) = s__q_LF_HFE_joint__;
	(*this)(2, 0) = -s__q_LF_HFE_joint__;
	(*this)(2, 1) = -c__q_LF_HFE_joint__;
	(*this)(3, 0) = -0.05*s__q_LF_HFE_joint__;
	(*this)(3, 1) = -0.05*c__q_LF_HFE_joint__;
	(*this)(3, 3) = -c__q_LF_HFE_joint__;
	(*this)(3, 4) = s__q_LF_HFE_joint__;
	(*this)(5, 0) = 0.05*c__q_LF_HFE_joint__;
	(*this)(5, 1) = -0.05*s__q_LF_HFE_joint__;
	(*this)(5, 3) = -s__q_LF_HFE_joint__;
	(*this)(5, 4) = -c__q_LF_HFE_joint__;
	return *this;
}

template <typename TRAIT>
iit::pegasus2::tpl::MotionTransforms<TRAIT>::Type_fr_LF_upperleg_X_fr_LF_hip::Type_fr_LF_upperleg_X_fr_LF_hip(){ 
	(*this)(0, 1) = 0;
	(*this)(0, 3) = 0;
	(*this)(0, 4) = 0;
	(*this)(0, 5) = 0;
	(*this)(1, 1) = 0;
	(*this)(1, 3) = 0;
	(*this)(1, 4) = 0;
	(*this)(1, 5) = 0;
	(*this)(2, 0) = 0;
	(*this)(2, 1) = -1;
	(*this)(2, 2) = 0;
	(*this)(2, 3) = 0;
	(*this)(2, 4) = 0;
	(*this)(2, 5) = 0;
	(*this)(3, 1) = 0;
	(*this)(3, 4) = 0;
	(*this)(4, 1) = 0;
	(*this)(4, 4) = 0;
	(*this)(5, 0) = 0;
	(*this)(5, 1) = 0;
	(*this)(5, 2) = 0;
	(*this)(5, 3) = 0;
	(*this)(5, 4) = -1;
	(*this)(5, 5) = 0;
}

template <typename TRAIT>
const typename iit::pegasus2::tpl::MotionTransforms<TRAIT>::Type_fr_LF_upperleg_X_fr_LF_hip& 
iit::pegasus2::tpl::MotionTransforms<TRAIT>::Type_fr_LF_upperleg_X_fr_LF_hip::update(const JState& q){ 
	Scalar c__q_LF_HFE_joint__;
	Scalar s__q_LF_HFE_joint__;

	c__q_LF_HFE_joint__ = TRAIT::cos( q[LF_HFE_JOINT]);
	s__q_LF_HFE_joint__ = TRAIT::sin( q[LF_HFE_JOINT]);

	(*this)(0, 0) = -c__q_LF_HFE_joint__;
	(*this)(0, 2) = -s__q_LF_HFE_joint__;
	(*this)(1, 0) = s__q_LF_HFE_joint__;
	(*this)(1, 2) = -c__q_LF_HFE_joint__;
	(*this)(3, 0) = -0.05*s__q_LF_HFE_joint__;
	(*this)(3, 2) = 0.05*c__q_LF_HFE_joint__;
	(*this)(3, 3) = -c__q_LF_HFE_joint__;
	(*this)(3, 5) = -s__q_LF_HFE_joint__;
	(*this)(4, 0) = -0.05*c__q_LF_HFE_joint__;
	(*this)(4, 2) = -0.05*s__q_LF_HFE_joint__;
	(*this)(4, 3) = s__q_LF_HFE_joint__;
	(*this)(4, 5) = -c__q_LF_HFE_joint__;
	return *this;
}

template <typename TRAIT>
iit::pegasus2::tpl::MotionTransforms<TRAIT>::Type_fr_LF_upperleg_X_fr_LF_lowerleg::Type_fr_LF_upperleg_X_fr_LF_lowerleg(){ 
	(*this)(0, 2) = 0;
	(*this)(0, 3) = 0;
	(*this)(0, 4) = 0;
	(*this)(0, 5) = 0;
	(*this)(1, 2) = 0;
	(*this)(1, 3) = 0;
	(*this)(1, 4) = 0;
	(*this)(1, 5) = 0;
	(*this)(2, 0) = 0;
	(*this)(2, 1) = 0;
	(*this)(2, 2) = 1;
	(*this)(2, 3) = 0;
	(*this)(2, 4) = 0;
	(*this)(2, 5) = 0;
	(*this)(3, 0) = 0;
	(*this)(3, 1) = 0;
	(*this)(3, 2) = 0;
	(*this)(3, 5) = 0;
	(*this)(4, 0) = 0;
	(*this)(4, 1) = 0;
	(*this)(4, 2) = -0.380000000000000;
	(*this)(4, 5) = 0;
	(*this)(5, 2) = 0;
	(*this)(5, 3) = 0;
	(*this)(5, 4) = 0;
	(*this)(5, 5) = 1;
}

template <typename TRAIT>
const typename iit::pegasus2::tpl::MotionTransforms<TRAIT>::Type_fr_LF_upperleg_X_fr_LF_lowerleg& 
iit::pegasus2::tpl::MotionTransforms<TRAIT>::Type_fr_LF_upperleg_X_fr_LF_lowerleg::update(const JState& q){ 
	Scalar c__q_LF_KFE_joint__;
	Scalar s__q_LF_KFE_joint__;

	c__q_LF_KFE_joint__ = TRAIT::cos( q[LF_KFE_JOINT]);
	s__q_LF_KFE_joint__ = TRAIT::sin( q[LF_KFE_JOINT]);

	(*this)(0, 0) = c__q_LF_KFE_joint__;
	(*this)(0, 1) = -s__q_LF_KFE_joint__;
	(*this)(1, 0) = s__q_LF_KFE_joint__;
	(*this)(1, 1) = c__q_LF_KFE_joint__;
	(*this)(3, 3) = c__q_LF_KFE_joint__;
	(*this)(3, 4) = -s__q_LF_KFE_joint__;
	(*this)(4, 3) = s__q_LF_KFE_joint__;
	(*this)(4, 4) = c__q_LF_KFE_joint__;
	(*this)(5, 0) = 0.38*s__q_LF_KFE_joint__;
	(*this)(5, 1) = 0.38*c__q_LF_KFE_joint__;
	return *this;
}

template <typename TRAIT>
iit::pegasus2::tpl::MotionTransforms<TRAIT>::Type_fr_LF_lowerleg_X_fr_LF_upperleg::Type_fr_LF_lowerleg_X_fr_LF_upperleg(){ 
	(*this)(0, 2) = 0;
	(*this)(0, 3) = 0;
	(*this)(0, 4) = 0;
	(*this)(0, 5) = 0;
	(*this)(1, 2) = 0;
	(*this)(1, 3) = 0;
	(*this)(1, 4) = 0;
	(*this)(1, 5) = 0;
	(*this)(2, 0) = 0;
	(*this)(2, 1) = 0;
	(*this)(2, 2) = 1;
	(*this)(2, 3) = 0;
	(*this)(2, 4) = 0;
	(*this)(2, 5) = 0;
	(*this)(3, 0) = 0;
	(*this)(3, 1) = 0;
	(*this)(3, 5) = 0;
	(*this)(4, 0) = 0;
	(*this)(4, 1) = 0;
	(*this)(4, 5) = 0;
	(*this)(5, 0) = 0;
	(*this)(5, 1) = -0.380000000000000;
	(*this)(5, 2) = 0;
	(*this)(5, 3) = 0;
	(*this)(5, 4) = 0;
	(*this)(5, 5) = 1;
}

template <typename TRAIT>
const typename iit::pegasus2::tpl::MotionTransforms<TRAIT>::Type_fr_LF_lowerleg_X_fr_LF_upperleg& 
iit::pegasus2::tpl::MotionTransforms<TRAIT>::Type_fr_LF_lowerleg_X_fr_LF_upperleg::update(const JState& q){ 
	Scalar c__q_LF_KFE_joint__;
	Scalar s__q_LF_KFE_joint__;

	c__q_LF_KFE_joint__ = TRAIT::cos( q[LF_KFE_JOINT]);
	s__q_LF_KFE_joint__ = TRAIT::sin( q[LF_KFE_JOINT]);

	(*this)(0, 0) = c__q_LF_KFE_joint__;
	(*this)(0, 1) = s__q_LF_KFE_joint__;
	(*this)(1, 0) = -s__q_LF_KFE_joint__;
	(*this)(1, 1) = c__q_LF_KFE_joint__;
	(*this)(3, 2) = 0.38*s__q_LF_KFE_joint__;
	(*this)(3, 3) = c__q_LF_KFE_joint__;
	(*this)(3, 4) = s__q_LF_KFE_joint__;
	(*this)(4, 2) = 0.38*c__q_LF_KFE_joint__;
	(*this)(4, 3) = -s__q_LF_KFE_joint__;
	(*this)(4, 4) = c__q_LF_KFE_joint__;
	return *this;
}

template <typename TRAIT>
iit::pegasus2::tpl::MotionTransforms<TRAIT>::Type_fr_torso_X_fr_RF_hip::Type_fr_torso_X_fr_RF_hip(){ 
	(*this)(0, 0) = 0;
	(*this)(0, 1) = 0;
	(*this)(0, 2) = -1;
	(*this)(0, 3) = 0;
	(*this)(0, 4) = 0;
	(*this)(0, 5) = 0;
	(*this)(1, 2) = 0;
	(*this)(1, 3) = 0;
	(*this)(1, 4) = 0;
	(*this)(1, 5) = 0;
	(*this)(2, 2) = 0;
	(*this)(2, 3) = 0;
	(*this)(2, 4) = 0;
	(*this)(2, 5) = 0;
	(*this)(3, 2) = 0;
	(*this)(3, 3) = 0;
	(*this)(3, 4) = 0;
	(*this)(3, 5) = -1;
	(*this)(4, 2) = 0;
	(*this)(4, 5) = 0;
	(*this)(5, 2) = -0.118500000000000;
	(*this)(5, 5) = 0;
}

template <typename TRAIT>
const typename iit::pegasus2::tpl::MotionTransforms<TRAIT>::Type_fr_torso_X_fr_RF_hip& 
iit::pegasus2::tpl::MotionTransforms<TRAIT>::Type_fr_torso_X_fr_RF_hip::update(const JState& q){ 
	Scalar c__q_RF_HAA_joint__;
	Scalar s__q_RF_HAA_joint__;

	c__q_RF_HAA_joint__ = TRAIT::cos( q[RF_HAA_JOINT]);
	s__q_RF_HAA_joint__ = TRAIT::sin( q[RF_HAA_JOINT]);

	(*this)(1, 0) = s__q_RF_HAA_joint__;
	(*this)(1, 1) = c__q_RF_HAA_joint__;
	(*this)(2, 0) = c__q_RF_HAA_joint__;
	(*this)(2, 1) = -s__q_RF_HAA_joint__;
	(*this)(3, 0) = -0.1185*c__q_RF_HAA_joint__;
	(*this)(3, 1) = 0.1185*s__q_RF_HAA_joint__;
	(*this)(4, 0) = -0.37*c__q_RF_HAA_joint__;
	(*this)(4, 1) = 0.37*s__q_RF_HAA_joint__;
	(*this)(4, 3) = s__q_RF_HAA_joint__;
	(*this)(4, 4) = c__q_RF_HAA_joint__;
	(*this)(5, 0) = 0.37*s__q_RF_HAA_joint__;
	(*this)(5, 1) = 0.37*c__q_RF_HAA_joint__;
	(*this)(5, 3) = c__q_RF_HAA_joint__;
	(*this)(5, 4) = -s__q_RF_HAA_joint__;
	return *this;
}

template <typename TRAIT>
iit::pegasus2::tpl::MotionTransforms<TRAIT>::Type_fr_RF_hip_X_fr_torso::Type_fr_RF_hip_X_fr_torso(){ 
	(*this)(0, 0) = 0;
	(*this)(0, 3) = 0;
	(*this)(0, 4) = 0;
	(*this)(0, 5) = 0;
	(*this)(1, 0) = 0;
	(*this)(1, 3) = 0;
	(*this)(1, 4) = 0;
	(*this)(1, 5) = 0;
	(*this)(2, 0) = -1;
	(*this)(2, 1) = 0;
	(*this)(2, 2) = 0;
	(*this)(2, 3) = 0;
	(*this)(2, 4) = 0;
	(*this)(2, 5) = 0;
	(*this)(3, 3) = 0;
	(*this)(4, 3) = 0;
	(*this)(5, 0) = 0;
	(*this)(5, 1) = 0;
	(*this)(5, 2) = -0.118500000000000;
	(*this)(5, 3) = -1;
	(*this)(5, 4) = 0;
	(*this)(5, 5) = 0;
}

template <typename TRAIT>
const typename iit::pegasus2::tpl::MotionTransforms<TRAIT>::Type_fr_RF_hip_X_fr_torso& 
iit::pegasus2::tpl::MotionTransforms<TRAIT>::Type_fr_RF_hip_X_fr_torso::update(const JState& q){ 
	Scalar c__q_RF_HAA_joint__;
	Scalar s__q_RF_HAA_joint__;

	c__q_RF_HAA_joint__ = TRAIT::cos( q[RF_HAA_JOINT]);
	s__q_RF_HAA_joint__ = TRAIT::sin( q[RF_HAA_JOINT]);

	(*this)(0, 1) = s__q_RF_HAA_joint__;
	(*this)(0, 2) = c__q_RF_HAA_joint__;
	(*this)(1, 1) = c__q_RF_HAA_joint__;
	(*this)(1, 2) = -s__q_RF_HAA_joint__;
	(*this)(3, 0) = -0.1185*c__q_RF_HAA_joint__;
	(*this)(3, 1) = -0.37*c__q_RF_HAA_joint__;
	(*this)(3, 2) = 0.37*s__q_RF_HAA_joint__;
	(*this)(3, 4) = s__q_RF_HAA_joint__;
	(*this)(3, 5) = c__q_RF_HAA_joint__;
	(*this)(4, 0) = 0.1185*s__q_RF_HAA_joint__;
	(*this)(4, 1) = 0.37*s__q_RF_HAA_joint__;
	(*this)(4, 2) = 0.37*c__q_RF_HAA_joint__;
	(*this)(4, 4) = c__q_RF_HAA_joint__;
	(*this)(4, 5) = -s__q_RF_HAA_joint__;
	return *this;
}

template <typename TRAIT>
iit::pegasus2::tpl::MotionTransforms<TRAIT>::Type_fr_torso_X_fr_RF_upperleg::Type_fr_torso_X_fr_RF_upperleg(){ 
	(*this)(0, 2) = 0;
	(*this)(0, 3) = 0;
	(*this)(0, 4) = 0;
	(*this)(0, 5) = 0;
	(*this)(1, 3) = 0;
	(*this)(1, 4) = 0;
	(*this)(1, 5) = 0;
	(*this)(2, 3) = 0;
	(*this)(2, 4) = 0;
	(*this)(2, 5) = 0;
	(*this)(3, 5) = 0;
}

template <typename TRAIT>
const typename iit::pegasus2::tpl::MotionTransforms<TRAIT>::Type_fr_torso_X_fr_RF_upperleg& 
iit::pegasus2::tpl::MotionTransforms<TRAIT>::Type_fr_torso_X_fr_RF_upperleg::update(const JState& q){ 
	Scalar c__q_RF_HAA_joint__;
	Scalar c__q_RF_HFE_joint__;
	Scalar s__q_RF_HFE_joint__;
	Scalar s__q_RF_HAA_joint__;

	c__q_RF_HAA_joint__ = TRAIT::cos( q[RF_HAA_JOINT]);
	c__q_RF_HFE_joint__ = TRAIT::cos( q[RF_HFE_JOINT]);
	s__q_RF_HFE_joint__ = TRAIT::sin( q[RF_HFE_JOINT]);
	s__q_RF_HAA_joint__ = TRAIT::sin( q[RF_HAA_JOINT]);

	(*this)(0, 0) = s__q_RF_HFE_joint__;
	(*this)(0, 1) = c__q_RF_HFE_joint__;
	(*this)(1, 0) = -s__q_RF_HAA_joint__*c__q_RF_HFE_joint__;
	(*this)(1, 1) = s__q_RF_HAA_joint__*s__q_RF_HFE_joint__;
	(*this)(1, 2) = -c__q_RF_HAA_joint__;
	(*this)(2, 0) = -c__q_RF_HAA_joint__*c__q_RF_HFE_joint__;
	(*this)(2, 1) = s__q_RF_HFE_joint__*c__q_RF_HAA_joint__;
	(*this)(2, 2) = s__q_RF_HAA_joint__;
	(*this)(3, 0) = (0.1185*c__q_RF_HAA_joint__ + 0.05)*c__q_RF_HFE_joint__;
	(*this)(3, 1) = -(0.1185*c__q_RF_HAA_joint__ + 0.05)*s__q_RF_HFE_joint__;
	(*this)(3, 2) = -0.1185*s__q_RF_HAA_joint__;
	(*this)(3, 3) = s__q_RF_HFE_joint__;
	(*this)(3, 4) = c__q_RF_HFE_joint__;
	(*this)(4, 0) = 0.05*s__q_RF_HAA_joint__*s__q_RF_HFE_joint__ + 0.37*c__q_RF_HAA_joint__*c__q_RF_HFE_joint__;
	(*this)(4, 1) = 0.05*s__q_RF_HAA_joint__*c__q_RF_HFE_joint__ - 0.37*s__q_RF_HFE_joint__*c__q_RF_HAA_joint__;
	(*this)(4, 2) = -0.37*s__q_RF_HAA_joint__;
	(*this)(4, 3) = -s__q_RF_HAA_joint__*c__q_RF_HFE_joint__;
	(*this)(4, 4) = s__q_RF_HAA_joint__*s__q_RF_HFE_joint__;
	(*this)(4, 5) = -c__q_RF_HAA_joint__;
	(*this)(5, 0) = -0.37*s__q_RF_HAA_joint__*c__q_RF_HFE_joint__ + 0.05*s__q_RF_HFE_joint__*c__q_RF_HAA_joint__ + 0.1185*s__q_RF_HFE_joint__;
	(*this)(5, 1) = 0.37*s__q_RF_HAA_joint__*s__q_RF_HFE_joint__ + 0.05*c__q_RF_HAA_joint__*c__q_RF_HFE_joint__ + 0.1185*c__q_RF_HFE_joint__;
	(*this)(5, 2) = -0.37*c__q_RF_HAA_joint__;
	(*this)(5, 3) = -c__q_RF_HAA_joint__*c__q_RF_HFE_joint__;
	(*this)(5, 4) = s__q_RF_HFE_joint__*c__q_RF_HAA_joint__;
	(*this)(5, 5) = s__q_RF_HAA_joint__;
	return *this;
}

template <typename TRAIT>
iit::pegasus2::tpl::MotionTransforms<TRAIT>::Type_fr_RF_upperleg_X_fr_torso::Type_fr_RF_upperleg_X_fr_torso(){ 
	(*this)(0, 3) = 0;
	(*this)(0, 4) = 0;
	(*this)(0, 5) = 0;
	(*this)(1, 3) = 0;
	(*this)(1, 4) = 0;
	(*this)(1, 5) = 0;
	(*this)(2, 0) = 0;
	(*this)(2, 3) = 0;
	(*this)(2, 4) = 0;
	(*this)(2, 5) = 0;
	(*this)(5, 3) = 0;
}

template <typename TRAIT>
const typename iit::pegasus2::tpl::MotionTransforms<TRAIT>::Type_fr_RF_upperleg_X_fr_torso& 
iit::pegasus2::tpl::MotionTransforms<TRAIT>::Type_fr_RF_upperleg_X_fr_torso::update(const JState& q){ 
	Scalar c__q_RF_HAA_joint__;
	Scalar c__q_RF_HFE_joint__;
	Scalar s__q_RF_HFE_joint__;
	Scalar s__q_RF_HAA_joint__;

	c__q_RF_HAA_joint__ = TRAIT::cos( q[RF_HAA_JOINT]);
	c__q_RF_HFE_joint__ = TRAIT::cos( q[RF_HFE_JOINT]);
	s__q_RF_HFE_joint__ = TRAIT::sin( q[RF_HFE_JOINT]);
	s__q_RF_HAA_joint__ = TRAIT::sin( q[RF_HAA_JOINT]);

	(*this)(0, 0) = s__q_RF_HFE_joint__;
	(*this)(0, 1) = -s__q_RF_HAA_joint__*c__q_RF_HFE_joint__;
	(*this)(0, 2) = -c__q_RF_HAA_joint__*c__q_RF_HFE_joint__;
	(*this)(1, 0) = c__q_RF_HFE_joint__;
	(*this)(1, 1) = s__q_RF_HAA_joint__*s__q_RF_HFE_joint__;
	(*this)(1, 2) = s__q_RF_HFE_joint__*c__q_RF_HAA_joint__;
	(*this)(2, 1) = -c__q_RF_HAA_joint__;
	(*this)(2, 2) = s__q_RF_HAA_joint__;
	(*this)(3, 0) = (0.1185*c__q_RF_HAA_joint__ + 0.05)*c__q_RF_HFE_joint__;
	(*this)(3, 1) = 0.05*s__q_RF_HAA_joint__*s__q_RF_HFE_joint__ + 0.37*c__q_RF_HAA_joint__*c__q_RF_HFE_joint__;
	(*this)(3, 2) = -0.37*s__q_RF_HAA_joint__*c__q_RF_HFE_joint__ + 0.05*s__q_RF_HFE_joint__*c__q_RF_HAA_joint__ + 0.1185*s__q_RF_HFE_joint__;
	(*this)(3, 3) = s__q_RF_HFE_joint__;
	(*this)(3, 4) = -s__q_RF_HAA_joint__*c__q_RF_HFE_joint__;
	(*this)(3, 5) = -c__q_RF_HAA_joint__*c__q_RF_HFE_joint__;
	(*this)(4, 0) = -(0.1185*c__q_RF_HAA_joint__ + 0.05)*s__q_RF_HFE_joint__;
	(*this)(4, 1) = 0.05*s__q_RF_HAA_joint__*c__q_RF_HFE_joint__ - 0.37*s__q_RF_HFE_joint__*c__q_RF_HAA_joint__;
	(*this)(4, 2) = 0.37*s__q_RF_HAA_joint__*s__q_RF_HFE_joint__ + 0.05*c__q_RF_HAA_joint__*c__q_RF_HFE_joint__ + 0.1185*c__q_RF_HFE_joint__;
	(*this)(4, 3) = c__q_RF_HFE_joint__;
	(*this)(4, 4) = s__q_RF_HAA_joint__*s__q_RF_HFE_joint__;
	(*this)(4, 5) = s__q_RF_HFE_joint__*c__q_RF_HAA_joint__;
	(*this)(5, 0) = -0.1185*s__q_RF_HAA_joint__;
	(*this)(5, 1) = -0.37*s__q_RF_HAA_joint__;
	(*this)(5, 2) = -0.37*c__q_RF_HAA_joint__;
	(*this)(5, 4) = -c__q_RF_HAA_joint__;
	(*this)(5, 5) = s__q_RF_HAA_joint__;
	return *this;
}

template <typename TRAIT>
iit::pegasus2::tpl::MotionTransforms<TRAIT>::Type_fr_torso_X_fr_RF_lowerleg::Type_fr_torso_X_fr_RF_lowerleg(){ 
	(*this)(0, 2) = 0;
	(*this)(0, 3) = 0;
	(*this)(0, 4) = 0;
	(*this)(0, 5) = 0;
	(*this)(1, 3) = 0;
	(*this)(1, 4) = 0;
	(*this)(1, 5) = 0;
	(*this)(2, 3) = 0;
	(*this)(2, 4) = 0;
	(*this)(2, 5) = 0;
	(*this)(3, 5) = 0;
}

template <typename TRAIT>
const typename iit::pegasus2::tpl::MotionTransforms<TRAIT>::Type_fr_torso_X_fr_RF_lowerleg& 
iit::pegasus2::tpl::MotionTransforms<TRAIT>::Type_fr_torso_X_fr_RF_lowerleg::update(const JState& q){ 
	Scalar c__q_RF_KFE_joint__;
	Scalar c__q_RF_HAA_joint__;
	Scalar c__q_RF_HFE_joint__;
	Scalar s__q_RF_HFE_joint__;
	Scalar s__q_RF_HAA_joint__;
	Scalar s__q_RF_KFE_joint__;

	c__q_RF_KFE_joint__ = TRAIT::cos( q[RF_KFE_JOINT]);
	c__q_RF_HAA_joint__ = TRAIT::cos( q[RF_HAA_JOINT]);
	c__q_RF_HFE_joint__ = TRAIT::cos( q[RF_HFE_JOINT]);
	s__q_RF_HFE_joint__ = TRAIT::sin( q[RF_HFE_JOINT]);
	s__q_RF_HAA_joint__ = TRAIT::sin( q[RF_HAA_JOINT]);
	s__q_RF_KFE_joint__ = TRAIT::sin( q[RF_KFE_JOINT]);

	(*this)(0, 0) = s__q_RF_HFE_joint__*c__q_RF_KFE_joint__ + s__q_RF_KFE_joint__*c__q_RF_HFE_joint__;
	(*this)(0, 1) = -s__q_RF_HFE_joint__*s__q_RF_KFE_joint__ + c__q_RF_HFE_joint__*c__q_RF_KFE_joint__;
	(*this)(1, 0) = -(-s__q_RF_HFE_joint__*s__q_RF_KFE_joint__ + c__q_RF_HFE_joint__*c__q_RF_KFE_joint__)*s__q_RF_HAA_joint__;
	(*this)(1, 1) = (s__q_RF_HFE_joint__*c__q_RF_KFE_joint__ + s__q_RF_KFE_joint__*c__q_RF_HFE_joint__)*s__q_RF_HAA_joint__;
	(*this)(1, 2) = -c__q_RF_HAA_joint__;
	(*this)(2, 0) = -(-s__q_RF_HFE_joint__*s__q_RF_KFE_joint__ + c__q_RF_HFE_joint__*c__q_RF_KFE_joint__)*c__q_RF_HAA_joint__;
	(*this)(2, 1) = (s__q_RF_HFE_joint__*c__q_RF_KFE_joint__ + s__q_RF_KFE_joint__*c__q_RF_HFE_joint__)*c__q_RF_HAA_joint__;
	(*this)(2, 2) = s__q_RF_HAA_joint__;
	(*this)(3, 0) = (-s__q_RF_HFE_joint__*s__q_RF_KFE_joint__ + c__q_RF_HFE_joint__*c__q_RF_KFE_joint__)*(0.1185*c__q_RF_HAA_joint__ + 0.05);
	(*this)(3, 1) = -(s__q_RF_HFE_joint__*c__q_RF_KFE_joint__ + s__q_RF_KFE_joint__*c__q_RF_HFE_joint__)*(0.1185*c__q_RF_HAA_joint__ + 0.05);
	(*this)(3, 2) = -0.1185*s__q_RF_HAA_joint__ - 0.38*c__q_RF_HFE_joint__;
	(*this)(3, 3) = s__q_RF_HFE_joint__*c__q_RF_KFE_joint__ + s__q_RF_KFE_joint__*c__q_RF_HFE_joint__;
	(*this)(3, 4) = -s__q_RF_HFE_joint__*s__q_RF_KFE_joint__ + c__q_RF_HFE_joint__*c__q_RF_KFE_joint__;
	(*this)(4, 0) = 0.21*(s__q_RF_HAA_joint__*s__q_RF_KFE_joint__ + c__q_RF_HAA_joint__*c__q_RF_KFE_joint__)*c__q_RF_HFE_joint__ - 0.21*(-s__q_RF_HAA_joint__*c__q_RF_KFE_joint__ + s__q_RF_KFE_joint__*c__q_RF_HAA_joint__)*s__q_RF_HFE_joint__ + 0.16*(-s__q_RF_HFE_joint__*s__q_RF_KFE_joint__ + c__q_RF_HFE_joint__*c__q_RF_KFE_joint__)*c__q_RF_HAA_joint__ - 0.16*(s__q_RF_HFE_joint__*c__q_RF_KFE_joint__ + s__q_RF_KFE_joint__*c__q_RF_HFE_joint__)*s__q_RF_HAA_joint__ - 0.38*s__q_RF_KFE_joint__*c__q_RF_HAA_joint__;
	(*this)(4, 1) = -0.21*(s__q_RF_HAA_joint__*s__q_RF_KFE_joint__ + c__q_RF_HAA_joint__*c__q_RF_KFE_joint__)*s__q_RF_HFE_joint__ - 0.21*(-s__q_RF_HAA_joint__*c__q_RF_KFE_joint__ + s__q_RF_KFE_joint__*c__q_RF_HAA_joint__)*c__q_RF_HFE_joint__ - 0.16*(-s__q_RF_HFE_joint__*s__q_RF_KFE_joint__ + c__q_RF_HFE_joint__*c__q_RF_KFE_joint__)*s__q_RF_HAA_joint__ - 0.16*(s__q_RF_HFE_joint__*c__q_RF_KFE_joint__ + s__q_RF_KFE_joint__*c__q_RF_HFE_joint__)*c__q_RF_HAA_joint__ - 0.38*c__q_RF_HAA_joint__*c__q_RF_KFE_joint__;
	(*this)(4, 2) = -(0.38*s__q_RF_HFE_joint__ + 0.37)*s__q_RF_HAA_joint__;
	(*this)(4, 3) = -(-s__q_RF_HFE_joint__*s__q_RF_KFE_joint__ + c__q_RF_HFE_joint__*c__q_RF_KFE_joint__)*s__q_RF_HAA_joint__;
	(*this)(4, 4) = (s__q_RF_HFE_joint__*c__q_RF_KFE_joint__ + s__q_RF_KFE_joint__*c__q_RF_HFE_joint__)*s__q_RF_HAA_joint__;
	(*this)(4, 5) = -c__q_RF_HAA_joint__;
	(*this)(5, 0) = -0.37*(-s__q_RF_HFE_joint__*s__q_RF_KFE_joint__ + c__q_RF_HFE_joint__*c__q_RF_KFE_joint__)*s__q_RF_HAA_joint__ + 0.05*(s__q_RF_HFE_joint__*c__q_RF_KFE_joint__ + s__q_RF_KFE_joint__*c__q_RF_HFE_joint__)*c__q_RF_HAA_joint__ + 0.38*s__q_RF_HAA_joint__*s__q_RF_KFE_joint__ + 0.1185*s__q_RF_HFE_joint__*c__q_RF_KFE_joint__ + 0.1185*s__q_RF_KFE_joint__*c__q_RF_HFE_joint__;
	(*this)(5, 1) = 0.05*(-s__q_RF_HFE_joint__*s__q_RF_KFE_joint__ + c__q_RF_HFE_joint__*c__q_RF_KFE_joint__)*c__q_RF_HAA_joint__ + 0.37*(s__q_RF_HFE_joint__*c__q_RF_KFE_joint__ + s__q_RF_KFE_joint__*c__q_RF_HFE_joint__)*s__q_RF_HAA_joint__ + 0.38*s__q_RF_HAA_joint__*c__q_RF_KFE_joint__ - 0.1185*s__q_RF_HFE_joint__*s__q_RF_KFE_joint__ + 0.1185*c__q_RF_HFE_joint__*c__q_RF_KFE_joint__;
	(*this)(5, 2) = -(0.38*s__q_RF_HFE_joint__ + 0.37)*c__q_RF_HAA_joint__;
	(*this)(5, 3) = -(-s__q_RF_HFE_joint__*s__q_RF_KFE_joint__ + c__q_RF_HFE_joint__*c__q_RF_KFE_joint__)*c__q_RF_HAA_joint__;
	(*this)(5, 4) = (s__q_RF_HFE_joint__*c__q_RF_KFE_joint__ + s__q_RF_KFE_joint__*c__q_RF_HFE_joint__)*c__q_RF_HAA_joint__;
	(*this)(5, 5) = s__q_RF_HAA_joint__;
	return *this;
}

template <typename TRAIT>
iit::pegasus2::tpl::MotionTransforms<TRAIT>::Type_fr_RF_lowerleg_X_fr_torso::Type_fr_RF_lowerleg_X_fr_torso(){ 
	(*this)(0, 3) = 0;
	(*this)(0, 4) = 0;
	(*this)(0, 5) = 0;
	(*this)(1, 3) = 0;
	(*this)(1, 4) = 0;
	(*this)(1, 5) = 0;
	(*this)(2, 0) = 0;
	(*this)(2, 3) = 0;
	(*this)(2, 4) = 0;
	(*this)(2, 5) = 0;
	(*this)(5, 3) = 0;
}

template <typename TRAIT>
const typename iit::pegasus2::tpl::MotionTransforms<TRAIT>::Type_fr_RF_lowerleg_X_fr_torso& 
iit::pegasus2::tpl::MotionTransforms<TRAIT>::Type_fr_RF_lowerleg_X_fr_torso::update(const JState& q){ 
	Scalar c__q_RF_KFE_joint__;
	Scalar c__q_RF_HAA_joint__;
	Scalar c__q_RF_HFE_joint__;
	Scalar s__q_RF_HFE_joint__;
	Scalar s__q_RF_HAA_joint__;
	Scalar s__q_RF_KFE_joint__;

	c__q_RF_KFE_joint__ = TRAIT::cos( q[RF_KFE_JOINT]);
	c__q_RF_HAA_joint__ = TRAIT::cos( q[RF_HAA_JOINT]);
	c__q_RF_HFE_joint__ = TRAIT::cos( q[RF_HFE_JOINT]);
	s__q_RF_HFE_joint__ = TRAIT::sin( q[RF_HFE_JOINT]);
	s__q_RF_HAA_joint__ = TRAIT::sin( q[RF_HAA_JOINT]);
	s__q_RF_KFE_joint__ = TRAIT::sin( q[RF_KFE_JOINT]);

	(*this)(0, 0) = s__q_RF_HFE_joint__*c__q_RF_KFE_joint__ + s__q_RF_KFE_joint__*c__q_RF_HFE_joint__;
	(*this)(0, 1) = -(-s__q_RF_HFE_joint__*s__q_RF_KFE_joint__ + c__q_RF_HFE_joint__*c__q_RF_KFE_joint__)*s__q_RF_HAA_joint__;
	(*this)(0, 2) = -(-s__q_RF_HFE_joint__*s__q_RF_KFE_joint__ + c__q_RF_HFE_joint__*c__q_RF_KFE_joint__)*c__q_RF_HAA_joint__;
	(*this)(1, 0) = -s__q_RF_HFE_joint__*s__q_RF_KFE_joint__ + c__q_RF_HFE_joint__*c__q_RF_KFE_joint__;
	(*this)(1, 1) = (s__q_RF_HFE_joint__*c__q_RF_KFE_joint__ + s__q_RF_KFE_joint__*c__q_RF_HFE_joint__)*s__q_RF_HAA_joint__;
	(*this)(1, 2) = (s__q_RF_HFE_joint__*c__q_RF_KFE_joint__ + s__q_RF_KFE_joint__*c__q_RF_HFE_joint__)*c__q_RF_HAA_joint__;
	(*this)(2, 1) = -c__q_RF_HAA_joint__;
	(*this)(2, 2) = s__q_RF_HAA_joint__;
	(*this)(3, 0) = (-s__q_RF_HFE_joint__*s__q_RF_KFE_joint__ + c__q_RF_HFE_joint__*c__q_RF_KFE_joint__)*(0.1185*c__q_RF_HAA_joint__ + 0.05);
	(*this)(3, 1) = 0.37*(-s__q_RF_HFE_joint__*s__q_RF_KFE_joint__ + c__q_RF_HFE_joint__*c__q_RF_KFE_joint__)*c__q_RF_HAA_joint__ + 0.05*(s__q_RF_HFE_joint__*c__q_RF_KFE_joint__ + s__q_RF_KFE_joint__*c__q_RF_HFE_joint__)*s__q_RF_HAA_joint__ - 0.38*s__q_RF_KFE_joint__*c__q_RF_HAA_joint__;
	(*this)(3, 2) = -0.37*(-s__q_RF_HFE_joint__*s__q_RF_KFE_joint__ + c__q_RF_HFE_joint__*c__q_RF_KFE_joint__)*s__q_RF_HAA_joint__ + 0.05*(s__q_RF_HFE_joint__*c__q_RF_KFE_joint__ + s__q_RF_KFE_joint__*c__q_RF_HFE_joint__)*c__q_RF_HAA_joint__ + 0.38*s__q_RF_HAA_joint__*s__q_RF_KFE_joint__ + 0.1185*s__q_RF_HFE_joint__*c__q_RF_KFE_joint__ + 0.1185*s__q_RF_KFE_joint__*c__q_RF_HFE_joint__;
	(*this)(3, 3) = s__q_RF_HFE_joint__*c__q_RF_KFE_joint__ + s__q_RF_KFE_joint__*c__q_RF_HFE_joint__;
	(*this)(3, 4) = -(-s__q_RF_HFE_joint__*s__q_RF_KFE_joint__ + c__q_RF_HFE_joint__*c__q_RF_KFE_joint__)*s__q_RF_HAA_joint__;
	(*this)(3, 5) = -(-s__q_RF_HFE_joint__*s__q_RF_KFE_joint__ + c__q_RF_HFE_joint__*c__q_RF_KFE_joint__)*c__q_RF_HAA_joint__;
	(*this)(4, 0) = -(s__q_RF_HFE_joint__*c__q_RF_KFE_joint__ + s__q_RF_KFE_joint__*c__q_RF_HFE_joint__)*(0.1185*c__q_RF_HAA_joint__ + 0.05);
	(*this)(4, 1) = 0.05*(-s__q_RF_HFE_joint__*s__q_RF_KFE_joint__ + c__q_RF_HFE_joint__*c__q_RF_KFE_joint__)*s__q_RF_HAA_joint__ - 0.37*(s__q_RF_HFE_joint__*c__q_RF_KFE_joint__ + s__q_RF_KFE_joint__*c__q_RF_HFE_joint__)*c__q_RF_HAA_joint__ - 0.38*c__q_RF_HAA_joint__*c__q_RF_KFE_joint__;
	(*this)(4, 2) = 0.05*(-s__q_RF_HFE_joint__*s__q_RF_KFE_joint__ + c__q_RF_HFE_joint__*c__q_RF_KFE_joint__)*c__q_RF_HAA_joint__ + 0.37*(s__q_RF_HFE_joint__*c__q_RF_KFE_joint__ + s__q_RF_KFE_joint__*c__q_RF_HFE_joint__)*s__q_RF_HAA_joint__ + 0.38*s__q_RF_HAA_joint__*c__q_RF_KFE_joint__ - 0.1185*s__q_RF_HFE_joint__*s__q_RF_KFE_joint__ + 0.1185*c__q_RF_HFE_joint__*c__q_RF_KFE_joint__;
	(*this)(4, 3) = -s__q_RF_HFE_joint__*s__q_RF_KFE_joint__ + c__q_RF_HFE_joint__*c__q_RF_KFE_joint__;
	(*this)(4, 4) = (s__q_RF_HFE_joint__*c__q_RF_KFE_joint__ + s__q_RF_KFE_joint__*c__q_RF_HFE_joint__)*s__q_RF_HAA_joint__;
	(*this)(4, 5) = (s__q_RF_HFE_joint__*c__q_RF_KFE_joint__ + s__q_RF_KFE_joint__*c__q_RF_HFE_joint__)*c__q_RF_HAA_joint__;
	(*this)(5, 0) = -0.1185*s__q_RF_HAA_joint__ - 0.38*c__q_RF_HFE_joint__;
	(*this)(5, 1) = -(0.38*s__q_RF_HFE_joint__ + 0.37)*s__q_RF_HAA_joint__;
	(*this)(5, 2) = -(0.38*s__q_RF_HFE_joint__ + 0.37)*c__q_RF_HAA_joint__;
	(*this)(5, 4) = -c__q_RF_HAA_joint__;
	(*this)(5, 5) = s__q_RF_HAA_joint__;
	return *this;
}

template <typename TRAIT>
iit::pegasus2::tpl::MotionTransforms<TRAIT>::Type_fr_torso_X_fr_RH_hip::Type_fr_torso_X_fr_RH_hip(){ 
	(*this)(0, 0) = 0;
	(*this)(0, 1) = 0;
	(*this)(0, 2) = -1;
	(*this)(0, 3) = 0;
	(*this)(0, 4) = 0;
	(*this)(0, 5) = 0;
	(*this)(1, 2) = 0;
	(*this)(1, 3) = 0;
	(*this)(1, 4) = 0;
	(*this)(1, 5) = 0;
	(*this)(2, 2) = 0;
	(*this)(2, 3) = 0;
	(*this)(2, 4) = 0;
	(*this)(2, 5) = 0;
	(*this)(3, 2) = 0;
	(*this)(3, 3) = 0;
	(*this)(3, 4) = 0;
	(*this)(3, 5) = -1;
	(*this)(4, 2) = 0;
	(*this)(4, 5) = 0;
	(*this)(5, 2) = -0.118500000000000;
	(*this)(5, 5) = 0;
}

template <typename TRAIT>
const typename iit::pegasus2::tpl::MotionTransforms<TRAIT>::Type_fr_torso_X_fr_RH_hip& 
iit::pegasus2::tpl::MotionTransforms<TRAIT>::Type_fr_torso_X_fr_RH_hip::update(const JState& q){ 
	Scalar c__q_RH_HAA_joint__;
	Scalar s__q_RH_HAA_joint__;

	c__q_RH_HAA_joint__ = TRAIT::cos( q[RH_HAA_JOINT]);
	s__q_RH_HAA_joint__ = TRAIT::sin( q[RH_HAA_JOINT]);

	(*this)(1, 0) = s__q_RH_HAA_joint__;
	(*this)(1, 1) = c__q_RH_HAA_joint__;
	(*this)(2, 0) = c__q_RH_HAA_joint__;
	(*this)(2, 1) = -s__q_RH_HAA_joint__;
	(*this)(3, 0) = -0.1185*c__q_RH_HAA_joint__;
	(*this)(3, 1) = 0.1185*s__q_RH_HAA_joint__;
	(*this)(4, 0) = 0.37*c__q_RH_HAA_joint__;
	(*this)(4, 1) = -0.37*s__q_RH_HAA_joint__;
	(*this)(4, 3) = s__q_RH_HAA_joint__;
	(*this)(4, 4) = c__q_RH_HAA_joint__;
	(*this)(5, 0) = -0.37*s__q_RH_HAA_joint__;
	(*this)(5, 1) = -0.37*c__q_RH_HAA_joint__;
	(*this)(5, 3) = c__q_RH_HAA_joint__;
	(*this)(5, 4) = -s__q_RH_HAA_joint__;
	return *this;
}

template <typename TRAIT>
iit::pegasus2::tpl::MotionTransforms<TRAIT>::Type_fr_RH_hip_X_fr_torso::Type_fr_RH_hip_X_fr_torso(){ 
	(*this)(0, 0) = 0;
	(*this)(0, 3) = 0;
	(*this)(0, 4) = 0;
	(*this)(0, 5) = 0;
	(*this)(1, 0) = 0;
	(*this)(1, 3) = 0;
	(*this)(1, 4) = 0;
	(*this)(1, 5) = 0;
	(*this)(2, 0) = -1;
	(*this)(2, 1) = 0;
	(*this)(2, 2) = 0;
	(*this)(2, 3) = 0;
	(*this)(2, 4) = 0;
	(*this)(2, 5) = 0;
	(*this)(3, 3) = 0;
	(*this)(4, 3) = 0;
	(*this)(5, 0) = 0;
	(*this)(5, 1) = 0;
	(*this)(5, 2) = -0.118500000000000;
	(*this)(5, 3) = -1;
	(*this)(5, 4) = 0;
	(*this)(5, 5) = 0;
}

template <typename TRAIT>
const typename iit::pegasus2::tpl::MotionTransforms<TRAIT>::Type_fr_RH_hip_X_fr_torso& 
iit::pegasus2::tpl::MotionTransforms<TRAIT>::Type_fr_RH_hip_X_fr_torso::update(const JState& q){ 
	Scalar c__q_RH_HAA_joint__;
	Scalar s__q_RH_HAA_joint__;

	c__q_RH_HAA_joint__ = TRAIT::cos( q[RH_HAA_JOINT]);
	s__q_RH_HAA_joint__ = TRAIT::sin( q[RH_HAA_JOINT]);

	(*this)(0, 1) = s__q_RH_HAA_joint__;
	(*this)(0, 2) = c__q_RH_HAA_joint__;
	(*this)(1, 1) = c__q_RH_HAA_joint__;
	(*this)(1, 2) = -s__q_RH_HAA_joint__;
	(*this)(3, 0) = -0.1185*c__q_RH_HAA_joint__;
	(*this)(3, 1) = 0.37*c__q_RH_HAA_joint__;
	(*this)(3, 2) = -0.37*s__q_RH_HAA_joint__;
	(*this)(3, 4) = s__q_RH_HAA_joint__;
	(*this)(3, 5) = c__q_RH_HAA_joint__;
	(*this)(4, 0) = 0.1185*s__q_RH_HAA_joint__;
	(*this)(4, 1) = -0.37*s__q_RH_HAA_joint__;
	(*this)(4, 2) = -0.37*c__q_RH_HAA_joint__;
	(*this)(4, 4) = c__q_RH_HAA_joint__;
	(*this)(4, 5) = -s__q_RH_HAA_joint__;
	return *this;
}

template <typename TRAIT>
iit::pegasus2::tpl::MotionTransforms<TRAIT>::Type_fr_torso_X_fr_RH_upperleg::Type_fr_torso_X_fr_RH_upperleg(){ 
	(*this)(0, 2) = 0;
	(*this)(0, 3) = 0;
	(*this)(0, 4) = 0;
	(*this)(0, 5) = 0;
	(*this)(1, 3) = 0;
	(*this)(1, 4) = 0;
	(*this)(1, 5) = 0;
	(*this)(2, 3) = 0;
	(*this)(2, 4) = 0;
	(*this)(2, 5) = 0;
	(*this)(3, 5) = 0;
}

template <typename TRAIT>
const typename iit::pegasus2::tpl::MotionTransforms<TRAIT>::Type_fr_torso_X_fr_RH_upperleg& 
iit::pegasus2::tpl::MotionTransforms<TRAIT>::Type_fr_torso_X_fr_RH_upperleg::update(const JState& q){ 
	Scalar c__q_RH_HFE_joint__;
	Scalar c__q_RH_HAA_joint__;
	Scalar s__q_RH_HAA_joint__;
	Scalar s__q_RH_HFE_joint__;

	c__q_RH_HFE_joint__ = TRAIT::cos( q[RH_HFE_JOINT]);
	c__q_RH_HAA_joint__ = TRAIT::cos( q[RH_HAA_JOINT]);
	s__q_RH_HAA_joint__ = TRAIT::sin( q[RH_HAA_JOINT]);
	s__q_RH_HFE_joint__ = TRAIT::sin( q[RH_HFE_JOINT]);

	(*this)(0, 0) = s__q_RH_HFE_joint__;
	(*this)(0, 1) = c__q_RH_HFE_joint__;
	(*this)(1, 0) = -s__q_RH_HAA_joint__*c__q_RH_HFE_joint__;
	(*this)(1, 1) = s__q_RH_HAA_joint__*s__q_RH_HFE_joint__;
	(*this)(1, 2) = -c__q_RH_HAA_joint__;
	(*this)(2, 0) = -c__q_RH_HAA_joint__*c__q_RH_HFE_joint__;
	(*this)(2, 1) = s__q_RH_HFE_joint__*c__q_RH_HAA_joint__;
	(*this)(2, 2) = s__q_RH_HAA_joint__;
	(*this)(3, 0) = (0.1185*c__q_RH_HAA_joint__ + 0.05)*c__q_RH_HFE_joint__;
	(*this)(3, 1) = -(0.1185*c__q_RH_HAA_joint__ + 0.05)*s__q_RH_HFE_joint__;
	(*this)(3, 2) = -0.1185*s__q_RH_HAA_joint__;
	(*this)(3, 3) = s__q_RH_HFE_joint__;
	(*this)(3, 4) = c__q_RH_HFE_joint__;
	(*this)(4, 0) = 0.05*s__q_RH_HAA_joint__*s__q_RH_HFE_joint__ - 0.37*c__q_RH_HAA_joint__*c__q_RH_HFE_joint__;
	(*this)(4, 1) = 0.05*s__q_RH_HAA_joint__*c__q_RH_HFE_joint__ + 0.37*s__q_RH_HFE_joint__*c__q_RH_HAA_joint__;
	(*this)(4, 2) = 0.37*s__q_RH_HAA_joint__;
	(*this)(4, 3) = -s__q_RH_HAA_joint__*c__q_RH_HFE_joint__;
	(*this)(4, 4) = s__q_RH_HAA_joint__*s__q_RH_HFE_joint__;
	(*this)(4, 5) = -c__q_RH_HAA_joint__;
	(*this)(5, 0) = 0.37*s__q_RH_HAA_joint__*c__q_RH_HFE_joint__ + 0.05*s__q_RH_HFE_joint__*c__q_RH_HAA_joint__ + 0.1185*s__q_RH_HFE_joint__;
	(*this)(5, 1) = -0.37*s__q_RH_HAA_joint__*s__q_RH_HFE_joint__ + 0.05*c__q_RH_HAA_joint__*c__q_RH_HFE_joint__ + 0.1185*c__q_RH_HFE_joint__;
	(*this)(5, 2) = 0.37*c__q_RH_HAA_joint__;
	(*this)(5, 3) = -c__q_RH_HAA_joint__*c__q_RH_HFE_joint__;
	(*this)(5, 4) = s__q_RH_HFE_joint__*c__q_RH_HAA_joint__;
	(*this)(5, 5) = s__q_RH_HAA_joint__;
	return *this;
}

template <typename TRAIT>
iit::pegasus2::tpl::MotionTransforms<TRAIT>::Type_fr_RH_upperleg_X_fr_torso::Type_fr_RH_upperleg_X_fr_torso(){ 
	(*this)(0, 3) = 0;
	(*this)(0, 4) = 0;
	(*this)(0, 5) = 0;
	(*this)(1, 3) = 0;
	(*this)(1, 4) = 0;
	(*this)(1, 5) = 0;
	(*this)(2, 0) = 0;
	(*this)(2, 3) = 0;
	(*this)(2, 4) = 0;
	(*this)(2, 5) = 0;
	(*this)(5, 3) = 0;
}

template <typename TRAIT>
const typename iit::pegasus2::tpl::MotionTransforms<TRAIT>::Type_fr_RH_upperleg_X_fr_torso& 
iit::pegasus2::tpl::MotionTransforms<TRAIT>::Type_fr_RH_upperleg_X_fr_torso::update(const JState& q){ 
	Scalar c__q_RH_HFE_joint__;
	Scalar c__q_RH_HAA_joint__;
	Scalar s__q_RH_HAA_joint__;
	Scalar s__q_RH_HFE_joint__;

	c__q_RH_HFE_joint__ = TRAIT::cos( q[RH_HFE_JOINT]);
	c__q_RH_HAA_joint__ = TRAIT::cos( q[RH_HAA_JOINT]);
	s__q_RH_HAA_joint__ = TRAIT::sin( q[RH_HAA_JOINT]);
	s__q_RH_HFE_joint__ = TRAIT::sin( q[RH_HFE_JOINT]);

	(*this)(0, 0) = s__q_RH_HFE_joint__;
	(*this)(0, 1) = -s__q_RH_HAA_joint__*c__q_RH_HFE_joint__;
	(*this)(0, 2) = -c__q_RH_HAA_joint__*c__q_RH_HFE_joint__;
	(*this)(1, 0) = c__q_RH_HFE_joint__;
	(*this)(1, 1) = s__q_RH_HAA_joint__*s__q_RH_HFE_joint__;
	(*this)(1, 2) = s__q_RH_HFE_joint__*c__q_RH_HAA_joint__;
	(*this)(2, 1) = -c__q_RH_HAA_joint__;
	(*this)(2, 2) = s__q_RH_HAA_joint__;
	(*this)(3, 0) = (0.1185*c__q_RH_HAA_joint__ + 0.05)*c__q_RH_HFE_joint__;
	(*this)(3, 1) = 0.05*s__q_RH_HAA_joint__*s__q_RH_HFE_joint__ - 0.37*c__q_RH_HAA_joint__*c__q_RH_HFE_joint__;
	(*this)(3, 2) = 0.37*s__q_RH_HAA_joint__*c__q_RH_HFE_joint__ + 0.05*s__q_RH_HFE_joint__*c__q_RH_HAA_joint__ + 0.1185*s__q_RH_HFE_joint__;
	(*this)(3, 3) = s__q_RH_HFE_joint__;
	(*this)(3, 4) = -s__q_RH_HAA_joint__*c__q_RH_HFE_joint__;
	(*this)(3, 5) = -c__q_RH_HAA_joint__*c__q_RH_HFE_joint__;
	(*this)(4, 0) = -(0.1185*c__q_RH_HAA_joint__ + 0.05)*s__q_RH_HFE_joint__;
	(*this)(4, 1) = 0.05*s__q_RH_HAA_joint__*c__q_RH_HFE_joint__ + 0.37*s__q_RH_HFE_joint__*c__q_RH_HAA_joint__;
	(*this)(4, 2) = -0.37*s__q_RH_HAA_joint__*s__q_RH_HFE_joint__ + 0.05*c__q_RH_HAA_joint__*c__q_RH_HFE_joint__ + 0.1185*c__q_RH_HFE_joint__;
	(*this)(4, 3) = c__q_RH_HFE_joint__;
	(*this)(4, 4) = s__q_RH_HAA_joint__*s__q_RH_HFE_joint__;
	(*this)(4, 5) = s__q_RH_HFE_joint__*c__q_RH_HAA_joint__;
	(*this)(5, 0) = -0.1185*s__q_RH_HAA_joint__;
	(*this)(5, 1) = 0.37*s__q_RH_HAA_joint__;
	(*this)(5, 2) = 0.37*c__q_RH_HAA_joint__;
	(*this)(5, 4) = -c__q_RH_HAA_joint__;
	(*this)(5, 5) = s__q_RH_HAA_joint__;
	return *this;
}

template <typename TRAIT>
iit::pegasus2::tpl::MotionTransforms<TRAIT>::Type_fr_torso_X_fr_RH_lowerleg::Type_fr_torso_X_fr_RH_lowerleg(){ 
	(*this)(0, 2) = 0;
	(*this)(0, 3) = 0;
	(*this)(0, 4) = 0;
	(*this)(0, 5) = 0;
	(*this)(1, 3) = 0;
	(*this)(1, 4) = 0;
	(*this)(1, 5) = 0;
	(*this)(2, 3) = 0;
	(*this)(2, 4) = 0;
	(*this)(2, 5) = 0;
	(*this)(3, 5) = 0;
}

template <typename TRAIT>
const typename iit::pegasus2::tpl::MotionTransforms<TRAIT>::Type_fr_torso_X_fr_RH_lowerleg& 
iit::pegasus2::tpl::MotionTransforms<TRAIT>::Type_fr_torso_X_fr_RH_lowerleg::update(const JState& q){ 
	Scalar c__q_RH_HFE_joint__;
	Scalar c__q_RH_HAA_joint__;
	Scalar c__q_RH_KFE_joint__;
	Scalar s__q_RH_KFE_joint__;
	Scalar s__q_RH_HAA_joint__;
	Scalar s__q_RH_HFE_joint__;

	c__q_RH_HFE_joint__ = TRAIT::cos( q[RH_HFE_JOINT]);
	c__q_RH_HAA_joint__ = TRAIT::cos( q[RH_HAA_JOINT]);
	c__q_RH_KFE_joint__ = TRAIT::cos( q[RH_KFE_JOINT]);
	s__q_RH_KFE_joint__ = TRAIT::sin( q[RH_KFE_JOINT]);
	s__q_RH_HAA_joint__ = TRAIT::sin( q[RH_HAA_JOINT]);
	s__q_RH_HFE_joint__ = TRAIT::sin( q[RH_HFE_JOINT]);

	(*this)(0, 0) = s__q_RH_HFE_joint__*c__q_RH_KFE_joint__ + s__q_RH_KFE_joint__*c__q_RH_HFE_joint__;
	(*this)(0, 1) = -s__q_RH_HFE_joint__*s__q_RH_KFE_joint__ + c__q_RH_HFE_joint__*c__q_RH_KFE_joint__;
	(*this)(1, 0) = -(-s__q_RH_HFE_joint__*s__q_RH_KFE_joint__ + c__q_RH_HFE_joint__*c__q_RH_KFE_joint__)*s__q_RH_HAA_joint__;
	(*this)(1, 1) = (s__q_RH_HFE_joint__*c__q_RH_KFE_joint__ + s__q_RH_KFE_joint__*c__q_RH_HFE_joint__)*s__q_RH_HAA_joint__;
	(*this)(1, 2) = -c__q_RH_HAA_joint__;
	(*this)(2, 0) = -(-s__q_RH_HFE_joint__*s__q_RH_KFE_joint__ + c__q_RH_HFE_joint__*c__q_RH_KFE_joint__)*c__q_RH_HAA_joint__;
	(*this)(2, 1) = (s__q_RH_HFE_joint__*c__q_RH_KFE_joint__ + s__q_RH_KFE_joint__*c__q_RH_HFE_joint__)*c__q_RH_HAA_joint__;
	(*this)(2, 2) = s__q_RH_HAA_joint__;
	(*this)(3, 0) = (-s__q_RH_HFE_joint__*s__q_RH_KFE_joint__ + c__q_RH_HFE_joint__*c__q_RH_KFE_joint__)*(0.1185*c__q_RH_HAA_joint__ + 0.05);
	(*this)(3, 1) = -(s__q_RH_HFE_joint__*c__q_RH_KFE_joint__ + s__q_RH_KFE_joint__*c__q_RH_HFE_joint__)*(0.1185*c__q_RH_HAA_joint__ + 0.05);
	(*this)(3, 2) = -0.1185*s__q_RH_HAA_joint__ - 0.38*c__q_RH_HFE_joint__;
	(*this)(3, 3) = s__q_RH_HFE_joint__*c__q_RH_KFE_joint__ + s__q_RH_KFE_joint__*c__q_RH_HFE_joint__;
	(*this)(3, 4) = -s__q_RH_HFE_joint__*s__q_RH_KFE_joint__ + c__q_RH_HFE_joint__*c__q_RH_KFE_joint__;
	(*this)(4, 0) = -0.37*(-s__q_RH_HFE_joint__*s__q_RH_KFE_joint__ + c__q_RH_HFE_joint__*c__q_RH_KFE_joint__)*c__q_RH_HAA_joint__ + 0.05*(s__q_RH_HFE_joint__*c__q_RH_KFE_joint__ + s__q_RH_KFE_joint__*c__q_RH_HFE_joint__)*s__q_RH_HAA_joint__ - 0.38*s__q_RH_KFE_joint__*c__q_RH_HAA_joint__;
	(*this)(4, 1) = 0.16*(s__q_RH_HAA_joint__*s__q_RH_KFE_joint__ + c__q_RH_HAA_joint__*c__q_RH_KFE_joint__)*s__q_RH_HFE_joint__ + 0.16*(-s__q_RH_HAA_joint__*c__q_RH_KFE_joint__ + s__q_RH_KFE_joint__*c__q_RH_HAA_joint__)*c__q_RH_HFE_joint__ + 0.21*(-s__q_RH_HFE_joint__*s__q_RH_KFE_joint__ + c__q_RH_HFE_joint__*c__q_RH_KFE_joint__)*s__q_RH_HAA_joint__ + 0.21*(s__q_RH_HFE_joint__*c__q_RH_KFE_joint__ + s__q_RH_KFE_joint__*c__q_RH_HFE_joint__)*c__q_RH_HAA_joint__ - 0.38*c__q_RH_HAA_joint__*c__q_RH_KFE_joint__;
	(*this)(4, 2) = (0.37 - 0.38*s__q_RH_HFE_joint__)*s__q_RH_HAA_joint__;
	(*this)(4, 3) = -(-s__q_RH_HFE_joint__*s__q_RH_KFE_joint__ + c__q_RH_HFE_joint__*c__q_RH_KFE_joint__)*s__q_RH_HAA_joint__;
	(*this)(4, 4) = (s__q_RH_HFE_joint__*c__q_RH_KFE_joint__ + s__q_RH_KFE_joint__*c__q_RH_HFE_joint__)*s__q_RH_HAA_joint__;
	(*this)(4, 5) = -c__q_RH_HAA_joint__;
	(*this)(5, 0) = 0.37*(-s__q_RH_HFE_joint__*s__q_RH_KFE_joint__ + c__q_RH_HFE_joint__*c__q_RH_KFE_joint__)*s__q_RH_HAA_joint__ + 0.05*(s__q_RH_HFE_joint__*c__q_RH_KFE_joint__ + s__q_RH_KFE_joint__*c__q_RH_HFE_joint__)*c__q_RH_HAA_joint__ + 0.38*s__q_RH_HAA_joint__*s__q_RH_KFE_joint__ + 0.1185*s__q_RH_HFE_joint__*c__q_RH_KFE_joint__ + 0.1185*s__q_RH_KFE_joint__*c__q_RH_HFE_joint__;
	(*this)(5, 1) = 0.05*(-s__q_RH_HFE_joint__*s__q_RH_KFE_joint__ + c__q_RH_HFE_joint__*c__q_RH_KFE_joint__)*c__q_RH_HAA_joint__ - 0.37*(s__q_RH_HFE_joint__*c__q_RH_KFE_joint__ + s__q_RH_KFE_joint__*c__q_RH_HFE_joint__)*s__q_RH_HAA_joint__ + 0.38*s__q_RH_HAA_joint__*c__q_RH_KFE_joint__ - 0.1185*s__q_RH_HFE_joint__*s__q_RH_KFE_joint__ + 0.1185*c__q_RH_HFE_joint__*c__q_RH_KFE_joint__;
	(*this)(5, 2) = (0.37 - 0.38*s__q_RH_HFE_joint__)*c__q_RH_HAA_joint__;
	(*this)(5, 3) = -(-s__q_RH_HFE_joint__*s__q_RH_KFE_joint__ + c__q_RH_HFE_joint__*c__q_RH_KFE_joint__)*c__q_RH_HAA_joint__;
	(*this)(5, 4) = (s__q_RH_HFE_joint__*c__q_RH_KFE_joint__ + s__q_RH_KFE_joint__*c__q_RH_HFE_joint__)*c__q_RH_HAA_joint__;
	(*this)(5, 5) = s__q_RH_HAA_joint__;
	return *this;
}

template <typename TRAIT>
iit::pegasus2::tpl::MotionTransforms<TRAIT>::Type_fr_RH_lowerleg_X_fr_torso::Type_fr_RH_lowerleg_X_fr_torso(){ 
	(*this)(0, 3) = 0;
	(*this)(0, 4) = 0;
	(*this)(0, 5) = 0;
	(*this)(1, 3) = 0;
	(*this)(1, 4) = 0;
	(*this)(1, 5) = 0;
	(*this)(2, 0) = 0;
	(*this)(2, 3) = 0;
	(*this)(2, 4) = 0;
	(*this)(2, 5) = 0;
	(*this)(5, 3) = 0;
}

template <typename TRAIT>
const typename iit::pegasus2::tpl::MotionTransforms<TRAIT>::Type_fr_RH_lowerleg_X_fr_torso& 
iit::pegasus2::tpl::MotionTransforms<TRAIT>::Type_fr_RH_lowerleg_X_fr_torso::update(const JState& q){ 
	Scalar c__q_RH_HFE_joint__;
	Scalar c__q_RH_HAA_joint__;
	Scalar c__q_RH_KFE_joint__;
	Scalar s__q_RH_KFE_joint__;
	Scalar s__q_RH_HAA_joint__;
	Scalar s__q_RH_HFE_joint__;

	c__q_RH_HFE_joint__ = TRAIT::cos( q[RH_HFE_JOINT]);
	c__q_RH_HAA_joint__ = TRAIT::cos( q[RH_HAA_JOINT]);
	c__q_RH_KFE_joint__ = TRAIT::cos( q[RH_KFE_JOINT]);
	s__q_RH_KFE_joint__ = TRAIT::sin( q[RH_KFE_JOINT]);
	s__q_RH_HAA_joint__ = TRAIT::sin( q[RH_HAA_JOINT]);
	s__q_RH_HFE_joint__ = TRAIT::sin( q[RH_HFE_JOINT]);

	(*this)(0, 0) = s__q_RH_HFE_joint__*c__q_RH_KFE_joint__ + s__q_RH_KFE_joint__*c__q_RH_HFE_joint__;
	(*this)(0, 1) = -(-s__q_RH_HFE_joint__*s__q_RH_KFE_joint__ + c__q_RH_HFE_joint__*c__q_RH_KFE_joint__)*s__q_RH_HAA_joint__;
	(*this)(0, 2) = -(-s__q_RH_HFE_joint__*s__q_RH_KFE_joint__ + c__q_RH_HFE_joint__*c__q_RH_KFE_joint__)*c__q_RH_HAA_joint__;
	(*this)(1, 0) = -s__q_RH_HFE_joint__*s__q_RH_KFE_joint__ + c__q_RH_HFE_joint__*c__q_RH_KFE_joint__;
	(*this)(1, 1) = (s__q_RH_HFE_joint__*c__q_RH_KFE_joint__ + s__q_RH_KFE_joint__*c__q_RH_HFE_joint__)*s__q_RH_HAA_joint__;
	(*this)(1, 2) = (s__q_RH_HFE_joint__*c__q_RH_KFE_joint__ + s__q_RH_KFE_joint__*c__q_RH_HFE_joint__)*c__q_RH_HAA_joint__;
	(*this)(2, 1) = -c__q_RH_HAA_joint__;
	(*this)(2, 2) = s__q_RH_HAA_joint__;
	(*this)(3, 0) = (-s__q_RH_HFE_joint__*s__q_RH_KFE_joint__ + c__q_RH_HFE_joint__*c__q_RH_KFE_joint__)*(0.1185*c__q_RH_HAA_joint__ + 0.05);
	(*this)(3, 1) = -0.37*(-s__q_RH_HFE_joint__*s__q_RH_KFE_joint__ + c__q_RH_HFE_joint__*c__q_RH_KFE_joint__)*c__q_RH_HAA_joint__ + 0.05*(s__q_RH_HFE_joint__*c__q_RH_KFE_joint__ + s__q_RH_KFE_joint__*c__q_RH_HFE_joint__)*s__q_RH_HAA_joint__ - 0.38*s__q_RH_KFE_joint__*c__q_RH_HAA_joint__;
	(*this)(3, 2) = 0.37*(-s__q_RH_HFE_joint__*s__q_RH_KFE_joint__ + c__q_RH_HFE_joint__*c__q_RH_KFE_joint__)*s__q_RH_HAA_joint__ + 0.05*(s__q_RH_HFE_joint__*c__q_RH_KFE_joint__ + s__q_RH_KFE_joint__*c__q_RH_HFE_joint__)*c__q_RH_HAA_joint__ + 0.38*s__q_RH_HAA_joint__*s__q_RH_KFE_joint__ + 0.1185*s__q_RH_HFE_joint__*c__q_RH_KFE_joint__ + 0.1185*s__q_RH_KFE_joint__*c__q_RH_HFE_joint__;
	(*this)(3, 3) = s__q_RH_HFE_joint__*c__q_RH_KFE_joint__ + s__q_RH_KFE_joint__*c__q_RH_HFE_joint__;
	(*this)(3, 4) = -(-s__q_RH_HFE_joint__*s__q_RH_KFE_joint__ + c__q_RH_HFE_joint__*c__q_RH_KFE_joint__)*s__q_RH_HAA_joint__;
	(*this)(3, 5) = -(-s__q_RH_HFE_joint__*s__q_RH_KFE_joint__ + c__q_RH_HFE_joint__*c__q_RH_KFE_joint__)*c__q_RH_HAA_joint__;
	(*this)(4, 0) = -(s__q_RH_HFE_joint__*c__q_RH_KFE_joint__ + s__q_RH_KFE_joint__*c__q_RH_HFE_joint__)*(0.1185*c__q_RH_HAA_joint__ + 0.05);
	(*this)(4, 1) = 0.05*(-s__q_RH_HFE_joint__*s__q_RH_KFE_joint__ + c__q_RH_HFE_joint__*c__q_RH_KFE_joint__)*s__q_RH_HAA_joint__ + 0.37*(s__q_RH_HFE_joint__*c__q_RH_KFE_joint__ + s__q_RH_KFE_joint__*c__q_RH_HFE_joint__)*c__q_RH_HAA_joint__ - 0.38*c__q_RH_HAA_joint__*c__q_RH_KFE_joint__;
	(*this)(4, 2) = 0.05*(-s__q_RH_HFE_joint__*s__q_RH_KFE_joint__ + c__q_RH_HFE_joint__*c__q_RH_KFE_joint__)*c__q_RH_HAA_joint__ - 0.37*(s__q_RH_HFE_joint__*c__q_RH_KFE_joint__ + s__q_RH_KFE_joint__*c__q_RH_HFE_joint__)*s__q_RH_HAA_joint__ + 0.38*s__q_RH_HAA_joint__*c__q_RH_KFE_joint__ - 0.1185*s__q_RH_HFE_joint__*s__q_RH_KFE_joint__ + 0.1185*c__q_RH_HFE_joint__*c__q_RH_KFE_joint__;
	(*this)(4, 3) = -s__q_RH_HFE_joint__*s__q_RH_KFE_joint__ + c__q_RH_HFE_joint__*c__q_RH_KFE_joint__;
	(*this)(4, 4) = (s__q_RH_HFE_joint__*c__q_RH_KFE_joint__ + s__q_RH_KFE_joint__*c__q_RH_HFE_joint__)*s__q_RH_HAA_joint__;
	(*this)(4, 5) = (s__q_RH_HFE_joint__*c__q_RH_KFE_joint__ + s__q_RH_KFE_joint__*c__q_RH_HFE_joint__)*c__q_RH_HAA_joint__;
	(*this)(5, 0) = -0.1185*s__q_RH_HAA_joint__ - 0.38*c__q_RH_HFE_joint__;
	(*this)(5, 1) = (0.37 - 0.38*s__q_RH_HFE_joint__)*s__q_RH_HAA_joint__;
	(*this)(5, 2) = (0.37 - 0.38*s__q_RH_HFE_joint__)*c__q_RH_HAA_joint__;
	(*this)(5, 4) = -c__q_RH_HAA_joint__;
	(*this)(5, 5) = s__q_RH_HAA_joint__;
	return *this;
}

template <typename TRAIT>
iit::pegasus2::tpl::MotionTransforms<TRAIT>::Type_fr_torso_X_fr_LH_hip::Type_fr_torso_X_fr_LH_hip(){ 
	(*this)(0, 0) = 0;
	(*this)(0, 1) = 0;
	(*this)(0, 2) = -1;
	(*this)(0, 3) = 0;
	(*this)(0, 4) = 0;
	(*this)(0, 5) = 0;
	(*this)(1, 2) = 0;
	(*this)(1, 3) = 0;
	(*this)(1, 4) = 0;
	(*this)(1, 5) = 0;
	(*this)(2, 2) = 0;
	(*this)(2, 3) = 0;
	(*this)(2, 4) = 0;
	(*this)(2, 5) = 0;
	(*this)(3, 2) = 0;
	(*this)(3, 3) = 0;
	(*this)(3, 4) = 0;
	(*this)(3, 5) = -1;
	(*this)(4, 2) = 0;
	(*this)(4, 5) = 0;
	(*this)(5, 2) = 0.118500000000000;
	(*this)(5, 5) = 0;
}

template <typename TRAIT>
const typename iit::pegasus2::tpl::MotionTransforms<TRAIT>::Type_fr_torso_X_fr_LH_hip& 
iit::pegasus2::tpl::MotionTransforms<TRAIT>::Type_fr_torso_X_fr_LH_hip::update(const JState& q){ 
	Scalar c__q_LH_HAA_joint__;
	Scalar s__q_LH_HAA_joint__;

	c__q_LH_HAA_joint__ = TRAIT::cos( q[LH_HAA_JOINT]);
	s__q_LH_HAA_joint__ = TRAIT::sin( q[LH_HAA_JOINT]);

	(*this)(1, 0) = s__q_LH_HAA_joint__;
	(*this)(1, 1) = c__q_LH_HAA_joint__;
	(*this)(2, 0) = c__q_LH_HAA_joint__;
	(*this)(2, 1) = -s__q_LH_HAA_joint__;
	(*this)(3, 0) = 0.1185*c__q_LH_HAA_joint__;
	(*this)(3, 1) = -0.1185*s__q_LH_HAA_joint__;
	(*this)(4, 0) = 0.37*c__q_LH_HAA_joint__;
	(*this)(4, 1) = -0.37*s__q_LH_HAA_joint__;
	(*this)(4, 3) = s__q_LH_HAA_joint__;
	(*this)(4, 4) = c__q_LH_HAA_joint__;
	(*this)(5, 0) = -0.37*s__q_LH_HAA_joint__;
	(*this)(5, 1) = -0.37*c__q_LH_HAA_joint__;
	(*this)(5, 3) = c__q_LH_HAA_joint__;
	(*this)(5, 4) = -s__q_LH_HAA_joint__;
	return *this;
}

template <typename TRAIT>
iit::pegasus2::tpl::MotionTransforms<TRAIT>::Type_fr_LH_hip_X_fr_torso::Type_fr_LH_hip_X_fr_torso(){ 
	(*this)(0, 0) = 0;
	(*this)(0, 3) = 0;
	(*this)(0, 4) = 0;
	(*this)(0, 5) = 0;
	(*this)(1, 0) = 0;
	(*this)(1, 3) = 0;
	(*this)(1, 4) = 0;
	(*this)(1, 5) = 0;
	(*this)(2, 0) = -1;
	(*this)(2, 1) = 0;
	(*this)(2, 2) = 0;
	(*this)(2, 3) = 0;
	(*this)(2, 4) = 0;
	(*this)(2, 5) = 0;
	(*this)(3, 3) = 0;
	(*this)(4, 3) = 0;
	(*this)(5, 0) = 0;
	(*this)(5, 1) = 0;
	(*this)(5, 2) = 0.118500000000000;
	(*this)(5, 3) = -1;
	(*this)(5, 4) = 0;
	(*this)(5, 5) = 0;
}

template <typename TRAIT>
const typename iit::pegasus2::tpl::MotionTransforms<TRAIT>::Type_fr_LH_hip_X_fr_torso& 
iit::pegasus2::tpl::MotionTransforms<TRAIT>::Type_fr_LH_hip_X_fr_torso::update(const JState& q){ 
	Scalar c__q_LH_HAA_joint__;
	Scalar s__q_LH_HAA_joint__;

	c__q_LH_HAA_joint__ = TRAIT::cos( q[LH_HAA_JOINT]);
	s__q_LH_HAA_joint__ = TRAIT::sin( q[LH_HAA_JOINT]);

	(*this)(0, 1) = s__q_LH_HAA_joint__;
	(*this)(0, 2) = c__q_LH_HAA_joint__;
	(*this)(1, 1) = c__q_LH_HAA_joint__;
	(*this)(1, 2) = -s__q_LH_HAA_joint__;
	(*this)(3, 0) = 0.1185*c__q_LH_HAA_joint__;
	(*this)(3, 1) = 0.37*c__q_LH_HAA_joint__;
	(*this)(3, 2) = -0.37*s__q_LH_HAA_joint__;
	(*this)(3, 4) = s__q_LH_HAA_joint__;
	(*this)(3, 5) = c__q_LH_HAA_joint__;
	(*this)(4, 0) = -0.1185*s__q_LH_HAA_joint__;
	(*this)(4, 1) = -0.37*s__q_LH_HAA_joint__;
	(*this)(4, 2) = -0.37*c__q_LH_HAA_joint__;
	(*this)(4, 4) = c__q_LH_HAA_joint__;
	(*this)(4, 5) = -s__q_LH_HAA_joint__;
	return *this;
}

template <typename TRAIT>
iit::pegasus2::tpl::MotionTransforms<TRAIT>::Type_fr_torso_X_fr_LH_upperleg::Type_fr_torso_X_fr_LH_upperleg(){ 
	(*this)(0, 2) = 0;
	(*this)(0, 3) = 0;
	(*this)(0, 4) = 0;
	(*this)(0, 5) = 0;
	(*this)(1, 3) = 0;
	(*this)(1, 4) = 0;
	(*this)(1, 5) = 0;
	(*this)(2, 3) = 0;
	(*this)(2, 4) = 0;
	(*this)(2, 5) = 0;
	(*this)(3, 5) = 0;
}

template <typename TRAIT>
const typename iit::pegasus2::tpl::MotionTransforms<TRAIT>::Type_fr_torso_X_fr_LH_upperleg& 
iit::pegasus2::tpl::MotionTransforms<TRAIT>::Type_fr_torso_X_fr_LH_upperleg::update(const JState& q){ 
	Scalar c__q_LH_HAA_joint__;
	Scalar c__q_LH_HFE_joint__;
	Scalar s__q_LH_HFE_joint__;
	Scalar s__q_LH_HAA_joint__;

	c__q_LH_HAA_joint__ = TRAIT::cos( q[LH_HAA_JOINT]);
	c__q_LH_HFE_joint__ = TRAIT::cos( q[LH_HFE_JOINT]);
	s__q_LH_HFE_joint__ = TRAIT::sin( q[LH_HFE_JOINT]);
	s__q_LH_HAA_joint__ = TRAIT::sin( q[LH_HAA_JOINT]);

	(*this)(0, 0) = s__q_LH_HFE_joint__;
	(*this)(0, 1) = c__q_LH_HFE_joint__;
	(*this)(1, 0) = -s__q_LH_HAA_joint__*c__q_LH_HFE_joint__;
	(*this)(1, 1) = s__q_LH_HAA_joint__*s__q_LH_HFE_joint__;
	(*this)(1, 2) = -c__q_LH_HAA_joint__;
	(*this)(2, 0) = -c__q_LH_HAA_joint__*c__q_LH_HFE_joint__;
	(*this)(2, 1) = s__q_LH_HFE_joint__*c__q_LH_HAA_joint__;
	(*this)(2, 2) = s__q_LH_HAA_joint__;
	(*this)(3, 0) = -(0.1185*c__q_LH_HAA_joint__ + 0.05)*c__q_LH_HFE_joint__;
	(*this)(3, 1) = (0.1185*c__q_LH_HAA_joint__ + 0.05)*s__q_LH_HFE_joint__;
	(*this)(3, 2) = 0.1185*s__q_LH_HAA_joint__;
	(*this)(3, 3) = s__q_LH_HFE_joint__;
	(*this)(3, 4) = c__q_LH_HFE_joint__;
	(*this)(4, 0) = -0.05*s__q_LH_HAA_joint__*s__q_LH_HFE_joint__ - 0.37*c__q_LH_HAA_joint__*c__q_LH_HFE_joint__;
	(*this)(4, 1) = -0.05*s__q_LH_HAA_joint__*c__q_LH_HFE_joint__ + 0.37*s__q_LH_HFE_joint__*c__q_LH_HAA_joint__;
	(*this)(4, 2) = 0.37*s__q_LH_HAA_joint__;
	(*this)(4, 3) = -s__q_LH_HAA_joint__*c__q_LH_HFE_joint__;
	(*this)(4, 4) = s__q_LH_HAA_joint__*s__q_LH_HFE_joint__;
	(*this)(4, 5) = -c__q_LH_HAA_joint__;
	(*this)(5, 0) = 0.37*s__q_LH_HAA_joint__*c__q_LH_HFE_joint__ - 0.05*s__q_LH_HFE_joint__*c__q_LH_HAA_joint__ - 0.1185*s__q_LH_HFE_joint__;
	(*this)(5, 1) = -0.37*s__q_LH_HAA_joint__*s__q_LH_HFE_joint__ - 0.05*c__q_LH_HAA_joint__*c__q_LH_HFE_joint__ - 0.1185*c__q_LH_HFE_joint__;
	(*this)(5, 2) = 0.37*c__q_LH_HAA_joint__;
	(*this)(5, 3) = -c__q_LH_HAA_joint__*c__q_LH_HFE_joint__;
	(*this)(5, 4) = s__q_LH_HFE_joint__*c__q_LH_HAA_joint__;
	(*this)(5, 5) = s__q_LH_HAA_joint__;
	return *this;
}

template <typename TRAIT>
iit::pegasus2::tpl::MotionTransforms<TRAIT>::Type_fr_LH_upperleg_X_fr_torso::Type_fr_LH_upperleg_X_fr_torso(){ 
	(*this)(0, 3) = 0;
	(*this)(0, 4) = 0;
	(*this)(0, 5) = 0;
	(*this)(1, 3) = 0;
	(*this)(1, 4) = 0;
	(*this)(1, 5) = 0;
	(*this)(2, 0) = 0;
	(*this)(2, 3) = 0;
	(*this)(2, 4) = 0;
	(*this)(2, 5) = 0;
	(*this)(5, 3) = 0;
}

template <typename TRAIT>
const typename iit::pegasus2::tpl::MotionTransforms<TRAIT>::Type_fr_LH_upperleg_X_fr_torso& 
iit::pegasus2::tpl::MotionTransforms<TRAIT>::Type_fr_LH_upperleg_X_fr_torso::update(const JState& q){ 
	Scalar c__q_LH_HAA_joint__;
	Scalar c__q_LH_HFE_joint__;
	Scalar s__q_LH_HFE_joint__;
	Scalar s__q_LH_HAA_joint__;

	c__q_LH_HAA_joint__ = TRAIT::cos( q[LH_HAA_JOINT]);
	c__q_LH_HFE_joint__ = TRAIT::cos( q[LH_HFE_JOINT]);
	s__q_LH_HFE_joint__ = TRAIT::sin( q[LH_HFE_JOINT]);
	s__q_LH_HAA_joint__ = TRAIT::sin( q[LH_HAA_JOINT]);

	(*this)(0, 0) = s__q_LH_HFE_joint__;
	(*this)(0, 1) = -s__q_LH_HAA_joint__*c__q_LH_HFE_joint__;
	(*this)(0, 2) = -c__q_LH_HAA_joint__*c__q_LH_HFE_joint__;
	(*this)(1, 0) = c__q_LH_HFE_joint__;
	(*this)(1, 1) = s__q_LH_HAA_joint__*s__q_LH_HFE_joint__;
	(*this)(1, 2) = s__q_LH_HFE_joint__*c__q_LH_HAA_joint__;
	(*this)(2, 1) = -c__q_LH_HAA_joint__;
	(*this)(2, 2) = s__q_LH_HAA_joint__;
	(*this)(3, 0) = -(0.1185*c__q_LH_HAA_joint__ + 0.05)*c__q_LH_HFE_joint__;
	(*this)(3, 1) = -0.05*s__q_LH_HAA_joint__*s__q_LH_HFE_joint__ - 0.37*c__q_LH_HAA_joint__*c__q_LH_HFE_joint__;
	(*this)(3, 2) = 0.37*s__q_LH_HAA_joint__*c__q_LH_HFE_joint__ - 0.05*s__q_LH_HFE_joint__*c__q_LH_HAA_joint__ - 0.1185*s__q_LH_HFE_joint__;
	(*this)(3, 3) = s__q_LH_HFE_joint__;
	(*this)(3, 4) = -s__q_LH_HAA_joint__*c__q_LH_HFE_joint__;
	(*this)(3, 5) = -c__q_LH_HAA_joint__*c__q_LH_HFE_joint__;
	(*this)(4, 0) = (0.1185*c__q_LH_HAA_joint__ + 0.05)*s__q_LH_HFE_joint__;
	(*this)(4, 1) = -0.05*s__q_LH_HAA_joint__*c__q_LH_HFE_joint__ + 0.37*s__q_LH_HFE_joint__*c__q_LH_HAA_joint__;
	(*this)(4, 2) = -0.37*s__q_LH_HAA_joint__*s__q_LH_HFE_joint__ - 0.05*c__q_LH_HAA_joint__*c__q_LH_HFE_joint__ - 0.1185*c__q_LH_HFE_joint__;
	(*this)(4, 3) = c__q_LH_HFE_joint__;
	(*this)(4, 4) = s__q_LH_HAA_joint__*s__q_LH_HFE_joint__;
	(*this)(4, 5) = s__q_LH_HFE_joint__*c__q_LH_HAA_joint__;
	(*this)(5, 0) = 0.1185*s__q_LH_HAA_joint__;
	(*this)(5, 1) = 0.37*s__q_LH_HAA_joint__;
	(*this)(5, 2) = 0.37*c__q_LH_HAA_joint__;
	(*this)(5, 4) = -c__q_LH_HAA_joint__;
	(*this)(5, 5) = s__q_LH_HAA_joint__;
	return *this;
}

template <typename TRAIT>
iit::pegasus2::tpl::MotionTransforms<TRAIT>::Type_fr_torso_X_fr_LH_lowerleg::Type_fr_torso_X_fr_LH_lowerleg(){ 
	(*this)(0, 2) = 0;
	(*this)(0, 3) = 0;
	(*this)(0, 4) = 0;
	(*this)(0, 5) = 0;
	(*this)(1, 3) = 0;
	(*this)(1, 4) = 0;
	(*this)(1, 5) = 0;
	(*this)(2, 3) = 0;
	(*this)(2, 4) = 0;
	(*this)(2, 5) = 0;
	(*this)(3, 5) = 0;
}

template <typename TRAIT>
const typename iit::pegasus2::tpl::MotionTransforms<TRAIT>::Type_fr_torso_X_fr_LH_lowerleg& 
iit::pegasus2::tpl::MotionTransforms<TRAIT>::Type_fr_torso_X_fr_LH_lowerleg::update(const JState& q){ 
	Scalar c__q_LH_KFE_joint__;
	Scalar c__q_LH_HAA_joint__;
	Scalar c__q_LH_HFE_joint__;
	Scalar s__q_LH_HFE_joint__;
	Scalar s__q_LH_HAA_joint__;
	Scalar s__q_LH_KFE_joint__;

	c__q_LH_KFE_joint__ = TRAIT::cos( q[LH_KFE_JOINT]);
	c__q_LH_HAA_joint__ = TRAIT::cos( q[LH_HAA_JOINT]);
	c__q_LH_HFE_joint__ = TRAIT::cos( q[LH_HFE_JOINT]);
	s__q_LH_HFE_joint__ = TRAIT::sin( q[LH_HFE_JOINT]);
	s__q_LH_HAA_joint__ = TRAIT::sin( q[LH_HAA_JOINT]);
	s__q_LH_KFE_joint__ = TRAIT::sin( q[LH_KFE_JOINT]);

	(*this)(0, 0) = s__q_LH_HFE_joint__*c__q_LH_KFE_joint__ + s__q_LH_KFE_joint__*c__q_LH_HFE_joint__;
	(*this)(0, 1) = -s__q_LH_HFE_joint__*s__q_LH_KFE_joint__ + c__q_LH_HFE_joint__*c__q_LH_KFE_joint__;
	(*this)(1, 0) = -(-s__q_LH_HFE_joint__*s__q_LH_KFE_joint__ + c__q_LH_HFE_joint__*c__q_LH_KFE_joint__)*s__q_LH_HAA_joint__;
	(*this)(1, 1) = (s__q_LH_HFE_joint__*c__q_LH_KFE_joint__ + s__q_LH_KFE_joint__*c__q_LH_HFE_joint__)*s__q_LH_HAA_joint__;
	(*this)(1, 2) = -c__q_LH_HAA_joint__;
	(*this)(2, 0) = -(-s__q_LH_HFE_joint__*s__q_LH_KFE_joint__ + c__q_LH_HFE_joint__*c__q_LH_KFE_joint__)*c__q_LH_HAA_joint__;
	(*this)(2, 1) = (s__q_LH_HFE_joint__*c__q_LH_KFE_joint__ + s__q_LH_KFE_joint__*c__q_LH_HFE_joint__)*c__q_LH_HAA_joint__;
	(*this)(2, 2) = s__q_LH_HAA_joint__;
	(*this)(3, 0) = -(-s__q_LH_HFE_joint__*s__q_LH_KFE_joint__ + c__q_LH_HFE_joint__*c__q_LH_KFE_joint__)*(0.1185*c__q_LH_HAA_joint__ + 0.05);
	(*this)(3, 1) = (s__q_LH_HFE_joint__*c__q_LH_KFE_joint__ + s__q_LH_KFE_joint__*c__q_LH_HFE_joint__)*(0.1185*c__q_LH_HAA_joint__ + 0.05);
	(*this)(3, 2) = 0.1185*s__q_LH_HAA_joint__ - 0.38*c__q_LH_HFE_joint__;
	(*this)(3, 3) = s__q_LH_HFE_joint__*c__q_LH_KFE_joint__ + s__q_LH_KFE_joint__*c__q_LH_HFE_joint__;
	(*this)(3, 4) = -s__q_LH_HFE_joint__*s__q_LH_KFE_joint__ + c__q_LH_HFE_joint__*c__q_LH_KFE_joint__;
	(*this)(4, 0) = -0.21*(s__q_LH_HAA_joint__*s__q_LH_KFE_joint__ + c__q_LH_HAA_joint__*c__q_LH_KFE_joint__)*c__q_LH_HFE_joint__ + 0.21*(-s__q_LH_HAA_joint__*c__q_LH_KFE_joint__ + s__q_LH_KFE_joint__*c__q_LH_HAA_joint__)*s__q_LH_HFE_joint__ - 0.16*(-s__q_LH_HFE_joint__*s__q_LH_KFE_joint__ + c__q_LH_HFE_joint__*c__q_LH_KFE_joint__)*c__q_LH_HAA_joint__ + 0.16*(s__q_LH_HFE_joint__*c__q_LH_KFE_joint__ + s__q_LH_KFE_joint__*c__q_LH_HFE_joint__)*s__q_LH_HAA_joint__ - 0.38*s__q_LH_KFE_joint__*c__q_LH_HAA_joint__;
	(*this)(4, 1) = 0.21*(s__q_LH_HAA_joint__*s__q_LH_KFE_joint__ + c__q_LH_HAA_joint__*c__q_LH_KFE_joint__)*s__q_LH_HFE_joint__ + 0.21*(-s__q_LH_HAA_joint__*c__q_LH_KFE_joint__ + s__q_LH_KFE_joint__*c__q_LH_HAA_joint__)*c__q_LH_HFE_joint__ + 0.16*(-s__q_LH_HFE_joint__*s__q_LH_KFE_joint__ + c__q_LH_HFE_joint__*c__q_LH_KFE_joint__)*s__q_LH_HAA_joint__ + 0.16*(s__q_LH_HFE_joint__*c__q_LH_KFE_joint__ + s__q_LH_KFE_joint__*c__q_LH_HFE_joint__)*c__q_LH_HAA_joint__ - 0.38*c__q_LH_HAA_joint__*c__q_LH_KFE_joint__;
	(*this)(4, 2) = (0.37 - 0.38*s__q_LH_HFE_joint__)*s__q_LH_HAA_joint__;
	(*this)(4, 3) = -(-s__q_LH_HFE_joint__*s__q_LH_KFE_joint__ + c__q_LH_HFE_joint__*c__q_LH_KFE_joint__)*s__q_LH_HAA_joint__;
	(*this)(4, 4) = (s__q_LH_HFE_joint__*c__q_LH_KFE_joint__ + s__q_LH_KFE_joint__*c__q_LH_HFE_joint__)*s__q_LH_HAA_joint__;
	(*this)(4, 5) = -c__q_LH_HAA_joint__;
	(*this)(5, 0) = 0.37*(-s__q_LH_HFE_joint__*s__q_LH_KFE_joint__ + c__q_LH_HFE_joint__*c__q_LH_KFE_joint__)*s__q_LH_HAA_joint__ - 0.05*(s__q_LH_HFE_joint__*c__q_LH_KFE_joint__ + s__q_LH_KFE_joint__*c__q_LH_HFE_joint__)*c__q_LH_HAA_joint__ + 0.38*s__q_LH_HAA_joint__*s__q_LH_KFE_joint__ - 0.1185*s__q_LH_HFE_joint__*c__q_LH_KFE_joint__ - 0.1185*s__q_LH_KFE_joint__*c__q_LH_HFE_joint__;
	(*this)(5, 1) = -0.05*(-s__q_LH_HFE_joint__*s__q_LH_KFE_joint__ + c__q_LH_HFE_joint__*c__q_LH_KFE_joint__)*c__q_LH_HAA_joint__ - 0.37*(s__q_LH_HFE_joint__*c__q_LH_KFE_joint__ + s__q_LH_KFE_joint__*c__q_LH_HFE_joint__)*s__q_LH_HAA_joint__ + 0.38*s__q_LH_HAA_joint__*c__q_LH_KFE_joint__ + 0.1185*s__q_LH_HFE_joint__*s__q_LH_KFE_joint__ - 0.1185*c__q_LH_HFE_joint__*c__q_LH_KFE_joint__;
	(*this)(5, 2) = (0.37 - 0.38*s__q_LH_HFE_joint__)*c__q_LH_HAA_joint__;
	(*this)(5, 3) = -(-s__q_LH_HFE_joint__*s__q_LH_KFE_joint__ + c__q_LH_HFE_joint__*c__q_LH_KFE_joint__)*c__q_LH_HAA_joint__;
	(*this)(5, 4) = (s__q_LH_HFE_joint__*c__q_LH_KFE_joint__ + s__q_LH_KFE_joint__*c__q_LH_HFE_joint__)*c__q_LH_HAA_joint__;
	(*this)(5, 5) = s__q_LH_HAA_joint__;
	return *this;
}

template <typename TRAIT>
iit::pegasus2::tpl::MotionTransforms<TRAIT>::Type_fr_LH_lowerleg_X_fr_torso::Type_fr_LH_lowerleg_X_fr_torso(){ 
	(*this)(0, 3) = 0;
	(*this)(0, 4) = 0;
	(*this)(0, 5) = 0;
	(*this)(1, 3) = 0;
	(*this)(1, 4) = 0;
	(*this)(1, 5) = 0;
	(*this)(2, 0) = 0;
	(*this)(2, 3) = 0;
	(*this)(2, 4) = 0;
	(*this)(2, 5) = 0;
	(*this)(5, 3) = 0;
}

template <typename TRAIT>
const typename iit::pegasus2::tpl::MotionTransforms<TRAIT>::Type_fr_LH_lowerleg_X_fr_torso& 
iit::pegasus2::tpl::MotionTransforms<TRAIT>::Type_fr_LH_lowerleg_X_fr_torso::update(const JState& q){ 
	Scalar c__q_LH_KFE_joint__;
	Scalar c__q_LH_HAA_joint__;
	Scalar c__q_LH_HFE_joint__;
	Scalar s__q_LH_HFE_joint__;
	Scalar s__q_LH_HAA_joint__;
	Scalar s__q_LH_KFE_joint__;

	c__q_LH_KFE_joint__ = TRAIT::cos( q[LH_KFE_JOINT]);
	c__q_LH_HAA_joint__ = TRAIT::cos( q[LH_HAA_JOINT]);
	c__q_LH_HFE_joint__ = TRAIT::cos( q[LH_HFE_JOINT]);
	s__q_LH_HFE_joint__ = TRAIT::sin( q[LH_HFE_JOINT]);
	s__q_LH_HAA_joint__ = TRAIT::sin( q[LH_HAA_JOINT]);
	s__q_LH_KFE_joint__ = TRAIT::sin( q[LH_KFE_JOINT]);

	(*this)(0, 0) = s__q_LH_HFE_joint__*c__q_LH_KFE_joint__ + s__q_LH_KFE_joint__*c__q_LH_HFE_joint__;
	(*this)(0, 1) = -(-s__q_LH_HFE_joint__*s__q_LH_KFE_joint__ + c__q_LH_HFE_joint__*c__q_LH_KFE_joint__)*s__q_LH_HAA_joint__;
	(*this)(0, 2) = -(-s__q_LH_HFE_joint__*s__q_LH_KFE_joint__ + c__q_LH_HFE_joint__*c__q_LH_KFE_joint__)*c__q_LH_HAA_joint__;
	(*this)(1, 0) = -s__q_LH_HFE_joint__*s__q_LH_KFE_joint__ + c__q_LH_HFE_joint__*c__q_LH_KFE_joint__;
	(*this)(1, 1) = (s__q_LH_HFE_joint__*c__q_LH_KFE_joint__ + s__q_LH_KFE_joint__*c__q_LH_HFE_joint__)*s__q_LH_HAA_joint__;
	(*this)(1, 2) = (s__q_LH_HFE_joint__*c__q_LH_KFE_joint__ + s__q_LH_KFE_joint__*c__q_LH_HFE_joint__)*c__q_LH_HAA_joint__;
	(*this)(2, 1) = -c__q_LH_HAA_joint__;
	(*this)(2, 2) = s__q_LH_HAA_joint__;
	(*this)(3, 0) = -(-s__q_LH_HFE_joint__*s__q_LH_KFE_joint__ + c__q_LH_HFE_joint__*c__q_LH_KFE_joint__)*(0.1185*c__q_LH_HAA_joint__ + 0.05);
	(*this)(3, 1) = -0.37*(-s__q_LH_HFE_joint__*s__q_LH_KFE_joint__ + c__q_LH_HFE_joint__*c__q_LH_KFE_joint__)*c__q_LH_HAA_joint__ - 0.05*(s__q_LH_HFE_joint__*c__q_LH_KFE_joint__ + s__q_LH_KFE_joint__*c__q_LH_HFE_joint__)*s__q_LH_HAA_joint__ - 0.38*s__q_LH_KFE_joint__*c__q_LH_HAA_joint__;
	(*this)(3, 2) = 0.37*(-s__q_LH_HFE_joint__*s__q_LH_KFE_joint__ + c__q_LH_HFE_joint__*c__q_LH_KFE_joint__)*s__q_LH_HAA_joint__ - 0.05*(s__q_LH_HFE_joint__*c__q_LH_KFE_joint__ + s__q_LH_KFE_joint__*c__q_LH_HFE_joint__)*c__q_LH_HAA_joint__ + 0.38*s__q_LH_HAA_joint__*s__q_LH_KFE_joint__ - 0.1185*s__q_LH_HFE_joint__*c__q_LH_KFE_joint__ - 0.1185*s__q_LH_KFE_joint__*c__q_LH_HFE_joint__;
	(*this)(3, 3) = s__q_LH_HFE_joint__*c__q_LH_KFE_joint__ + s__q_LH_KFE_joint__*c__q_LH_HFE_joint__;
	(*this)(3, 4) = -(-s__q_LH_HFE_joint__*s__q_LH_KFE_joint__ + c__q_LH_HFE_joint__*c__q_LH_KFE_joint__)*s__q_LH_HAA_joint__;
	(*this)(3, 5) = -(-s__q_LH_HFE_joint__*s__q_LH_KFE_joint__ + c__q_LH_HFE_joint__*c__q_LH_KFE_joint__)*c__q_LH_HAA_joint__;
	(*this)(4, 0) = (s__q_LH_HFE_joint__*c__q_LH_KFE_joint__ + s__q_LH_KFE_joint__*c__q_LH_HFE_joint__)*(0.1185*c__q_LH_HAA_joint__ + 0.05);
	(*this)(4, 1) = -0.05*(-s__q_LH_HFE_joint__*s__q_LH_KFE_joint__ + c__q_LH_HFE_joint__*c__q_LH_KFE_joint__)*s__q_LH_HAA_joint__ + 0.37*(s__q_LH_HFE_joint__*c__q_LH_KFE_joint__ + s__q_LH_KFE_joint__*c__q_LH_HFE_joint__)*c__q_LH_HAA_joint__ - 0.38*c__q_LH_HAA_joint__*c__q_LH_KFE_joint__;
	(*this)(4, 2) = -0.05*(-s__q_LH_HFE_joint__*s__q_LH_KFE_joint__ + c__q_LH_HFE_joint__*c__q_LH_KFE_joint__)*c__q_LH_HAA_joint__ - 0.37*(s__q_LH_HFE_joint__*c__q_LH_KFE_joint__ + s__q_LH_KFE_joint__*c__q_LH_HFE_joint__)*s__q_LH_HAA_joint__ + 0.38*s__q_LH_HAA_joint__*c__q_LH_KFE_joint__ + 0.1185*s__q_LH_HFE_joint__*s__q_LH_KFE_joint__ - 0.1185*c__q_LH_HFE_joint__*c__q_LH_KFE_joint__;
	(*this)(4, 3) = -s__q_LH_HFE_joint__*s__q_LH_KFE_joint__ + c__q_LH_HFE_joint__*c__q_LH_KFE_joint__;
	(*this)(4, 4) = (s__q_LH_HFE_joint__*c__q_LH_KFE_joint__ + s__q_LH_KFE_joint__*c__q_LH_HFE_joint__)*s__q_LH_HAA_joint__;
	(*this)(4, 5) = (s__q_LH_HFE_joint__*c__q_LH_KFE_joint__ + s__q_LH_KFE_joint__*c__q_LH_HFE_joint__)*c__q_LH_HAA_joint__;
	(*this)(5, 0) = 0.1185*s__q_LH_HAA_joint__ - 0.38*c__q_LH_HFE_joint__;
	(*this)(5, 1) = (0.37 - 0.38*s__q_LH_HFE_joint__)*s__q_LH_HAA_joint__;
	(*this)(5, 2) = (0.37 - 0.38*s__q_LH_HFE_joint__)*c__q_LH_HAA_joint__;
	(*this)(5, 4) = -c__q_LH_HAA_joint__;
	(*this)(5, 5) = s__q_LH_HAA_joint__;
	return *this;
}

template <typename TRAIT>
iit::pegasus2::tpl::MotionTransforms<TRAIT>::Type_fr_torso_X_fr_LF_hip::Type_fr_torso_X_fr_LF_hip(){ 
	(*this)(0, 0) = 0;
	(*this)(0, 1) = 0;
	(*this)(0, 2) = -1;
	(*this)(0, 3) = 0;
	(*this)(0, 4) = 0;
	(*this)(0, 5) = 0;
	(*this)(1, 2) = 0;
	(*this)(1, 3) = 0;
	(*this)(1, 4) = 0;
	(*this)(1, 5) = 0;
	(*this)(2, 2) = 0;
	(*this)(2, 3) = 0;
	(*this)(2, 4) = 0;
	(*this)(2, 5) = 0;
	(*this)(3, 2) = 0;
	(*this)(3, 3) = 0;
	(*this)(3, 4) = 0;
	(*this)(3, 5) = -1;
	(*this)(4, 2) = 0;
	(*this)(4, 5) = 0;
	(*this)(5, 2) = 0.118500000000000;
	(*this)(5, 5) = 0;
}

template <typename TRAIT>
const typename iit::pegasus2::tpl::MotionTransforms<TRAIT>::Type_fr_torso_X_fr_LF_hip& 
iit::pegasus2::tpl::MotionTransforms<TRAIT>::Type_fr_torso_X_fr_LF_hip::update(const JState& q){ 
	Scalar c__q_LF_HAA_joint__;
	Scalar s__q_LF_HAA_joint__;

	c__q_LF_HAA_joint__ = TRAIT::cos( q[LF_HAA_JOINT]);
	s__q_LF_HAA_joint__ = TRAIT::sin( q[LF_HAA_JOINT]);

	(*this)(1, 0) = s__q_LF_HAA_joint__;
	(*this)(1, 1) = c__q_LF_HAA_joint__;
	(*this)(2, 0) = c__q_LF_HAA_joint__;
	(*this)(2, 1) = -s__q_LF_HAA_joint__;
	(*this)(3, 0) = 0.1185*c__q_LF_HAA_joint__;
	(*this)(3, 1) = -0.1185*s__q_LF_HAA_joint__;
	(*this)(4, 0) = -0.37*c__q_LF_HAA_joint__;
	(*this)(4, 1) = 0.37*s__q_LF_HAA_joint__;
	(*this)(4, 3) = s__q_LF_HAA_joint__;
	(*this)(4, 4) = c__q_LF_HAA_joint__;
	(*this)(5, 0) = 0.37*s__q_LF_HAA_joint__;
	(*this)(5, 1) = 0.37*c__q_LF_HAA_joint__;
	(*this)(5, 3) = c__q_LF_HAA_joint__;
	(*this)(5, 4) = -s__q_LF_HAA_joint__;
	return *this;
}

template <typename TRAIT>
iit::pegasus2::tpl::MotionTransforms<TRAIT>::Type_fr_LF_hip_X_fr_torso::Type_fr_LF_hip_X_fr_torso(){ 
	(*this)(0, 0) = 0;
	(*this)(0, 3) = 0;
	(*this)(0, 4) = 0;
	(*this)(0, 5) = 0;
	(*this)(1, 0) = 0;
	(*this)(1, 3) = 0;
	(*this)(1, 4) = 0;
	(*this)(1, 5) = 0;
	(*this)(2, 0) = -1;
	(*this)(2, 1) = 0;
	(*this)(2, 2) = 0;
	(*this)(2, 3) = 0;
	(*this)(2, 4) = 0;
	(*this)(2, 5) = 0;
	(*this)(3, 3) = 0;
	(*this)(4, 3) = 0;
	(*this)(5, 0) = 0;
	(*this)(5, 1) = 0;
	(*this)(5, 2) = 0.118500000000000;
	(*this)(5, 3) = -1;
	(*this)(5, 4) = 0;
	(*this)(5, 5) = 0;
}

template <typename TRAIT>
const typename iit::pegasus2::tpl::MotionTransforms<TRAIT>::Type_fr_LF_hip_X_fr_torso& 
iit::pegasus2::tpl::MotionTransforms<TRAIT>::Type_fr_LF_hip_X_fr_torso::update(const JState& q){ 
	Scalar c__q_LF_HAA_joint__;
	Scalar s__q_LF_HAA_joint__;

	c__q_LF_HAA_joint__ = TRAIT::cos( q[LF_HAA_JOINT]);
	s__q_LF_HAA_joint__ = TRAIT::sin( q[LF_HAA_JOINT]);

	(*this)(0, 1) = s__q_LF_HAA_joint__;
	(*this)(0, 2) = c__q_LF_HAA_joint__;
	(*this)(1, 1) = c__q_LF_HAA_joint__;
	(*this)(1, 2) = -s__q_LF_HAA_joint__;
	(*this)(3, 0) = 0.1185*c__q_LF_HAA_joint__;
	(*this)(3, 1) = -0.37*c__q_LF_HAA_joint__;
	(*this)(3, 2) = 0.37*s__q_LF_HAA_joint__;
	(*this)(3, 4) = s__q_LF_HAA_joint__;
	(*this)(3, 5) = c__q_LF_HAA_joint__;
	(*this)(4, 0) = -0.1185*s__q_LF_HAA_joint__;
	(*this)(4, 1) = 0.37*s__q_LF_HAA_joint__;
	(*this)(4, 2) = 0.37*c__q_LF_HAA_joint__;
	(*this)(4, 4) = c__q_LF_HAA_joint__;
	(*this)(4, 5) = -s__q_LF_HAA_joint__;
	return *this;
}

template <typename TRAIT>
iit::pegasus2::tpl::MotionTransforms<TRAIT>::Type_fr_torso_X_fr_LF_upperleg::Type_fr_torso_X_fr_LF_upperleg(){ 
	(*this)(0, 2) = 0;
	(*this)(0, 3) = 0;
	(*this)(0, 4) = 0;
	(*this)(0, 5) = 0;
	(*this)(1, 3) = 0;
	(*this)(1, 4) = 0;
	(*this)(1, 5) = 0;
	(*this)(2, 3) = 0;
	(*this)(2, 4) = 0;
	(*this)(2, 5) = 0;
	(*this)(3, 5) = 0;
}

template <typename TRAIT>
const typename iit::pegasus2::tpl::MotionTransforms<TRAIT>::Type_fr_torso_X_fr_LF_upperleg& 
iit::pegasus2::tpl::MotionTransforms<TRAIT>::Type_fr_torso_X_fr_LF_upperleg::update(const JState& q){ 
	Scalar c__q_LF_HFE_joint__;
	Scalar c__q_LF_HAA_joint__;
	Scalar s__q_LF_HAA_joint__;
	Scalar s__q_LF_HFE_joint__;

	c__q_LF_HFE_joint__ = TRAIT::cos( q[LF_HFE_JOINT]);
	c__q_LF_HAA_joint__ = TRAIT::cos( q[LF_HAA_JOINT]);
	s__q_LF_HAA_joint__ = TRAIT::sin( q[LF_HAA_JOINT]);
	s__q_LF_HFE_joint__ = TRAIT::sin( q[LF_HFE_JOINT]);

	(*this)(0, 0) = s__q_LF_HFE_joint__;
	(*this)(0, 1) = c__q_LF_HFE_joint__;
	(*this)(1, 0) = -s__q_LF_HAA_joint__*c__q_LF_HFE_joint__;
	(*this)(1, 1) = s__q_LF_HAA_joint__*s__q_LF_HFE_joint__;
	(*this)(1, 2) = -c__q_LF_HAA_joint__;
	(*this)(2, 0) = -c__q_LF_HAA_joint__*c__q_LF_HFE_joint__;
	(*this)(2, 1) = s__q_LF_HFE_joint__*c__q_LF_HAA_joint__;
	(*this)(2, 2) = s__q_LF_HAA_joint__;
	(*this)(3, 0) = -(0.1185*c__q_LF_HAA_joint__ + 0.05)*c__q_LF_HFE_joint__;
	(*this)(3, 1) = (0.1185*c__q_LF_HAA_joint__ + 0.05)*s__q_LF_HFE_joint__;
	(*this)(3, 2) = 0.1185*s__q_LF_HAA_joint__;
	(*this)(3, 3) = s__q_LF_HFE_joint__;
	(*this)(3, 4) = c__q_LF_HFE_joint__;
	(*this)(4, 0) = -0.05*s__q_LF_HAA_joint__*s__q_LF_HFE_joint__ + 0.37*c__q_LF_HAA_joint__*c__q_LF_HFE_joint__;
	(*this)(4, 1) = -0.05*s__q_LF_HAA_joint__*c__q_LF_HFE_joint__ - 0.37*s__q_LF_HFE_joint__*c__q_LF_HAA_joint__;
	(*this)(4, 2) = -0.37*s__q_LF_HAA_joint__;
	(*this)(4, 3) = -s__q_LF_HAA_joint__*c__q_LF_HFE_joint__;
	(*this)(4, 4) = s__q_LF_HAA_joint__*s__q_LF_HFE_joint__;
	(*this)(4, 5) = -c__q_LF_HAA_joint__;
	(*this)(5, 0) = -0.37*s__q_LF_HAA_joint__*c__q_LF_HFE_joint__ - 0.05*s__q_LF_HFE_joint__*c__q_LF_HAA_joint__ - 0.1185*s__q_LF_HFE_joint__;
	(*this)(5, 1) = 0.37*s__q_LF_HAA_joint__*s__q_LF_HFE_joint__ - 0.05*c__q_LF_HAA_joint__*c__q_LF_HFE_joint__ - 0.1185*c__q_LF_HFE_joint__;
	(*this)(5, 2) = -0.37*c__q_LF_HAA_joint__;
	(*this)(5, 3) = -c__q_LF_HAA_joint__*c__q_LF_HFE_joint__;
	(*this)(5, 4) = s__q_LF_HFE_joint__*c__q_LF_HAA_joint__;
	(*this)(5, 5) = s__q_LF_HAA_joint__;
	return *this;
}

template <typename TRAIT>
iit::pegasus2::tpl::MotionTransforms<TRAIT>::Type_fr_LF_upperleg_X_fr_torso::Type_fr_LF_upperleg_X_fr_torso(){ 
	(*this)(0, 3) = 0;
	(*this)(0, 4) = 0;
	(*this)(0, 5) = 0;
	(*this)(1, 3) = 0;
	(*this)(1, 4) = 0;
	(*this)(1, 5) = 0;
	(*this)(2, 0) = 0;
	(*this)(2, 3) = 0;
	(*this)(2, 4) = 0;
	(*this)(2, 5) = 0;
	(*this)(5, 3) = 0;
}

template <typename TRAIT>
const typename iit::pegasus2::tpl::MotionTransforms<TRAIT>::Type_fr_LF_upperleg_X_fr_torso& 
iit::pegasus2::tpl::MotionTransforms<TRAIT>::Type_fr_LF_upperleg_X_fr_torso::update(const JState& q){ 
	Scalar c__q_LF_HFE_joint__;
	Scalar c__q_LF_HAA_joint__;
	Scalar s__q_LF_HAA_joint__;
	Scalar s__q_LF_HFE_joint__;

	c__q_LF_HFE_joint__ = TRAIT::cos( q[LF_HFE_JOINT]);
	c__q_LF_HAA_joint__ = TRAIT::cos( q[LF_HAA_JOINT]);
	s__q_LF_HAA_joint__ = TRAIT::sin( q[LF_HAA_JOINT]);
	s__q_LF_HFE_joint__ = TRAIT::sin( q[LF_HFE_JOINT]);

	(*this)(0, 0) = s__q_LF_HFE_joint__;
	(*this)(0, 1) = -s__q_LF_HAA_joint__*c__q_LF_HFE_joint__;
	(*this)(0, 2) = -c__q_LF_HAA_joint__*c__q_LF_HFE_joint__;
	(*this)(1, 0) = c__q_LF_HFE_joint__;
	(*this)(1, 1) = s__q_LF_HAA_joint__*s__q_LF_HFE_joint__;
	(*this)(1, 2) = s__q_LF_HFE_joint__*c__q_LF_HAA_joint__;
	(*this)(2, 1) = -c__q_LF_HAA_joint__;
	(*this)(2, 2) = s__q_LF_HAA_joint__;
	(*this)(3, 0) = -(0.1185*c__q_LF_HAA_joint__ + 0.05)*c__q_LF_HFE_joint__;
	(*this)(3, 1) = -0.05*s__q_LF_HAA_joint__*s__q_LF_HFE_joint__ + 0.37*c__q_LF_HAA_joint__*c__q_LF_HFE_joint__;
	(*this)(3, 2) = -0.37*s__q_LF_HAA_joint__*c__q_LF_HFE_joint__ - 0.05*s__q_LF_HFE_joint__*c__q_LF_HAA_joint__ - 0.1185*s__q_LF_HFE_joint__;
	(*this)(3, 3) = s__q_LF_HFE_joint__;
	(*this)(3, 4) = -s__q_LF_HAA_joint__*c__q_LF_HFE_joint__;
	(*this)(3, 5) = -c__q_LF_HAA_joint__*c__q_LF_HFE_joint__;
	(*this)(4, 0) = (0.1185*c__q_LF_HAA_joint__ + 0.05)*s__q_LF_HFE_joint__;
	(*this)(4, 1) = -0.05*s__q_LF_HAA_joint__*c__q_LF_HFE_joint__ - 0.37*s__q_LF_HFE_joint__*c__q_LF_HAA_joint__;
	(*this)(4, 2) = 0.37*s__q_LF_HAA_joint__*s__q_LF_HFE_joint__ - 0.05*c__q_LF_HAA_joint__*c__q_LF_HFE_joint__ - 0.1185*c__q_LF_HFE_joint__;
	(*this)(4, 3) = c__q_LF_HFE_joint__;
	(*this)(4, 4) = s__q_LF_HAA_joint__*s__q_LF_HFE_joint__;
	(*this)(4, 5) = s__q_LF_HFE_joint__*c__q_LF_HAA_joint__;
	(*this)(5, 0) = 0.1185*s__q_LF_HAA_joint__;
	(*this)(5, 1) = -0.37*s__q_LF_HAA_joint__;
	(*this)(5, 2) = -0.37*c__q_LF_HAA_joint__;
	(*this)(5, 4) = -c__q_LF_HAA_joint__;
	(*this)(5, 5) = s__q_LF_HAA_joint__;
	return *this;
}

template <typename TRAIT>
iit::pegasus2::tpl::MotionTransforms<TRAIT>::Type_fr_torso_X_fr_LF_lowerleg::Type_fr_torso_X_fr_LF_lowerleg(){ 
	(*this)(0, 2) = 0;
	(*this)(0, 3) = 0;
	(*this)(0, 4) = 0;
	(*this)(0, 5) = 0;
	(*this)(1, 3) = 0;
	(*this)(1, 4) = 0;
	(*this)(1, 5) = 0;
	(*this)(2, 3) = 0;
	(*this)(2, 4) = 0;
	(*this)(2, 5) = 0;
	(*this)(3, 5) = 0;
}

template <typename TRAIT>
const typename iit::pegasus2::tpl::MotionTransforms<TRAIT>::Type_fr_torso_X_fr_LF_lowerleg& 
iit::pegasus2::tpl::MotionTransforms<TRAIT>::Type_fr_torso_X_fr_LF_lowerleg::update(const JState& q){ 
	Scalar c__q_LF_KFE_joint__;
	Scalar c__q_LF_HFE_joint__;
	Scalar c__q_LF_HAA_joint__;
	Scalar s__q_LF_HAA_joint__;
	Scalar s__q_LF_KFE_joint__;
	Scalar s__q_LF_HFE_joint__;

	c__q_LF_KFE_joint__ = TRAIT::cos( q[LF_KFE_JOINT]);
	c__q_LF_HFE_joint__ = TRAIT::cos( q[LF_HFE_JOINT]);
	c__q_LF_HAA_joint__ = TRAIT::cos( q[LF_HAA_JOINT]);
	s__q_LF_HAA_joint__ = TRAIT::sin( q[LF_HAA_JOINT]);
	s__q_LF_KFE_joint__ = TRAIT::sin( q[LF_KFE_JOINT]);
	s__q_LF_HFE_joint__ = TRAIT::sin( q[LF_HFE_JOINT]);

	(*this)(0, 0) = s__q_LF_HFE_joint__*c__q_LF_KFE_joint__ + s__q_LF_KFE_joint__*c__q_LF_HFE_joint__;
	(*this)(0, 1) = -s__q_LF_HFE_joint__*s__q_LF_KFE_joint__ + c__q_LF_HFE_joint__*c__q_LF_KFE_joint__;
	(*this)(1, 0) = -(-s__q_LF_HFE_joint__*s__q_LF_KFE_joint__ + c__q_LF_HFE_joint__*c__q_LF_KFE_joint__)*s__q_LF_HAA_joint__;
	(*this)(1, 1) = (s__q_LF_HFE_joint__*c__q_LF_KFE_joint__ + s__q_LF_KFE_joint__*c__q_LF_HFE_joint__)*s__q_LF_HAA_joint__;
	(*this)(1, 2) = -c__q_LF_HAA_joint__;
	(*this)(2, 0) = -(-s__q_LF_HFE_joint__*s__q_LF_KFE_joint__ + c__q_LF_HFE_joint__*c__q_LF_KFE_joint__)*c__q_LF_HAA_joint__;
	(*this)(2, 1) = (s__q_LF_HFE_joint__*c__q_LF_KFE_joint__ + s__q_LF_KFE_joint__*c__q_LF_HFE_joint__)*c__q_LF_HAA_joint__;
	(*this)(2, 2) = s__q_LF_HAA_joint__;
	(*this)(3, 0) = -(-s__q_LF_HFE_joint__*s__q_LF_KFE_joint__ + c__q_LF_HFE_joint__*c__q_LF_KFE_joint__)*(0.1185*c__q_LF_HAA_joint__ + 0.05);
	(*this)(3, 1) = (s__q_LF_HFE_joint__*c__q_LF_KFE_joint__ + s__q_LF_KFE_joint__*c__q_LF_HFE_joint__)*(0.1185*c__q_LF_HAA_joint__ + 0.05);
	(*this)(3, 2) = 0.1185*s__q_LF_HAA_joint__ - 0.38*c__q_LF_HFE_joint__;
	(*this)(3, 3) = s__q_LF_HFE_joint__*c__q_LF_KFE_joint__ + s__q_LF_KFE_joint__*c__q_LF_HFE_joint__;
	(*this)(3, 4) = -s__q_LF_HFE_joint__*s__q_LF_KFE_joint__ + c__q_LF_HFE_joint__*c__q_LF_KFE_joint__;
	(*this)(4, 0) = 0.16*(s__q_LF_HAA_joint__*s__q_LF_KFE_joint__ + c__q_LF_HAA_joint__*c__q_LF_KFE_joint__)*c__q_LF_HFE_joint__ - 0.16*(-s__q_LF_HAA_joint__*c__q_LF_KFE_joint__ + s__q_LF_KFE_joint__*c__q_LF_HAA_joint__)*s__q_LF_HFE_joint__ + 0.21*(-s__q_LF_HFE_joint__*s__q_LF_KFE_joint__ + c__q_LF_HFE_joint__*c__q_LF_KFE_joint__)*c__q_LF_HAA_joint__ - 0.21*(s__q_LF_HFE_joint__*c__q_LF_KFE_joint__ + s__q_LF_KFE_joint__*c__q_LF_HFE_joint__)*s__q_LF_HAA_joint__ - 0.38*s__q_LF_KFE_joint__*c__q_LF_HAA_joint__;
	(*this)(4, 1) = -0.16*(s__q_LF_HAA_joint__*s__q_LF_KFE_joint__ + c__q_LF_HAA_joint__*c__q_LF_KFE_joint__)*s__q_LF_HFE_joint__ - 0.16*(-s__q_LF_HAA_joint__*c__q_LF_KFE_joint__ + s__q_LF_KFE_joint__*c__q_LF_HAA_joint__)*c__q_LF_HFE_joint__ - 0.21*(-s__q_LF_HFE_joint__*s__q_LF_KFE_joint__ + c__q_LF_HFE_joint__*c__q_LF_KFE_joint__)*s__q_LF_HAA_joint__ - 0.21*(s__q_LF_HFE_joint__*c__q_LF_KFE_joint__ + s__q_LF_KFE_joint__*c__q_LF_HFE_joint__)*c__q_LF_HAA_joint__ - 0.38*c__q_LF_HAA_joint__*c__q_LF_KFE_joint__;
	(*this)(4, 2) = -(0.38*s__q_LF_HFE_joint__ + 0.37)*s__q_LF_HAA_joint__;
	(*this)(4, 3) = -(-s__q_LF_HFE_joint__*s__q_LF_KFE_joint__ + c__q_LF_HFE_joint__*c__q_LF_KFE_joint__)*s__q_LF_HAA_joint__;
	(*this)(4, 4) = (s__q_LF_HFE_joint__*c__q_LF_KFE_joint__ + s__q_LF_KFE_joint__*c__q_LF_HFE_joint__)*s__q_LF_HAA_joint__;
	(*this)(4, 5) = -c__q_LF_HAA_joint__;
	(*this)(5, 0) = -0.37*(-s__q_LF_HFE_joint__*s__q_LF_KFE_joint__ + c__q_LF_HFE_joint__*c__q_LF_KFE_joint__)*s__q_LF_HAA_joint__ - 0.05*(s__q_LF_HFE_joint__*c__q_LF_KFE_joint__ + s__q_LF_KFE_joint__*c__q_LF_HFE_joint__)*c__q_LF_HAA_joint__ + 0.38*s__q_LF_HAA_joint__*s__q_LF_KFE_joint__ - 0.1185*s__q_LF_HFE_joint__*c__q_LF_KFE_joint__ - 0.1185*s__q_LF_KFE_joint__*c__q_LF_HFE_joint__;
	(*this)(5, 1) = -0.05*(-s__q_LF_HFE_joint__*s__q_LF_KFE_joint__ + c__q_LF_HFE_joint__*c__q_LF_KFE_joint__)*c__q_LF_HAA_joint__ + 0.37*(s__q_LF_HFE_joint__*c__q_LF_KFE_joint__ + s__q_LF_KFE_joint__*c__q_LF_HFE_joint__)*s__q_LF_HAA_joint__ + 0.38*s__q_LF_HAA_joint__*c__q_LF_KFE_joint__ + 0.1185*s__q_LF_HFE_joint__*s__q_LF_KFE_joint__ - 0.1185*c__q_LF_HFE_joint__*c__q_LF_KFE_joint__;
	(*this)(5, 2) = -(0.38*s__q_LF_HFE_joint__ + 0.37)*c__q_LF_HAA_joint__;
	(*this)(5, 3) = -(-s__q_LF_HFE_joint__*s__q_LF_KFE_joint__ + c__q_LF_HFE_joint__*c__q_LF_KFE_joint__)*c__q_LF_HAA_joint__;
	(*this)(5, 4) = (s__q_LF_HFE_joint__*c__q_LF_KFE_joint__ + s__q_LF_KFE_joint__*c__q_LF_HFE_joint__)*c__q_LF_HAA_joint__;
	(*this)(5, 5) = s__q_LF_HAA_joint__;
	return *this;
}

template <typename TRAIT>
iit::pegasus2::tpl::MotionTransforms<TRAIT>::Type_fr_LF_lowerleg_X_fr_torso::Type_fr_LF_lowerleg_X_fr_torso(){ 
	(*this)(0, 3) = 0;
	(*this)(0, 4) = 0;
	(*this)(0, 5) = 0;
	(*this)(1, 3) = 0;
	(*this)(1, 4) = 0;
	(*this)(1, 5) = 0;
	(*this)(2, 0) = 0;
	(*this)(2, 3) = 0;
	(*this)(2, 4) = 0;
	(*this)(2, 5) = 0;
	(*this)(5, 3) = 0;
}

template <typename TRAIT>
const typename iit::pegasus2::tpl::MotionTransforms<TRAIT>::Type_fr_LF_lowerleg_X_fr_torso& 
iit::pegasus2::tpl::MotionTransforms<TRAIT>::Type_fr_LF_lowerleg_X_fr_torso::update(const JState& q){ 
	Scalar c__q_LF_KFE_joint__;
	Scalar c__q_LF_HFE_joint__;
	Scalar c__q_LF_HAA_joint__;
	Scalar s__q_LF_HAA_joint__;
	Scalar s__q_LF_KFE_joint__;
	Scalar s__q_LF_HFE_joint__;

	c__q_LF_KFE_joint__ = TRAIT::cos( q[LF_KFE_JOINT]);
	c__q_LF_HFE_joint__ = TRAIT::cos( q[LF_HFE_JOINT]);
	c__q_LF_HAA_joint__ = TRAIT::cos( q[LF_HAA_JOINT]);
	s__q_LF_HAA_joint__ = TRAIT::sin( q[LF_HAA_JOINT]);
	s__q_LF_KFE_joint__ = TRAIT::sin( q[LF_KFE_JOINT]);
	s__q_LF_HFE_joint__ = TRAIT::sin( q[LF_HFE_JOINT]);

	(*this)(0, 0) = s__q_LF_HFE_joint__*c__q_LF_KFE_joint__ + s__q_LF_KFE_joint__*c__q_LF_HFE_joint__;
	(*this)(0, 1) = -(-s__q_LF_HFE_joint__*s__q_LF_KFE_joint__ + c__q_LF_HFE_joint__*c__q_LF_KFE_joint__)*s__q_LF_HAA_joint__;
	(*this)(0, 2) = -(-s__q_LF_HFE_joint__*s__q_LF_KFE_joint__ + c__q_LF_HFE_joint__*c__q_LF_KFE_joint__)*c__q_LF_HAA_joint__;
	(*this)(1, 0) = -s__q_LF_HFE_joint__*s__q_LF_KFE_joint__ + c__q_LF_HFE_joint__*c__q_LF_KFE_joint__;
	(*this)(1, 1) = (s__q_LF_HFE_joint__*c__q_LF_KFE_joint__ + s__q_LF_KFE_joint__*c__q_LF_HFE_joint__)*s__q_LF_HAA_joint__;
	(*this)(1, 2) = (s__q_LF_HFE_joint__*c__q_LF_KFE_joint__ + s__q_LF_KFE_joint__*c__q_LF_HFE_joint__)*c__q_LF_HAA_joint__;
	(*this)(2, 1) = -c__q_LF_HAA_joint__;
	(*this)(2, 2) = s__q_LF_HAA_joint__;
	(*this)(3, 0) = -(-s__q_LF_HFE_joint__*s__q_LF_KFE_joint__ + c__q_LF_HFE_joint__*c__q_LF_KFE_joint__)*(0.1185*c__q_LF_HAA_joint__ + 0.05);
	(*this)(3, 1) = 0.37*(-s__q_LF_HFE_joint__*s__q_LF_KFE_joint__ + c__q_LF_HFE_joint__*c__q_LF_KFE_joint__)*c__q_LF_HAA_joint__ - 0.05*(s__q_LF_HFE_joint__*c__q_LF_KFE_joint__ + s__q_LF_KFE_joint__*c__q_LF_HFE_joint__)*s__q_LF_HAA_joint__ - 0.38*s__q_LF_KFE_joint__*c__q_LF_HAA_joint__;
	(*this)(3, 2) = -0.37*(-s__q_LF_HFE_joint__*s__q_LF_KFE_joint__ + c__q_LF_HFE_joint__*c__q_LF_KFE_joint__)*s__q_LF_HAA_joint__ - 0.05*(s__q_LF_HFE_joint__*c__q_LF_KFE_joint__ + s__q_LF_KFE_joint__*c__q_LF_HFE_joint__)*c__q_LF_HAA_joint__ + 0.38*s__q_LF_HAA_joint__*s__q_LF_KFE_joint__ - 0.1185*s__q_LF_HFE_joint__*c__q_LF_KFE_joint__ - 0.1185*s__q_LF_KFE_joint__*c__q_LF_HFE_joint__;
	(*this)(3, 3) = s__q_LF_HFE_joint__*c__q_LF_KFE_joint__ + s__q_LF_KFE_joint__*c__q_LF_HFE_joint__;
	(*this)(3, 4) = -(-s__q_LF_HFE_joint__*s__q_LF_KFE_joint__ + c__q_LF_HFE_joint__*c__q_LF_KFE_joint__)*s__q_LF_HAA_joint__;
	(*this)(3, 5) = -(-s__q_LF_HFE_joint__*s__q_LF_KFE_joint__ + c__q_LF_HFE_joint__*c__q_LF_KFE_joint__)*c__q_LF_HAA_joint__;
	(*this)(4, 0) = (s__q_LF_HFE_joint__*c__q_LF_KFE_joint__ + s__q_LF_KFE_joint__*c__q_LF_HFE_joint__)*(0.1185*c__q_LF_HAA_joint__ + 0.05);
	(*this)(4, 1) = -0.05*(-s__q_LF_HFE_joint__*s__q_LF_KFE_joint__ + c__q_LF_HFE_joint__*c__q_LF_KFE_joint__)*s__q_LF_HAA_joint__ - 0.37*(s__q_LF_HFE_joint__*c__q_LF_KFE_joint__ + s__q_LF_KFE_joint__*c__q_LF_HFE_joint__)*c__q_LF_HAA_joint__ - 0.38*c__q_LF_HAA_joint__*c__q_LF_KFE_joint__;
	(*this)(4, 2) = -0.05*(-s__q_LF_HFE_joint__*s__q_LF_KFE_joint__ + c__q_LF_HFE_joint__*c__q_LF_KFE_joint__)*c__q_LF_HAA_joint__ + 0.37*(s__q_LF_HFE_joint__*c__q_LF_KFE_joint__ + s__q_LF_KFE_joint__*c__q_LF_HFE_joint__)*s__q_LF_HAA_joint__ + 0.38*s__q_LF_HAA_joint__*c__q_LF_KFE_joint__ + 0.1185*s__q_LF_HFE_joint__*s__q_LF_KFE_joint__ - 0.1185*c__q_LF_HFE_joint__*c__q_LF_KFE_joint__;
	(*this)(4, 3) = -s__q_LF_HFE_joint__*s__q_LF_KFE_joint__ + c__q_LF_HFE_joint__*c__q_LF_KFE_joint__;
	(*this)(4, 4) = (s__q_LF_HFE_joint__*c__q_LF_KFE_joint__ + s__q_LF_KFE_joint__*c__q_LF_HFE_joint__)*s__q_LF_HAA_joint__;
	(*this)(4, 5) = (s__q_LF_HFE_joint__*c__q_LF_KFE_joint__ + s__q_LF_KFE_joint__*c__q_LF_HFE_joint__)*c__q_LF_HAA_joint__;
	(*this)(5, 0) = 0.1185*s__q_LF_HAA_joint__ - 0.38*c__q_LF_HFE_joint__;
	(*this)(5, 1) = -(0.38*s__q_LF_HFE_joint__ + 0.37)*s__q_LF_HAA_joint__;
	(*this)(5, 2) = -(0.38*s__q_LF_HFE_joint__ + 0.37)*c__q_LF_HAA_joint__;
	(*this)(5, 4) = -c__q_LF_HAA_joint__;
	(*this)(5, 5) = s__q_LF_HAA_joint__;
	return *this;
}

template <typename TRAIT>
iit::pegasus2::tpl::MotionTransforms<TRAIT>::Type_fr_torso_X_fr_RF_foot::Type_fr_torso_X_fr_RF_foot(){ 
	(*this)(0, 2) = 0;
	(*this)(0, 3) = 0;
	(*this)(0, 4) = 0;
	(*this)(0, 5) = 0;
	(*this)(1, 3) = 0;
	(*this)(1, 4) = 0;
	(*this)(1, 5) = 0;
	(*this)(2, 3) = 0;
	(*this)(2, 4) = 0;
	(*this)(2, 5) = 0;
	(*this)(3, 5) = 0;
}

template <typename TRAIT>
const typename iit::pegasus2::tpl::MotionTransforms<TRAIT>::Type_fr_torso_X_fr_RF_foot& 
iit::pegasus2::tpl::MotionTransforms<TRAIT>::Type_fr_torso_X_fr_RF_foot::update(const JState& q){ 
	Scalar c__q_RF_KFE_joint__;
	Scalar c__q_RF_HAA_joint__;
	Scalar c__q_RF_HFE_joint__;
	Scalar s__q_RF_HFE_joint__;
	Scalar s__q_RF_HAA_joint__;
	Scalar s__q_RF_KFE_joint__;

	c__q_RF_KFE_joint__ = TRAIT::cos( q[RF_KFE_JOINT]);
	c__q_RF_HAA_joint__ = TRAIT::cos( q[RF_HAA_JOINT]);
	c__q_RF_HFE_joint__ = TRAIT::cos( q[RF_HFE_JOINT]);
	s__q_RF_HFE_joint__ = TRAIT::sin( q[RF_HFE_JOINT]);
	s__q_RF_HAA_joint__ = TRAIT::sin( q[RF_HAA_JOINT]);
	s__q_RF_KFE_joint__ = TRAIT::sin( q[RF_KFE_JOINT]);

	(*this)(0, 0) = s__q_RF_HFE_joint__*c__q_RF_KFE_joint__ + s__q_RF_KFE_joint__*c__q_RF_HFE_joint__;
	(*this)(0, 1) = -s__q_RF_HFE_joint__*s__q_RF_KFE_joint__ + c__q_RF_HFE_joint__*c__q_RF_KFE_joint__;
	(*this)(1, 0) = -(-s__q_RF_HFE_joint__*s__q_RF_KFE_joint__ + c__q_RF_HFE_joint__*c__q_RF_KFE_joint__)*s__q_RF_HAA_joint__;
	(*this)(1, 1) = (s__q_RF_HFE_joint__*c__q_RF_KFE_joint__ + s__q_RF_KFE_joint__*c__q_RF_HFE_joint__)*s__q_RF_HAA_joint__;
	(*this)(1, 2) = -c__q_RF_HAA_joint__;
	(*this)(2, 0) = -(-s__q_RF_HFE_joint__*s__q_RF_KFE_joint__ + c__q_RF_HFE_joint__*c__q_RF_KFE_joint__)*c__q_RF_HAA_joint__;
	(*this)(2, 1) = (s__q_RF_HFE_joint__*c__q_RF_KFE_joint__ + s__q_RF_KFE_joint__*c__q_RF_HFE_joint__)*c__q_RF_HAA_joint__;
	(*this)(2, 2) = s__q_RF_HAA_joint__;
	(*this)(3, 0) = (-s__q_RF_HFE_joint__*s__q_RF_KFE_joint__ + c__q_RF_HFE_joint__*c__q_RF_KFE_joint__)*(0.1185*c__q_RF_HAA_joint__ + 0.05);
	(*this)(3, 1) = -(s__q_RF_HFE_joint__*c__q_RF_KFE_joint__ + s__q_RF_KFE_joint__*c__q_RF_HFE_joint__)*(0.1185*c__q_RF_HAA_joint__ + 0.05);
	(*this)(3, 2) = -0.1185*s__q_RF_HAA_joint__ + 0.3612*s__q_RF_HFE_joint__*s__q_RF_KFE_joint__ - 0.3612*c__q_RF_HFE_joint__*c__q_RF_KFE_joint__ - 0.38*c__q_RF_HFE_joint__;
	(*this)(3, 3) = s__q_RF_HFE_joint__*c__q_RF_KFE_joint__ + s__q_RF_KFE_joint__*c__q_RF_HFE_joint__;
	(*this)(3, 4) = -s__q_RF_HFE_joint__*s__q_RF_KFE_joint__ + c__q_RF_HFE_joint__*c__q_RF_KFE_joint__;
	(*this)(4, 0) = 0.21*(s__q_RF_HAA_joint__*s__q_RF_KFE_joint__ + c__q_RF_HAA_joint__*c__q_RF_KFE_joint__)*c__q_RF_HFE_joint__ - 0.21*(-s__q_RF_HAA_joint__*c__q_RF_KFE_joint__ + s__q_RF_KFE_joint__*c__q_RF_HAA_joint__)*s__q_RF_HFE_joint__ + 0.16*(-s__q_RF_HFE_joint__*s__q_RF_KFE_joint__ + c__q_RF_HFE_joint__*c__q_RF_KFE_joint__)*c__q_RF_HAA_joint__ - 0.16*(s__q_RF_HFE_joint__*c__q_RF_KFE_joint__ + s__q_RF_KFE_joint__*c__q_RF_HFE_joint__)*s__q_RF_HAA_joint__ - 0.38*s__q_RF_KFE_joint__*c__q_RF_HAA_joint__;
	(*this)(4, 1) = -0.21*(s__q_RF_HAA_joint__*s__q_RF_KFE_joint__ + c__q_RF_HAA_joint__*c__q_RF_KFE_joint__)*s__q_RF_HFE_joint__ - 0.21*(-s__q_RF_HAA_joint__*c__q_RF_KFE_joint__ + s__q_RF_KFE_joint__*c__q_RF_HAA_joint__)*c__q_RF_HFE_joint__ - 0.16*(-s__q_RF_HFE_joint__*s__q_RF_KFE_joint__ + c__q_RF_HFE_joint__*c__q_RF_KFE_joint__)*s__q_RF_HAA_joint__ - 0.16*(s__q_RF_HFE_joint__*c__q_RF_KFE_joint__ + s__q_RF_KFE_joint__*c__q_RF_HFE_joint__)*c__q_RF_HAA_joint__ - 0.38*c__q_RF_HAA_joint__*c__q_RF_KFE_joint__ - 0.3612*c__q_RF_HAA_joint__;
	(*this)(4, 2) = -(0.3612*s__q_RF_HFE_joint__*c__q_RF_KFE_joint__ + 0.38*s__q_RF_HFE_joint__ + 0.3612*s__q_RF_KFE_joint__*c__q_RF_HFE_joint__ + 0.37)*s__q_RF_HAA_joint__;
	(*this)(4, 3) = -(-s__q_RF_HFE_joint__*s__q_RF_KFE_joint__ + c__q_RF_HFE_joint__*c__q_RF_KFE_joint__)*s__q_RF_HAA_joint__;
	(*this)(4, 4) = (s__q_RF_HFE_joint__*c__q_RF_KFE_joint__ + s__q_RF_KFE_joint__*c__q_RF_HFE_joint__)*s__q_RF_HAA_joint__;
	(*this)(4, 5) = -c__q_RF_HAA_joint__;
	(*this)(5, 0) = -0.37*(-s__q_RF_HFE_joint__*s__q_RF_KFE_joint__ + c__q_RF_HFE_joint__*c__q_RF_KFE_joint__)*s__q_RF_HAA_joint__ + 0.05*(s__q_RF_HFE_joint__*c__q_RF_KFE_joint__ + s__q_RF_KFE_joint__*c__q_RF_HFE_joint__)*c__q_RF_HAA_joint__ + 0.38*s__q_RF_HAA_joint__*s__q_RF_KFE_joint__ + 0.1185*s__q_RF_HFE_joint__*c__q_RF_KFE_joint__ + 0.1185*s__q_RF_KFE_joint__*c__q_RF_HFE_joint__;
	(*this)(5, 1) = 0.05*(-s__q_RF_HFE_joint__*s__q_RF_KFE_joint__ + c__q_RF_HFE_joint__*c__q_RF_KFE_joint__)*c__q_RF_HAA_joint__ + 0.37*(s__q_RF_HFE_joint__*c__q_RF_KFE_joint__ + s__q_RF_KFE_joint__*c__q_RF_HFE_joint__)*s__q_RF_HAA_joint__ + 0.38*s__q_RF_HAA_joint__*c__q_RF_KFE_joint__ + 0.3612*s__q_RF_HAA_joint__ - 0.1185*s__q_RF_HFE_joint__*s__q_RF_KFE_joint__ + 0.1185*c__q_RF_HFE_joint__*c__q_RF_KFE_joint__;
	(*this)(5, 2) = -(0.3612*s__q_RF_HFE_joint__*c__q_RF_KFE_joint__ + 0.38*s__q_RF_HFE_joint__ + 0.3612*s__q_RF_KFE_joint__*c__q_RF_HFE_joint__ + 0.37)*c__q_RF_HAA_joint__;
	(*this)(5, 3) = -(-s__q_RF_HFE_joint__*s__q_RF_KFE_joint__ + c__q_RF_HFE_joint__*c__q_RF_KFE_joint__)*c__q_RF_HAA_joint__;
	(*this)(5, 4) = (s__q_RF_HFE_joint__*c__q_RF_KFE_joint__ + s__q_RF_KFE_joint__*c__q_RF_HFE_joint__)*c__q_RF_HAA_joint__;
	(*this)(5, 5) = s__q_RF_HAA_joint__;
	return *this;
}

template <typename TRAIT>
iit::pegasus2::tpl::MotionTransforms<TRAIT>::Type_fr_RF_foot_X_fr_torso::Type_fr_RF_foot_X_fr_torso(){ 
	(*this)(0, 3) = 0;
	(*this)(0, 4) = 0;
	(*this)(0, 5) = 0;
	(*this)(1, 3) = 0;
	(*this)(1, 4) = 0;
	(*this)(1, 5) = 0;
	(*this)(2, 0) = 0;
	(*this)(2, 3) = 0;
	(*this)(2, 4) = 0;
	(*this)(2, 5) = 0;
	(*this)(5, 3) = 0;
}

template <typename TRAIT>
const typename iit::pegasus2::tpl::MotionTransforms<TRAIT>::Type_fr_RF_foot_X_fr_torso& 
iit::pegasus2::tpl::MotionTransforms<TRAIT>::Type_fr_RF_foot_X_fr_torso::update(const JState& q){ 
	Scalar c__q_RF_KFE_joint__;
	Scalar c__q_RF_HAA_joint__;
	Scalar c__q_RF_HFE_joint__;
	Scalar s__q_RF_HFE_joint__;
	Scalar s__q_RF_HAA_joint__;
	Scalar s__q_RF_KFE_joint__;

	c__q_RF_KFE_joint__ = TRAIT::cos( q[RF_KFE_JOINT]);
	c__q_RF_HAA_joint__ = TRAIT::cos( q[RF_HAA_JOINT]);
	c__q_RF_HFE_joint__ = TRAIT::cos( q[RF_HFE_JOINT]);
	s__q_RF_HFE_joint__ = TRAIT::sin( q[RF_HFE_JOINT]);
	s__q_RF_HAA_joint__ = TRAIT::sin( q[RF_HAA_JOINT]);
	s__q_RF_KFE_joint__ = TRAIT::sin( q[RF_KFE_JOINT]);

	(*this)(0, 0) = s__q_RF_HFE_joint__*c__q_RF_KFE_joint__ + s__q_RF_KFE_joint__*c__q_RF_HFE_joint__;
	(*this)(0, 1) = -(-s__q_RF_HFE_joint__*s__q_RF_KFE_joint__ + c__q_RF_HFE_joint__*c__q_RF_KFE_joint__)*s__q_RF_HAA_joint__;
	(*this)(0, 2) = -(-s__q_RF_HFE_joint__*s__q_RF_KFE_joint__ + c__q_RF_HFE_joint__*c__q_RF_KFE_joint__)*c__q_RF_HAA_joint__;
	(*this)(1, 0) = -s__q_RF_HFE_joint__*s__q_RF_KFE_joint__ + c__q_RF_HFE_joint__*c__q_RF_KFE_joint__;
	(*this)(1, 1) = (s__q_RF_HFE_joint__*c__q_RF_KFE_joint__ + s__q_RF_KFE_joint__*c__q_RF_HFE_joint__)*s__q_RF_HAA_joint__;
	(*this)(1, 2) = (s__q_RF_HFE_joint__*c__q_RF_KFE_joint__ + s__q_RF_KFE_joint__*c__q_RF_HFE_joint__)*c__q_RF_HAA_joint__;
	(*this)(2, 1) = -c__q_RF_HAA_joint__;
	(*this)(2, 2) = s__q_RF_HAA_joint__;
	(*this)(3, 0) = (-s__q_RF_HFE_joint__*s__q_RF_KFE_joint__ + c__q_RF_HFE_joint__*c__q_RF_KFE_joint__)*(0.1185*c__q_RF_HAA_joint__ + 0.05);
	(*this)(3, 1) = 0.37*(-s__q_RF_HFE_joint__*s__q_RF_KFE_joint__ + c__q_RF_HFE_joint__*c__q_RF_KFE_joint__)*c__q_RF_HAA_joint__ + 0.05*(s__q_RF_HFE_joint__*c__q_RF_KFE_joint__ + s__q_RF_KFE_joint__*c__q_RF_HFE_joint__)*s__q_RF_HAA_joint__ - 0.38*s__q_RF_KFE_joint__*c__q_RF_HAA_joint__;
	(*this)(3, 2) = -0.37*(-s__q_RF_HFE_joint__*s__q_RF_KFE_joint__ + c__q_RF_HFE_joint__*c__q_RF_KFE_joint__)*s__q_RF_HAA_joint__ + 0.05*(s__q_RF_HFE_joint__*c__q_RF_KFE_joint__ + s__q_RF_KFE_joint__*c__q_RF_HFE_joint__)*c__q_RF_HAA_joint__ + 0.38*s__q_RF_HAA_joint__*s__q_RF_KFE_joint__ + 0.1185*s__q_RF_HFE_joint__*c__q_RF_KFE_joint__ + 0.1185*s__q_RF_KFE_joint__*c__q_RF_HFE_joint__;
	(*this)(3, 3) = s__q_RF_HFE_joint__*c__q_RF_KFE_joint__ + s__q_RF_KFE_joint__*c__q_RF_HFE_joint__;
	(*this)(3, 4) = -(-s__q_RF_HFE_joint__*s__q_RF_KFE_joint__ + c__q_RF_HFE_joint__*c__q_RF_KFE_joint__)*s__q_RF_HAA_joint__;
	(*this)(3, 5) = -(-s__q_RF_HFE_joint__*s__q_RF_KFE_joint__ + c__q_RF_HFE_joint__*c__q_RF_KFE_joint__)*c__q_RF_HAA_joint__;
	(*this)(4, 0) = -(s__q_RF_HFE_joint__*c__q_RF_KFE_joint__ + s__q_RF_KFE_joint__*c__q_RF_HFE_joint__)*(0.1185*c__q_RF_HAA_joint__ + 0.05);
	(*this)(4, 1) = 0.05*(-s__q_RF_HFE_joint__*s__q_RF_KFE_joint__ + c__q_RF_HFE_joint__*c__q_RF_KFE_joint__)*s__q_RF_HAA_joint__ - 0.37*(s__q_RF_HFE_joint__*c__q_RF_KFE_joint__ + s__q_RF_KFE_joint__*c__q_RF_HFE_joint__)*c__q_RF_HAA_joint__ - (0.38*c__q_RF_KFE_joint__ - 0.3612)*c__q_RF_HAA_joint__;
	(*this)(4, 2) = 0.05*(-s__q_RF_HFE_joint__*s__q_RF_KFE_joint__ + c__q_RF_HFE_joint__*c__q_RF_KFE_joint__)*c__q_RF_HAA_joint__ + 0.37*(s__q_RF_HFE_joint__*c__q_RF_KFE_joint__ + s__q_RF_KFE_joint__*c__q_RF_HFE_joint__)*s__q_RF_HAA_joint__ + (0.38*c__q_RF_KFE_joint__ - 0.3612)*s__q_RF_HAA_joint__ - 0.1185*s__q_RF_HFE_joint__*s__q_RF_KFE_joint__ + 0.1185*c__q_RF_HFE_joint__*c__q_RF_KFE_joint__;
	(*this)(4, 3) = -s__q_RF_HFE_joint__*s__q_RF_KFE_joint__ + c__q_RF_HFE_joint__*c__q_RF_KFE_joint__;
	(*this)(4, 4) = (s__q_RF_HFE_joint__*c__q_RF_KFE_joint__ + s__q_RF_KFE_joint__*c__q_RF_HFE_joint__)*s__q_RF_HAA_joint__;
	(*this)(4, 5) = (s__q_RF_HFE_joint__*c__q_RF_KFE_joint__ + s__q_RF_KFE_joint__*c__q_RF_HFE_joint__)*c__q_RF_HAA_joint__;
	(*this)(5, 0) = -0.1185*s__q_RF_HAA_joint__ - 0.3612*s__q_RF_HFE_joint__*s__q_RF_KFE_joint__ + 0.3612*c__q_RF_HFE_joint__*c__q_RF_KFE_joint__ - 0.38*c__q_RF_HFE_joint__;
	(*this)(5, 1) = (0.3612*s__q_RF_HFE_joint__*c__q_RF_KFE_joint__ - 0.38*s__q_RF_HFE_joint__ + 0.3612*s__q_RF_KFE_joint__*c__q_RF_HFE_joint__ - 0.37)*s__q_RF_HAA_joint__;
	(*this)(5, 2) = (0.3612*s__q_RF_HFE_joint__*c__q_RF_KFE_joint__ - 0.38*s__q_RF_HFE_joint__ + 0.3612*s__q_RF_KFE_joint__*c__q_RF_HFE_joint__ - 0.37)*c__q_RF_HAA_joint__;
	(*this)(5, 4) = -c__q_RF_HAA_joint__;
	(*this)(5, 5) = s__q_RF_HAA_joint__;
	return *this;
}

template <typename TRAIT>
iit::pegasus2::tpl::MotionTransforms<TRAIT>::Type_fr_torso_X_fr_RH_foot::Type_fr_torso_X_fr_RH_foot(){ 
	(*this)(0, 2) = 0;
	(*this)(0, 3) = 0;
	(*this)(0, 4) = 0;
	(*this)(0, 5) = 0;
	(*this)(1, 3) = 0;
	(*this)(1, 4) = 0;
	(*this)(1, 5) = 0;
	(*this)(2, 3) = 0;
	(*this)(2, 4) = 0;
	(*this)(2, 5) = 0;
	(*this)(3, 5) = 0;
}

template <typename TRAIT>
const typename iit::pegasus2::tpl::MotionTransforms<TRAIT>::Type_fr_torso_X_fr_RH_foot& 
iit::pegasus2::tpl::MotionTransforms<TRAIT>::Type_fr_torso_X_fr_RH_foot::update(const JState& q){ 
	Scalar c__q_RH_HFE_joint__;
	Scalar c__q_RH_HAA_joint__;
	Scalar c__q_RH_KFE_joint__;
	Scalar s__q_RH_KFE_joint__;
	Scalar s__q_RH_HAA_joint__;
	Scalar s__q_RH_HFE_joint__;

	c__q_RH_HFE_joint__ = TRAIT::cos( q[RH_HFE_JOINT]);
	c__q_RH_HAA_joint__ = TRAIT::cos( q[RH_HAA_JOINT]);
	c__q_RH_KFE_joint__ = TRAIT::cos( q[RH_KFE_JOINT]);
	s__q_RH_KFE_joint__ = TRAIT::sin( q[RH_KFE_JOINT]);
	s__q_RH_HAA_joint__ = TRAIT::sin( q[RH_HAA_JOINT]);
	s__q_RH_HFE_joint__ = TRAIT::sin( q[RH_HFE_JOINT]);

	(*this)(0, 0) = s__q_RH_HFE_joint__*c__q_RH_KFE_joint__ + s__q_RH_KFE_joint__*c__q_RH_HFE_joint__;
	(*this)(0, 1) = -s__q_RH_HFE_joint__*s__q_RH_KFE_joint__ + c__q_RH_HFE_joint__*c__q_RH_KFE_joint__;
	(*this)(1, 0) = -(-s__q_RH_HFE_joint__*s__q_RH_KFE_joint__ + c__q_RH_HFE_joint__*c__q_RH_KFE_joint__)*s__q_RH_HAA_joint__;
	(*this)(1, 1) = (s__q_RH_HFE_joint__*c__q_RH_KFE_joint__ + s__q_RH_KFE_joint__*c__q_RH_HFE_joint__)*s__q_RH_HAA_joint__;
	(*this)(1, 2) = -c__q_RH_HAA_joint__;
	(*this)(2, 0) = -(-s__q_RH_HFE_joint__*s__q_RH_KFE_joint__ + c__q_RH_HFE_joint__*c__q_RH_KFE_joint__)*c__q_RH_HAA_joint__;
	(*this)(2, 1) = (s__q_RH_HFE_joint__*c__q_RH_KFE_joint__ + s__q_RH_KFE_joint__*c__q_RH_HFE_joint__)*c__q_RH_HAA_joint__;
	(*this)(2, 2) = s__q_RH_HAA_joint__;
	(*this)(3, 0) = (-s__q_RH_HFE_joint__*s__q_RH_KFE_joint__ + c__q_RH_HFE_joint__*c__q_RH_KFE_joint__)*(0.1185*c__q_RH_HAA_joint__ + 0.05);
	(*this)(3, 1) = -(s__q_RH_HFE_joint__*c__q_RH_KFE_joint__ + s__q_RH_KFE_joint__*c__q_RH_HFE_joint__)*(0.1185*c__q_RH_HAA_joint__ + 0.05);
	(*this)(3, 2) = -0.1185*s__q_RH_HAA_joint__ + 0.3612*s__q_RH_HFE_joint__*s__q_RH_KFE_joint__ - 0.3612*c__q_RH_HFE_joint__*c__q_RH_KFE_joint__ - 0.38*c__q_RH_HFE_joint__;
	(*this)(3, 3) = s__q_RH_HFE_joint__*c__q_RH_KFE_joint__ + s__q_RH_KFE_joint__*c__q_RH_HFE_joint__;
	(*this)(3, 4) = -s__q_RH_HFE_joint__*s__q_RH_KFE_joint__ + c__q_RH_HFE_joint__*c__q_RH_KFE_joint__;
	(*this)(4, 0) = -0.37*(-s__q_RH_HFE_joint__*s__q_RH_KFE_joint__ + c__q_RH_HFE_joint__*c__q_RH_KFE_joint__)*c__q_RH_HAA_joint__ + 0.05*(s__q_RH_HFE_joint__*c__q_RH_KFE_joint__ + s__q_RH_KFE_joint__*c__q_RH_HFE_joint__)*s__q_RH_HAA_joint__ - 0.38*s__q_RH_KFE_joint__*c__q_RH_HAA_joint__;
	(*this)(4, 1) = 0.16*(s__q_RH_HAA_joint__*s__q_RH_KFE_joint__ + c__q_RH_HAA_joint__*c__q_RH_KFE_joint__)*s__q_RH_HFE_joint__ + 0.16*(-s__q_RH_HAA_joint__*c__q_RH_KFE_joint__ + s__q_RH_KFE_joint__*c__q_RH_HAA_joint__)*c__q_RH_HFE_joint__ + 0.21*(-s__q_RH_HFE_joint__*s__q_RH_KFE_joint__ + c__q_RH_HFE_joint__*c__q_RH_KFE_joint__)*s__q_RH_HAA_joint__ + 0.21*(s__q_RH_HFE_joint__*c__q_RH_KFE_joint__ + s__q_RH_KFE_joint__*c__q_RH_HFE_joint__)*c__q_RH_HAA_joint__ - 0.38*c__q_RH_HAA_joint__*c__q_RH_KFE_joint__ - 0.3612*c__q_RH_HAA_joint__;
	(*this)(4, 2) = (-0.3612*s__q_RH_HFE_joint__*c__q_RH_KFE_joint__ - 0.38*s__q_RH_HFE_joint__ - 0.3612*s__q_RH_KFE_joint__*c__q_RH_HFE_joint__ + 0.37)*s__q_RH_HAA_joint__;
	(*this)(4, 3) = -(-s__q_RH_HFE_joint__*s__q_RH_KFE_joint__ + c__q_RH_HFE_joint__*c__q_RH_KFE_joint__)*s__q_RH_HAA_joint__;
	(*this)(4, 4) = (s__q_RH_HFE_joint__*c__q_RH_KFE_joint__ + s__q_RH_KFE_joint__*c__q_RH_HFE_joint__)*s__q_RH_HAA_joint__;
	(*this)(4, 5) = -c__q_RH_HAA_joint__;
	(*this)(5, 0) = 0.37*(-s__q_RH_HFE_joint__*s__q_RH_KFE_joint__ + c__q_RH_HFE_joint__*c__q_RH_KFE_joint__)*s__q_RH_HAA_joint__ + 0.05*(s__q_RH_HFE_joint__*c__q_RH_KFE_joint__ + s__q_RH_KFE_joint__*c__q_RH_HFE_joint__)*c__q_RH_HAA_joint__ + 0.38*s__q_RH_HAA_joint__*s__q_RH_KFE_joint__ + 0.1185*s__q_RH_HFE_joint__*c__q_RH_KFE_joint__ + 0.1185*s__q_RH_KFE_joint__*c__q_RH_HFE_joint__;
	(*this)(5, 1) = 0.05*(-s__q_RH_HFE_joint__*s__q_RH_KFE_joint__ + c__q_RH_HFE_joint__*c__q_RH_KFE_joint__)*c__q_RH_HAA_joint__ - 0.37*(s__q_RH_HFE_joint__*c__q_RH_KFE_joint__ + s__q_RH_KFE_joint__*c__q_RH_HFE_joint__)*s__q_RH_HAA_joint__ + 0.38*s__q_RH_HAA_joint__*c__q_RH_KFE_joint__ + 0.3612*s__q_RH_HAA_joint__ - 0.1185*s__q_RH_HFE_joint__*s__q_RH_KFE_joint__ + 0.1185*c__q_RH_HFE_joint__*c__q_RH_KFE_joint__;
	(*this)(5, 2) = (-0.3612*s__q_RH_HFE_joint__*c__q_RH_KFE_joint__ - 0.38*s__q_RH_HFE_joint__ - 0.3612*s__q_RH_KFE_joint__*c__q_RH_HFE_joint__ + 0.37)*c__q_RH_HAA_joint__;
	(*this)(5, 3) = -(-s__q_RH_HFE_joint__*s__q_RH_KFE_joint__ + c__q_RH_HFE_joint__*c__q_RH_KFE_joint__)*c__q_RH_HAA_joint__;
	(*this)(5, 4) = (s__q_RH_HFE_joint__*c__q_RH_KFE_joint__ + s__q_RH_KFE_joint__*c__q_RH_HFE_joint__)*c__q_RH_HAA_joint__;
	(*this)(5, 5) = s__q_RH_HAA_joint__;
	return *this;
}

template <typename TRAIT>
iit::pegasus2::tpl::MotionTransforms<TRAIT>::Type_fr_RH_foot_X_fr_torso::Type_fr_RH_foot_X_fr_torso(){ 
	(*this)(0, 3) = 0;
	(*this)(0, 4) = 0;
	(*this)(0, 5) = 0;
	(*this)(1, 3) = 0;
	(*this)(1, 4) = 0;
	(*this)(1, 5) = 0;
	(*this)(2, 0) = 0;
	(*this)(2, 3) = 0;
	(*this)(2, 4) = 0;
	(*this)(2, 5) = 0;
	(*this)(5, 3) = 0;
}

template <typename TRAIT>
const typename iit::pegasus2::tpl::MotionTransforms<TRAIT>::Type_fr_RH_foot_X_fr_torso& 
iit::pegasus2::tpl::MotionTransforms<TRAIT>::Type_fr_RH_foot_X_fr_torso::update(const JState& q){ 
	Scalar c__q_RH_HFE_joint__;
	Scalar c__q_RH_HAA_joint__;
	Scalar c__q_RH_KFE_joint__;
	Scalar s__q_RH_KFE_joint__;
	Scalar s__q_RH_HAA_joint__;
	Scalar s__q_RH_HFE_joint__;

	c__q_RH_HFE_joint__ = TRAIT::cos( q[RH_HFE_JOINT]);
	c__q_RH_HAA_joint__ = TRAIT::cos( q[RH_HAA_JOINT]);
	c__q_RH_KFE_joint__ = TRAIT::cos( q[RH_KFE_JOINT]);
	s__q_RH_KFE_joint__ = TRAIT::sin( q[RH_KFE_JOINT]);
	s__q_RH_HAA_joint__ = TRAIT::sin( q[RH_HAA_JOINT]);
	s__q_RH_HFE_joint__ = TRAIT::sin( q[RH_HFE_JOINT]);

	(*this)(0, 0) = s__q_RH_HFE_joint__*c__q_RH_KFE_joint__ + s__q_RH_KFE_joint__*c__q_RH_HFE_joint__;
	(*this)(0, 1) = -(-s__q_RH_HFE_joint__*s__q_RH_KFE_joint__ + c__q_RH_HFE_joint__*c__q_RH_KFE_joint__)*s__q_RH_HAA_joint__;
	(*this)(0, 2) = -(-s__q_RH_HFE_joint__*s__q_RH_KFE_joint__ + c__q_RH_HFE_joint__*c__q_RH_KFE_joint__)*c__q_RH_HAA_joint__;
	(*this)(1, 0) = -s__q_RH_HFE_joint__*s__q_RH_KFE_joint__ + c__q_RH_HFE_joint__*c__q_RH_KFE_joint__;
	(*this)(1, 1) = (s__q_RH_HFE_joint__*c__q_RH_KFE_joint__ + s__q_RH_KFE_joint__*c__q_RH_HFE_joint__)*s__q_RH_HAA_joint__;
	(*this)(1, 2) = (s__q_RH_HFE_joint__*c__q_RH_KFE_joint__ + s__q_RH_KFE_joint__*c__q_RH_HFE_joint__)*c__q_RH_HAA_joint__;
	(*this)(2, 1) = -c__q_RH_HAA_joint__;
	(*this)(2, 2) = s__q_RH_HAA_joint__;
	(*this)(3, 0) = (-s__q_RH_HFE_joint__*s__q_RH_KFE_joint__ + c__q_RH_HFE_joint__*c__q_RH_KFE_joint__)*(0.1185*c__q_RH_HAA_joint__ + 0.05);
	(*this)(3, 1) = -0.37*(-s__q_RH_HFE_joint__*s__q_RH_KFE_joint__ + c__q_RH_HFE_joint__*c__q_RH_KFE_joint__)*c__q_RH_HAA_joint__ + 0.05*(s__q_RH_HFE_joint__*c__q_RH_KFE_joint__ + s__q_RH_KFE_joint__*c__q_RH_HFE_joint__)*s__q_RH_HAA_joint__ - 0.38*s__q_RH_KFE_joint__*c__q_RH_HAA_joint__;
	(*this)(3, 2) = 0.37*(-s__q_RH_HFE_joint__*s__q_RH_KFE_joint__ + c__q_RH_HFE_joint__*c__q_RH_KFE_joint__)*s__q_RH_HAA_joint__ + 0.05*(s__q_RH_HFE_joint__*c__q_RH_KFE_joint__ + s__q_RH_KFE_joint__*c__q_RH_HFE_joint__)*c__q_RH_HAA_joint__ + 0.38*s__q_RH_HAA_joint__*s__q_RH_KFE_joint__ + 0.1185*s__q_RH_HFE_joint__*c__q_RH_KFE_joint__ + 0.1185*s__q_RH_KFE_joint__*c__q_RH_HFE_joint__;
	(*this)(3, 3) = s__q_RH_HFE_joint__*c__q_RH_KFE_joint__ + s__q_RH_KFE_joint__*c__q_RH_HFE_joint__;
	(*this)(3, 4) = -(-s__q_RH_HFE_joint__*s__q_RH_KFE_joint__ + c__q_RH_HFE_joint__*c__q_RH_KFE_joint__)*s__q_RH_HAA_joint__;
	(*this)(3, 5) = -(-s__q_RH_HFE_joint__*s__q_RH_KFE_joint__ + c__q_RH_HFE_joint__*c__q_RH_KFE_joint__)*c__q_RH_HAA_joint__;
	(*this)(4, 0) = -(s__q_RH_HFE_joint__*c__q_RH_KFE_joint__ + s__q_RH_KFE_joint__*c__q_RH_HFE_joint__)*(0.1185*c__q_RH_HAA_joint__ + 0.05);
	(*this)(4, 1) = 0.05*(-s__q_RH_HFE_joint__*s__q_RH_KFE_joint__ + c__q_RH_HFE_joint__*c__q_RH_KFE_joint__)*s__q_RH_HAA_joint__ + 0.37*(s__q_RH_HFE_joint__*c__q_RH_KFE_joint__ + s__q_RH_KFE_joint__*c__q_RH_HFE_joint__)*c__q_RH_HAA_joint__ - (0.38*c__q_RH_KFE_joint__ - 0.3612)*c__q_RH_HAA_joint__;
	(*this)(4, 2) = 0.05*(-s__q_RH_HFE_joint__*s__q_RH_KFE_joint__ + c__q_RH_HFE_joint__*c__q_RH_KFE_joint__)*c__q_RH_HAA_joint__ - 0.37*(s__q_RH_HFE_joint__*c__q_RH_KFE_joint__ + s__q_RH_KFE_joint__*c__q_RH_HFE_joint__)*s__q_RH_HAA_joint__ + (0.38*c__q_RH_KFE_joint__ - 0.3612)*s__q_RH_HAA_joint__ - 0.1185*s__q_RH_HFE_joint__*s__q_RH_KFE_joint__ + 0.1185*c__q_RH_HFE_joint__*c__q_RH_KFE_joint__;
	(*this)(4, 3) = -s__q_RH_HFE_joint__*s__q_RH_KFE_joint__ + c__q_RH_HFE_joint__*c__q_RH_KFE_joint__;
	(*this)(4, 4) = (s__q_RH_HFE_joint__*c__q_RH_KFE_joint__ + s__q_RH_KFE_joint__*c__q_RH_HFE_joint__)*s__q_RH_HAA_joint__;
	(*this)(4, 5) = (s__q_RH_HFE_joint__*c__q_RH_KFE_joint__ + s__q_RH_KFE_joint__*c__q_RH_HFE_joint__)*c__q_RH_HAA_joint__;
	(*this)(5, 0) = -0.1185*s__q_RH_HAA_joint__ - 0.3612*s__q_RH_HFE_joint__*s__q_RH_KFE_joint__ + 0.3612*c__q_RH_HFE_joint__*c__q_RH_KFE_joint__ - 0.38*c__q_RH_HFE_joint__;
	(*this)(5, 1) = (0.3612*s__q_RH_HFE_joint__*c__q_RH_KFE_joint__ - 0.38*s__q_RH_HFE_joint__ + 0.3612*s__q_RH_KFE_joint__*c__q_RH_HFE_joint__ + 0.37)*s__q_RH_HAA_joint__;
	(*this)(5, 2) = (0.3612*s__q_RH_HFE_joint__*c__q_RH_KFE_joint__ - 0.38*s__q_RH_HFE_joint__ + 0.3612*s__q_RH_KFE_joint__*c__q_RH_HFE_joint__ + 0.37)*c__q_RH_HAA_joint__;
	(*this)(5, 4) = -c__q_RH_HAA_joint__;
	(*this)(5, 5) = s__q_RH_HAA_joint__;
	return *this;
}

template <typename TRAIT>
iit::pegasus2::tpl::MotionTransforms<TRAIT>::Type_fr_torso_X_fr_LH_foot::Type_fr_torso_X_fr_LH_foot(){ 
	(*this)(0, 2) = 0;
	(*this)(0, 3) = 0;
	(*this)(0, 4) = 0;
	(*this)(0, 5) = 0;
	(*this)(1, 3) = 0;
	(*this)(1, 4) = 0;
	(*this)(1, 5) = 0;
	(*this)(2, 3) = 0;
	(*this)(2, 4) = 0;
	(*this)(2, 5) = 0;
	(*this)(3, 5) = 0;
}

template <typename TRAIT>
const typename iit::pegasus2::tpl::MotionTransforms<TRAIT>::Type_fr_torso_X_fr_LH_foot& 
iit::pegasus2::tpl::MotionTransforms<TRAIT>::Type_fr_torso_X_fr_LH_foot::update(const JState& q){ 
	Scalar c__q_LH_KFE_joint__;
	Scalar c__q_LH_HAA_joint__;
	Scalar c__q_LH_HFE_joint__;
	Scalar s__q_LH_HFE_joint__;
	Scalar s__q_LH_HAA_joint__;
	Scalar s__q_LH_KFE_joint__;

	c__q_LH_KFE_joint__ = TRAIT::cos( q[LH_KFE_JOINT]);
	c__q_LH_HAA_joint__ = TRAIT::cos( q[LH_HAA_JOINT]);
	c__q_LH_HFE_joint__ = TRAIT::cos( q[LH_HFE_JOINT]);
	s__q_LH_HFE_joint__ = TRAIT::sin( q[LH_HFE_JOINT]);
	s__q_LH_HAA_joint__ = TRAIT::sin( q[LH_HAA_JOINT]);
	s__q_LH_KFE_joint__ = TRAIT::sin( q[LH_KFE_JOINT]);

	(*this)(0, 0) = s__q_LH_HFE_joint__*c__q_LH_KFE_joint__ + s__q_LH_KFE_joint__*c__q_LH_HFE_joint__;
	(*this)(0, 1) = -s__q_LH_HFE_joint__*s__q_LH_KFE_joint__ + c__q_LH_HFE_joint__*c__q_LH_KFE_joint__;
	(*this)(1, 0) = -(-s__q_LH_HFE_joint__*s__q_LH_KFE_joint__ + c__q_LH_HFE_joint__*c__q_LH_KFE_joint__)*s__q_LH_HAA_joint__;
	(*this)(1, 1) = (s__q_LH_HFE_joint__*c__q_LH_KFE_joint__ + s__q_LH_KFE_joint__*c__q_LH_HFE_joint__)*s__q_LH_HAA_joint__;
	(*this)(1, 2) = -c__q_LH_HAA_joint__;
	(*this)(2, 0) = -(-s__q_LH_HFE_joint__*s__q_LH_KFE_joint__ + c__q_LH_HFE_joint__*c__q_LH_KFE_joint__)*c__q_LH_HAA_joint__;
	(*this)(2, 1) = (s__q_LH_HFE_joint__*c__q_LH_KFE_joint__ + s__q_LH_KFE_joint__*c__q_LH_HFE_joint__)*c__q_LH_HAA_joint__;
	(*this)(2, 2) = s__q_LH_HAA_joint__;
	(*this)(3, 0) = -(-s__q_LH_HFE_joint__*s__q_LH_KFE_joint__ + c__q_LH_HFE_joint__*c__q_LH_KFE_joint__)*(0.1185*c__q_LH_HAA_joint__ + 0.05);
	(*this)(3, 1) = (s__q_LH_HFE_joint__*c__q_LH_KFE_joint__ + s__q_LH_KFE_joint__*c__q_LH_HFE_joint__)*(0.1185*c__q_LH_HAA_joint__ + 0.05);
	(*this)(3, 2) = 0.1185*s__q_LH_HAA_joint__ + 0.3612*s__q_LH_HFE_joint__*s__q_LH_KFE_joint__ - 0.3612*c__q_LH_HFE_joint__*c__q_LH_KFE_joint__ - 0.38*c__q_LH_HFE_joint__;
	(*this)(3, 3) = s__q_LH_HFE_joint__*c__q_LH_KFE_joint__ + s__q_LH_KFE_joint__*c__q_LH_HFE_joint__;
	(*this)(3, 4) = -s__q_LH_HFE_joint__*s__q_LH_KFE_joint__ + c__q_LH_HFE_joint__*c__q_LH_KFE_joint__;
	(*this)(4, 0) = -0.21*(s__q_LH_HAA_joint__*s__q_LH_KFE_joint__ + c__q_LH_HAA_joint__*c__q_LH_KFE_joint__)*c__q_LH_HFE_joint__ + 0.21*(-s__q_LH_HAA_joint__*c__q_LH_KFE_joint__ + s__q_LH_KFE_joint__*c__q_LH_HAA_joint__)*s__q_LH_HFE_joint__ - 0.16*(-s__q_LH_HFE_joint__*s__q_LH_KFE_joint__ + c__q_LH_HFE_joint__*c__q_LH_KFE_joint__)*c__q_LH_HAA_joint__ + 0.16*(s__q_LH_HFE_joint__*c__q_LH_KFE_joint__ + s__q_LH_KFE_joint__*c__q_LH_HFE_joint__)*s__q_LH_HAA_joint__ - 0.38*s__q_LH_KFE_joint__*c__q_LH_HAA_joint__;
	(*this)(4, 1) = 0.21*(s__q_LH_HAA_joint__*s__q_LH_KFE_joint__ + c__q_LH_HAA_joint__*c__q_LH_KFE_joint__)*s__q_LH_HFE_joint__ + 0.21*(-s__q_LH_HAA_joint__*c__q_LH_KFE_joint__ + s__q_LH_KFE_joint__*c__q_LH_HAA_joint__)*c__q_LH_HFE_joint__ + 0.16*(-s__q_LH_HFE_joint__*s__q_LH_KFE_joint__ + c__q_LH_HFE_joint__*c__q_LH_KFE_joint__)*s__q_LH_HAA_joint__ + 0.16*(s__q_LH_HFE_joint__*c__q_LH_KFE_joint__ + s__q_LH_KFE_joint__*c__q_LH_HFE_joint__)*c__q_LH_HAA_joint__ - 0.38*c__q_LH_HAA_joint__*c__q_LH_KFE_joint__ - 0.3612*c__q_LH_HAA_joint__;
	(*this)(4, 2) = (-0.3612*s__q_LH_HFE_joint__*c__q_LH_KFE_joint__ - 0.38*s__q_LH_HFE_joint__ - 0.3612*s__q_LH_KFE_joint__*c__q_LH_HFE_joint__ + 0.37)*s__q_LH_HAA_joint__;
	(*this)(4, 3) = -(-s__q_LH_HFE_joint__*s__q_LH_KFE_joint__ + c__q_LH_HFE_joint__*c__q_LH_KFE_joint__)*s__q_LH_HAA_joint__;
	(*this)(4, 4) = (s__q_LH_HFE_joint__*c__q_LH_KFE_joint__ + s__q_LH_KFE_joint__*c__q_LH_HFE_joint__)*s__q_LH_HAA_joint__;
	(*this)(4, 5) = -c__q_LH_HAA_joint__;
	(*this)(5, 0) = 0.37*(-s__q_LH_HFE_joint__*s__q_LH_KFE_joint__ + c__q_LH_HFE_joint__*c__q_LH_KFE_joint__)*s__q_LH_HAA_joint__ - 0.05*(s__q_LH_HFE_joint__*c__q_LH_KFE_joint__ + s__q_LH_KFE_joint__*c__q_LH_HFE_joint__)*c__q_LH_HAA_joint__ + 0.38*s__q_LH_HAA_joint__*s__q_LH_KFE_joint__ - 0.1185*s__q_LH_HFE_joint__*c__q_LH_KFE_joint__ - 0.1185*s__q_LH_KFE_joint__*c__q_LH_HFE_joint__;
	(*this)(5, 1) = -0.05*(-s__q_LH_HFE_joint__*s__q_LH_KFE_joint__ + c__q_LH_HFE_joint__*c__q_LH_KFE_joint__)*c__q_LH_HAA_joint__ - 0.37*(s__q_LH_HFE_joint__*c__q_LH_KFE_joint__ + s__q_LH_KFE_joint__*c__q_LH_HFE_joint__)*s__q_LH_HAA_joint__ + 0.38*s__q_LH_HAA_joint__*c__q_LH_KFE_joint__ + 0.3612*s__q_LH_HAA_joint__ + 0.1185*s__q_LH_HFE_joint__*s__q_LH_KFE_joint__ - 0.1185*c__q_LH_HFE_joint__*c__q_LH_KFE_joint__;
	(*this)(5, 2) = (-0.3612*s__q_LH_HFE_joint__*c__q_LH_KFE_joint__ - 0.38*s__q_LH_HFE_joint__ - 0.3612*s__q_LH_KFE_joint__*c__q_LH_HFE_joint__ + 0.37)*c__q_LH_HAA_joint__;
	(*this)(5, 3) = -(-s__q_LH_HFE_joint__*s__q_LH_KFE_joint__ + c__q_LH_HFE_joint__*c__q_LH_KFE_joint__)*c__q_LH_HAA_joint__;
	(*this)(5, 4) = (s__q_LH_HFE_joint__*c__q_LH_KFE_joint__ + s__q_LH_KFE_joint__*c__q_LH_HFE_joint__)*c__q_LH_HAA_joint__;
	(*this)(5, 5) = s__q_LH_HAA_joint__;
	return *this;
}

template <typename TRAIT>
iit::pegasus2::tpl::MotionTransforms<TRAIT>::Type_fr_LH_foot_X_fr_torso::Type_fr_LH_foot_X_fr_torso(){ 
	(*this)(0, 3) = 0;
	(*this)(0, 4) = 0;
	(*this)(0, 5) = 0;
	(*this)(1, 3) = 0;
	(*this)(1, 4) = 0;
	(*this)(1, 5) = 0;
	(*this)(2, 0) = 0;
	(*this)(2, 3) = 0;
	(*this)(2, 4) = 0;
	(*this)(2, 5) = 0;
	(*this)(5, 3) = 0;
}

template <typename TRAIT>
const typename iit::pegasus2::tpl::MotionTransforms<TRAIT>::Type_fr_LH_foot_X_fr_torso& 
iit::pegasus2::tpl::MotionTransforms<TRAIT>::Type_fr_LH_foot_X_fr_torso::update(const JState& q){ 
	Scalar c__q_LH_KFE_joint__;
	Scalar c__q_LH_HAA_joint__;
	Scalar c__q_LH_HFE_joint__;
	Scalar s__q_LH_HFE_joint__;
	Scalar s__q_LH_HAA_joint__;
	Scalar s__q_LH_KFE_joint__;

	c__q_LH_KFE_joint__ = TRAIT::cos( q[LH_KFE_JOINT]);
	c__q_LH_HAA_joint__ = TRAIT::cos( q[LH_HAA_JOINT]);
	c__q_LH_HFE_joint__ = TRAIT::cos( q[LH_HFE_JOINT]);
	s__q_LH_HFE_joint__ = TRAIT::sin( q[LH_HFE_JOINT]);
	s__q_LH_HAA_joint__ = TRAIT::sin( q[LH_HAA_JOINT]);
	s__q_LH_KFE_joint__ = TRAIT::sin( q[LH_KFE_JOINT]);

	(*this)(0, 0) = s__q_LH_HFE_joint__*c__q_LH_KFE_joint__ + s__q_LH_KFE_joint__*c__q_LH_HFE_joint__;
	(*this)(0, 1) = -(-s__q_LH_HFE_joint__*s__q_LH_KFE_joint__ + c__q_LH_HFE_joint__*c__q_LH_KFE_joint__)*s__q_LH_HAA_joint__;
	(*this)(0, 2) = -(-s__q_LH_HFE_joint__*s__q_LH_KFE_joint__ + c__q_LH_HFE_joint__*c__q_LH_KFE_joint__)*c__q_LH_HAA_joint__;
	(*this)(1, 0) = -s__q_LH_HFE_joint__*s__q_LH_KFE_joint__ + c__q_LH_HFE_joint__*c__q_LH_KFE_joint__;
	(*this)(1, 1) = (s__q_LH_HFE_joint__*c__q_LH_KFE_joint__ + s__q_LH_KFE_joint__*c__q_LH_HFE_joint__)*s__q_LH_HAA_joint__;
	(*this)(1, 2) = (s__q_LH_HFE_joint__*c__q_LH_KFE_joint__ + s__q_LH_KFE_joint__*c__q_LH_HFE_joint__)*c__q_LH_HAA_joint__;
	(*this)(2, 1) = -c__q_LH_HAA_joint__;
	(*this)(2, 2) = s__q_LH_HAA_joint__;
	(*this)(3, 0) = -(-s__q_LH_HFE_joint__*s__q_LH_KFE_joint__ + c__q_LH_HFE_joint__*c__q_LH_KFE_joint__)*(0.1185*c__q_LH_HAA_joint__ + 0.05);
	(*this)(3, 1) = -0.37*(-s__q_LH_HFE_joint__*s__q_LH_KFE_joint__ + c__q_LH_HFE_joint__*c__q_LH_KFE_joint__)*c__q_LH_HAA_joint__ - 0.05*(s__q_LH_HFE_joint__*c__q_LH_KFE_joint__ + s__q_LH_KFE_joint__*c__q_LH_HFE_joint__)*s__q_LH_HAA_joint__ - 0.38*s__q_LH_KFE_joint__*c__q_LH_HAA_joint__;
	(*this)(3, 2) = 0.37*(-s__q_LH_HFE_joint__*s__q_LH_KFE_joint__ + c__q_LH_HFE_joint__*c__q_LH_KFE_joint__)*s__q_LH_HAA_joint__ - 0.05*(s__q_LH_HFE_joint__*c__q_LH_KFE_joint__ + s__q_LH_KFE_joint__*c__q_LH_HFE_joint__)*c__q_LH_HAA_joint__ + 0.38*s__q_LH_HAA_joint__*s__q_LH_KFE_joint__ - 0.1185*s__q_LH_HFE_joint__*c__q_LH_KFE_joint__ - 0.1185*s__q_LH_KFE_joint__*c__q_LH_HFE_joint__;
	(*this)(3, 3) = s__q_LH_HFE_joint__*c__q_LH_KFE_joint__ + s__q_LH_KFE_joint__*c__q_LH_HFE_joint__;
	(*this)(3, 4) = -(-s__q_LH_HFE_joint__*s__q_LH_KFE_joint__ + c__q_LH_HFE_joint__*c__q_LH_KFE_joint__)*s__q_LH_HAA_joint__;
	(*this)(3, 5) = -(-s__q_LH_HFE_joint__*s__q_LH_KFE_joint__ + c__q_LH_HFE_joint__*c__q_LH_KFE_joint__)*c__q_LH_HAA_joint__;
	(*this)(4, 0) = (s__q_LH_HFE_joint__*c__q_LH_KFE_joint__ + s__q_LH_KFE_joint__*c__q_LH_HFE_joint__)*(0.1185*c__q_LH_HAA_joint__ + 0.05);
	(*this)(4, 1) = -0.05*(-s__q_LH_HFE_joint__*s__q_LH_KFE_joint__ + c__q_LH_HFE_joint__*c__q_LH_KFE_joint__)*s__q_LH_HAA_joint__ + 0.37*(s__q_LH_HFE_joint__*c__q_LH_KFE_joint__ + s__q_LH_KFE_joint__*c__q_LH_HFE_joint__)*c__q_LH_HAA_joint__ - (0.38*c__q_LH_KFE_joint__ - 0.3612)*c__q_LH_HAA_joint__;
	(*this)(4, 2) = -0.05*(-s__q_LH_HFE_joint__*s__q_LH_KFE_joint__ + c__q_LH_HFE_joint__*c__q_LH_KFE_joint__)*c__q_LH_HAA_joint__ - 0.37*(s__q_LH_HFE_joint__*c__q_LH_KFE_joint__ + s__q_LH_KFE_joint__*c__q_LH_HFE_joint__)*s__q_LH_HAA_joint__ + (0.38*c__q_LH_KFE_joint__ - 0.3612)*s__q_LH_HAA_joint__ + 0.1185*s__q_LH_HFE_joint__*s__q_LH_KFE_joint__ - 0.1185*c__q_LH_HFE_joint__*c__q_LH_KFE_joint__;
	(*this)(4, 3) = -s__q_LH_HFE_joint__*s__q_LH_KFE_joint__ + c__q_LH_HFE_joint__*c__q_LH_KFE_joint__;
	(*this)(4, 4) = (s__q_LH_HFE_joint__*c__q_LH_KFE_joint__ + s__q_LH_KFE_joint__*c__q_LH_HFE_joint__)*s__q_LH_HAA_joint__;
	(*this)(4, 5) = (s__q_LH_HFE_joint__*c__q_LH_KFE_joint__ + s__q_LH_KFE_joint__*c__q_LH_HFE_joint__)*c__q_LH_HAA_joint__;
	(*this)(5, 0) = 0.1185*s__q_LH_HAA_joint__ - 0.3612*s__q_LH_HFE_joint__*s__q_LH_KFE_joint__ + 0.3612*c__q_LH_HFE_joint__*c__q_LH_KFE_joint__ - 0.38*c__q_LH_HFE_joint__;
	(*this)(5, 1) = (0.3612*s__q_LH_HFE_joint__*c__q_LH_KFE_joint__ - 0.38*s__q_LH_HFE_joint__ + 0.3612*s__q_LH_KFE_joint__*c__q_LH_HFE_joint__ + 0.37)*s__q_LH_HAA_joint__;
	(*this)(5, 2) = (0.3612*s__q_LH_HFE_joint__*c__q_LH_KFE_joint__ - 0.38*s__q_LH_HFE_joint__ + 0.3612*s__q_LH_KFE_joint__*c__q_LH_HFE_joint__ + 0.37)*c__q_LH_HAA_joint__;
	(*this)(5, 4) = -c__q_LH_HAA_joint__;
	(*this)(5, 5) = s__q_LH_HAA_joint__;
	return *this;
}

template <typename TRAIT>
iit::pegasus2::tpl::MotionTransforms<TRAIT>::Type_fr_torso_X_fr_LF_foot::Type_fr_torso_X_fr_LF_foot(){ 
	(*this)(0, 2) = 0;
	(*this)(0, 3) = 0;
	(*this)(0, 4) = 0;
	(*this)(0, 5) = 0;
	(*this)(1, 3) = 0;
	(*this)(1, 4) = 0;
	(*this)(1, 5) = 0;
	(*this)(2, 3) = 0;
	(*this)(2, 4) = 0;
	(*this)(2, 5) = 0;
	(*this)(3, 5) = 0;
}

template <typename TRAIT>
const typename iit::pegasus2::tpl::MotionTransforms<TRAIT>::Type_fr_torso_X_fr_LF_foot& 
iit::pegasus2::tpl::MotionTransforms<TRAIT>::Type_fr_torso_X_fr_LF_foot::update(const JState& q){ 
	Scalar c__q_LF_KFE_joint__;
	Scalar c__q_LF_HFE_joint__;
	Scalar c__q_LF_HAA_joint__;
	Scalar s__q_LF_HAA_joint__;
	Scalar s__q_LF_KFE_joint__;
	Scalar s__q_LF_HFE_joint__;

	c__q_LF_KFE_joint__ = TRAIT::cos( q[LF_KFE_JOINT]);
	c__q_LF_HFE_joint__ = TRAIT::cos( q[LF_HFE_JOINT]);
	c__q_LF_HAA_joint__ = TRAIT::cos( q[LF_HAA_JOINT]);
	s__q_LF_HAA_joint__ = TRAIT::sin( q[LF_HAA_JOINT]);
	s__q_LF_KFE_joint__ = TRAIT::sin( q[LF_KFE_JOINT]);
	s__q_LF_HFE_joint__ = TRAIT::sin( q[LF_HFE_JOINT]);

	(*this)(0, 0) = s__q_LF_HFE_joint__*c__q_LF_KFE_joint__ + s__q_LF_KFE_joint__*c__q_LF_HFE_joint__;
	(*this)(0, 1) = -s__q_LF_HFE_joint__*s__q_LF_KFE_joint__ + c__q_LF_HFE_joint__*c__q_LF_KFE_joint__;
	(*this)(1, 0) = -(-s__q_LF_HFE_joint__*s__q_LF_KFE_joint__ + c__q_LF_HFE_joint__*c__q_LF_KFE_joint__)*s__q_LF_HAA_joint__;
	(*this)(1, 1) = (s__q_LF_HFE_joint__*c__q_LF_KFE_joint__ + s__q_LF_KFE_joint__*c__q_LF_HFE_joint__)*s__q_LF_HAA_joint__;
	(*this)(1, 2) = -c__q_LF_HAA_joint__;
	(*this)(2, 0) = -(-s__q_LF_HFE_joint__*s__q_LF_KFE_joint__ + c__q_LF_HFE_joint__*c__q_LF_KFE_joint__)*c__q_LF_HAA_joint__;
	(*this)(2, 1) = (s__q_LF_HFE_joint__*c__q_LF_KFE_joint__ + s__q_LF_KFE_joint__*c__q_LF_HFE_joint__)*c__q_LF_HAA_joint__;
	(*this)(2, 2) = s__q_LF_HAA_joint__;
	(*this)(3, 0) = -(-s__q_LF_HFE_joint__*s__q_LF_KFE_joint__ + c__q_LF_HFE_joint__*c__q_LF_KFE_joint__)*(0.1185*c__q_LF_HAA_joint__ + 0.05);
	(*this)(3, 1) = (s__q_LF_HFE_joint__*c__q_LF_KFE_joint__ + s__q_LF_KFE_joint__*c__q_LF_HFE_joint__)*(0.1185*c__q_LF_HAA_joint__ + 0.05);
	(*this)(3, 2) = 0.1185*s__q_LF_HAA_joint__ + 0.3612*s__q_LF_HFE_joint__*s__q_LF_KFE_joint__ - 0.3612*c__q_LF_HFE_joint__*c__q_LF_KFE_joint__ - 0.38*c__q_LF_HFE_joint__;
	(*this)(3, 3) = s__q_LF_HFE_joint__*c__q_LF_KFE_joint__ + s__q_LF_KFE_joint__*c__q_LF_HFE_joint__;
	(*this)(3, 4) = -s__q_LF_HFE_joint__*s__q_LF_KFE_joint__ + c__q_LF_HFE_joint__*c__q_LF_KFE_joint__;
	(*this)(4, 0) = 0.16*(s__q_LF_HAA_joint__*s__q_LF_KFE_joint__ + c__q_LF_HAA_joint__*c__q_LF_KFE_joint__)*c__q_LF_HFE_joint__ - 0.16*(-s__q_LF_HAA_joint__*c__q_LF_KFE_joint__ + s__q_LF_KFE_joint__*c__q_LF_HAA_joint__)*s__q_LF_HFE_joint__ + 0.21*(-s__q_LF_HFE_joint__*s__q_LF_KFE_joint__ + c__q_LF_HFE_joint__*c__q_LF_KFE_joint__)*c__q_LF_HAA_joint__ - 0.21*(s__q_LF_HFE_joint__*c__q_LF_KFE_joint__ + s__q_LF_KFE_joint__*c__q_LF_HFE_joint__)*s__q_LF_HAA_joint__ - 0.38*s__q_LF_KFE_joint__*c__q_LF_HAA_joint__;
	(*this)(4, 1) = -0.16*(s__q_LF_HAA_joint__*s__q_LF_KFE_joint__ + c__q_LF_HAA_joint__*c__q_LF_KFE_joint__)*s__q_LF_HFE_joint__ - 0.16*(-s__q_LF_HAA_joint__*c__q_LF_KFE_joint__ + s__q_LF_KFE_joint__*c__q_LF_HAA_joint__)*c__q_LF_HFE_joint__ - 0.21*(-s__q_LF_HFE_joint__*s__q_LF_KFE_joint__ + c__q_LF_HFE_joint__*c__q_LF_KFE_joint__)*s__q_LF_HAA_joint__ - 0.21*(s__q_LF_HFE_joint__*c__q_LF_KFE_joint__ + s__q_LF_KFE_joint__*c__q_LF_HFE_joint__)*c__q_LF_HAA_joint__ - 0.38*c__q_LF_HAA_joint__*c__q_LF_KFE_joint__ - 0.3612*c__q_LF_HAA_joint__;
	(*this)(4, 2) = -(0.3612*s__q_LF_HFE_joint__*c__q_LF_KFE_joint__ + 0.38*s__q_LF_HFE_joint__ + 0.3612*s__q_LF_KFE_joint__*c__q_LF_HFE_joint__ + 0.37)*s__q_LF_HAA_joint__;
	(*this)(4, 3) = -(-s__q_LF_HFE_joint__*s__q_LF_KFE_joint__ + c__q_LF_HFE_joint__*c__q_LF_KFE_joint__)*s__q_LF_HAA_joint__;
	(*this)(4, 4) = (s__q_LF_HFE_joint__*c__q_LF_KFE_joint__ + s__q_LF_KFE_joint__*c__q_LF_HFE_joint__)*s__q_LF_HAA_joint__;
	(*this)(4, 5) = -c__q_LF_HAA_joint__;
	(*this)(5, 0) = -0.37*(-s__q_LF_HFE_joint__*s__q_LF_KFE_joint__ + c__q_LF_HFE_joint__*c__q_LF_KFE_joint__)*s__q_LF_HAA_joint__ - 0.05*(s__q_LF_HFE_joint__*c__q_LF_KFE_joint__ + s__q_LF_KFE_joint__*c__q_LF_HFE_joint__)*c__q_LF_HAA_joint__ + 0.38*s__q_LF_HAA_joint__*s__q_LF_KFE_joint__ - 0.1185*s__q_LF_HFE_joint__*c__q_LF_KFE_joint__ - 0.1185*s__q_LF_KFE_joint__*c__q_LF_HFE_joint__;
	(*this)(5, 1) = -0.05*(-s__q_LF_HFE_joint__*s__q_LF_KFE_joint__ + c__q_LF_HFE_joint__*c__q_LF_KFE_joint__)*c__q_LF_HAA_joint__ + 0.37*(s__q_LF_HFE_joint__*c__q_LF_KFE_joint__ + s__q_LF_KFE_joint__*c__q_LF_HFE_joint__)*s__q_LF_HAA_joint__ + 0.38*s__q_LF_HAA_joint__*c__q_LF_KFE_joint__ + 0.3612*s__q_LF_HAA_joint__ + 0.1185*s__q_LF_HFE_joint__*s__q_LF_KFE_joint__ - 0.1185*c__q_LF_HFE_joint__*c__q_LF_KFE_joint__;
	(*this)(5, 2) = -(0.3612*s__q_LF_HFE_joint__*c__q_LF_KFE_joint__ + 0.38*s__q_LF_HFE_joint__ + 0.3612*s__q_LF_KFE_joint__*c__q_LF_HFE_joint__ + 0.37)*c__q_LF_HAA_joint__;
	(*this)(5, 3) = -(-s__q_LF_HFE_joint__*s__q_LF_KFE_joint__ + c__q_LF_HFE_joint__*c__q_LF_KFE_joint__)*c__q_LF_HAA_joint__;
	(*this)(5, 4) = (s__q_LF_HFE_joint__*c__q_LF_KFE_joint__ + s__q_LF_KFE_joint__*c__q_LF_HFE_joint__)*c__q_LF_HAA_joint__;
	(*this)(5, 5) = s__q_LF_HAA_joint__;
	return *this;
}

template <typename TRAIT>
iit::pegasus2::tpl::MotionTransforms<TRAIT>::Type_fr_LF_foot_X_fr_torso::Type_fr_LF_foot_X_fr_torso(){ 
	(*this)(0, 3) = 0;
	(*this)(0, 4) = 0;
	(*this)(0, 5) = 0;
	(*this)(1, 3) = 0;
	(*this)(1, 4) = 0;
	(*this)(1, 5) = 0;
	(*this)(2, 0) = 0;
	(*this)(2, 3) = 0;
	(*this)(2, 4) = 0;
	(*this)(2, 5) = 0;
	(*this)(5, 3) = 0;
}

template <typename TRAIT>
const typename iit::pegasus2::tpl::MotionTransforms<TRAIT>::Type_fr_LF_foot_X_fr_torso& 
iit::pegasus2::tpl::MotionTransforms<TRAIT>::Type_fr_LF_foot_X_fr_torso::update(const JState& q){ 
	Scalar c__q_LF_KFE_joint__;
	Scalar c__q_LF_HFE_joint__;
	Scalar c__q_LF_HAA_joint__;
	Scalar s__q_LF_HAA_joint__;
	Scalar s__q_LF_KFE_joint__;
	Scalar s__q_LF_HFE_joint__;

	c__q_LF_KFE_joint__ = TRAIT::cos( q[LF_KFE_JOINT]);
	c__q_LF_HFE_joint__ = TRAIT::cos( q[LF_HFE_JOINT]);
	c__q_LF_HAA_joint__ = TRAIT::cos( q[LF_HAA_JOINT]);
	s__q_LF_HAA_joint__ = TRAIT::sin( q[LF_HAA_JOINT]);
	s__q_LF_KFE_joint__ = TRAIT::sin( q[LF_KFE_JOINT]);
	s__q_LF_HFE_joint__ = TRAIT::sin( q[LF_HFE_JOINT]);

	(*this)(0, 0) = s__q_LF_HFE_joint__*c__q_LF_KFE_joint__ + s__q_LF_KFE_joint__*c__q_LF_HFE_joint__;
	(*this)(0, 1) = -(-s__q_LF_HFE_joint__*s__q_LF_KFE_joint__ + c__q_LF_HFE_joint__*c__q_LF_KFE_joint__)*s__q_LF_HAA_joint__;
	(*this)(0, 2) = -(-s__q_LF_HFE_joint__*s__q_LF_KFE_joint__ + c__q_LF_HFE_joint__*c__q_LF_KFE_joint__)*c__q_LF_HAA_joint__;
	(*this)(1, 0) = -s__q_LF_HFE_joint__*s__q_LF_KFE_joint__ + c__q_LF_HFE_joint__*c__q_LF_KFE_joint__;
	(*this)(1, 1) = (s__q_LF_HFE_joint__*c__q_LF_KFE_joint__ + s__q_LF_KFE_joint__*c__q_LF_HFE_joint__)*s__q_LF_HAA_joint__;
	(*this)(1, 2) = (s__q_LF_HFE_joint__*c__q_LF_KFE_joint__ + s__q_LF_KFE_joint__*c__q_LF_HFE_joint__)*c__q_LF_HAA_joint__;
	(*this)(2, 1) = -c__q_LF_HAA_joint__;
	(*this)(2, 2) = s__q_LF_HAA_joint__;
	(*this)(3, 0) = -(-s__q_LF_HFE_joint__*s__q_LF_KFE_joint__ + c__q_LF_HFE_joint__*c__q_LF_KFE_joint__)*(0.1185*c__q_LF_HAA_joint__ + 0.05);
	(*this)(3, 1) = 0.37*(-s__q_LF_HFE_joint__*s__q_LF_KFE_joint__ + c__q_LF_HFE_joint__*c__q_LF_KFE_joint__)*c__q_LF_HAA_joint__ - 0.05*(s__q_LF_HFE_joint__*c__q_LF_KFE_joint__ + s__q_LF_KFE_joint__*c__q_LF_HFE_joint__)*s__q_LF_HAA_joint__ - 0.38*s__q_LF_KFE_joint__*c__q_LF_HAA_joint__;
	(*this)(3, 2) = -0.37*(-s__q_LF_HFE_joint__*s__q_LF_KFE_joint__ + c__q_LF_HFE_joint__*c__q_LF_KFE_joint__)*s__q_LF_HAA_joint__ - 0.05*(s__q_LF_HFE_joint__*c__q_LF_KFE_joint__ + s__q_LF_KFE_joint__*c__q_LF_HFE_joint__)*c__q_LF_HAA_joint__ + 0.38*s__q_LF_HAA_joint__*s__q_LF_KFE_joint__ - 0.1185*s__q_LF_HFE_joint__*c__q_LF_KFE_joint__ - 0.1185*s__q_LF_KFE_joint__*c__q_LF_HFE_joint__;
	(*this)(3, 3) = s__q_LF_HFE_joint__*c__q_LF_KFE_joint__ + s__q_LF_KFE_joint__*c__q_LF_HFE_joint__;
	(*this)(3, 4) = -(-s__q_LF_HFE_joint__*s__q_LF_KFE_joint__ + c__q_LF_HFE_joint__*c__q_LF_KFE_joint__)*s__q_LF_HAA_joint__;
	(*this)(3, 5) = -(-s__q_LF_HFE_joint__*s__q_LF_KFE_joint__ + c__q_LF_HFE_joint__*c__q_LF_KFE_joint__)*c__q_LF_HAA_joint__;
	(*this)(4, 0) = (s__q_LF_HFE_joint__*c__q_LF_KFE_joint__ + s__q_LF_KFE_joint__*c__q_LF_HFE_joint__)*(0.1185*c__q_LF_HAA_joint__ + 0.05);
	(*this)(4, 1) = -0.05*(-s__q_LF_HFE_joint__*s__q_LF_KFE_joint__ + c__q_LF_HFE_joint__*c__q_LF_KFE_joint__)*s__q_LF_HAA_joint__ - 0.37*(s__q_LF_HFE_joint__*c__q_LF_KFE_joint__ + s__q_LF_KFE_joint__*c__q_LF_HFE_joint__)*c__q_LF_HAA_joint__ - (0.38*c__q_LF_KFE_joint__ - 0.3612)*c__q_LF_HAA_joint__;
	(*this)(4, 2) = -0.05*(-s__q_LF_HFE_joint__*s__q_LF_KFE_joint__ + c__q_LF_HFE_joint__*c__q_LF_KFE_joint__)*c__q_LF_HAA_joint__ + 0.37*(s__q_LF_HFE_joint__*c__q_LF_KFE_joint__ + s__q_LF_KFE_joint__*c__q_LF_HFE_joint__)*s__q_LF_HAA_joint__ + (0.38*c__q_LF_KFE_joint__ - 0.3612)*s__q_LF_HAA_joint__ + 0.1185*s__q_LF_HFE_joint__*s__q_LF_KFE_joint__ - 0.1185*c__q_LF_HFE_joint__*c__q_LF_KFE_joint__;
	(*this)(4, 3) = -s__q_LF_HFE_joint__*s__q_LF_KFE_joint__ + c__q_LF_HFE_joint__*c__q_LF_KFE_joint__;
	(*this)(4, 4) = (s__q_LF_HFE_joint__*c__q_LF_KFE_joint__ + s__q_LF_KFE_joint__*c__q_LF_HFE_joint__)*s__q_LF_HAA_joint__;
	(*this)(4, 5) = (s__q_LF_HFE_joint__*c__q_LF_KFE_joint__ + s__q_LF_KFE_joint__*c__q_LF_HFE_joint__)*c__q_LF_HAA_joint__;
	(*this)(5, 0) = 0.1185*s__q_LF_HAA_joint__ - 0.3612*s__q_LF_HFE_joint__*s__q_LF_KFE_joint__ + 0.3612*c__q_LF_HFE_joint__*c__q_LF_KFE_joint__ - 0.38*c__q_LF_HFE_joint__;
	(*this)(5, 1) = (0.3612*s__q_LF_HFE_joint__*c__q_LF_KFE_joint__ - 0.38*s__q_LF_HFE_joint__ + 0.3612*s__q_LF_KFE_joint__*c__q_LF_HFE_joint__ - 0.37)*s__q_LF_HAA_joint__;
	(*this)(5, 2) = (0.3612*s__q_LF_HFE_joint__*c__q_LF_KFE_joint__ - 0.38*s__q_LF_HFE_joint__ + 0.3612*s__q_LF_KFE_joint__*c__q_LF_HFE_joint__ - 0.37)*c__q_LF_HAA_joint__;
	(*this)(5, 4) = -c__q_LF_HAA_joint__;
	(*this)(5, 5) = s__q_LF_HAA_joint__;
	return *this;
}

template <typename TRAIT>
iit::pegasus2::tpl::ForceTransforms<TRAIT>::Type_fr_RF_hip_X_fr_RF_upperleg::Type_fr_RF_hip_X_fr_RF_upperleg(){ 
	(*this)(0, 2) = 0;
	(*this)(0, 5) = 0;
	(*this)(1, 0) = 0;
	(*this)(1, 1) = 0;
	(*this)(1, 2) = -1;
	(*this)(1, 3) = 0;
	(*this)(1, 4) = 0;
	(*this)(1, 5) = 0;
	(*this)(2, 2) = 0;
	(*this)(2, 5) = 0;
	(*this)(3, 0) = 0;
	(*this)(3, 1) = 0;
	(*this)(3, 2) = 0;
	(*this)(3, 5) = 0;
	(*this)(4, 0) = 0;
	(*this)(4, 1) = 0;
	(*this)(4, 2) = 0;
	(*this)(4, 3) = 0;
	(*this)(4, 4) = 0;
	(*this)(4, 5) = -1;
	(*this)(5, 0) = 0;
	(*this)(5, 1) = 0;
	(*this)(5, 2) = 0;
	(*this)(5, 5) = 0;
}

template <typename TRAIT>
const typename iit::pegasus2::tpl::ForceTransforms<TRAIT>::Type_fr_RF_hip_X_fr_RF_upperleg& 
iit::pegasus2::tpl::ForceTransforms<TRAIT>::Type_fr_RF_hip_X_fr_RF_upperleg::update(const JState& q){ 
	Scalar c__q_RF_HFE_joint__;
	Scalar s__q_RF_HFE_joint__;

	c__q_RF_HFE_joint__ = TRAIT::cos( q[RF_HFE_JOINT]);
	s__q_RF_HFE_joint__ = TRAIT::sin( q[RF_HFE_JOINT]);

	(*this)(0, 0) = -c__q_RF_HFE_joint__;
	(*this)(0, 1) = s__q_RF_HFE_joint__;
	(*this)(0, 3) = 0.05*s__q_RF_HFE_joint__;
	(*this)(0, 4) = 0.05*c__q_RF_HFE_joint__;
	(*this)(2, 0) = -s__q_RF_HFE_joint__;
	(*this)(2, 1) = -c__q_RF_HFE_joint__;
	(*this)(2, 3) = -0.05*c__q_RF_HFE_joint__;
	(*this)(2, 4) = 0.05*s__q_RF_HFE_joint__;
	(*this)(3, 3) = -c__q_RF_HFE_joint__;
	(*this)(3, 4) = s__q_RF_HFE_joint__;
	(*this)(5, 3) = -s__q_RF_HFE_joint__;
	(*this)(5, 4) = -c__q_RF_HFE_joint__;
	return *this;
}

template <typename TRAIT>
iit::pegasus2::tpl::ForceTransforms<TRAIT>::Type_fr_RF_upperleg_X_fr_RF_hip::Type_fr_RF_upperleg_X_fr_RF_hip(){ 
	(*this)(0, 1) = 0;
	(*this)(0, 4) = 0;
	(*this)(1, 1) = 0;
	(*this)(1, 4) = 0;
	(*this)(2, 0) = 0;
	(*this)(2, 1) = -1;
	(*this)(2, 2) = 0;
	(*this)(2, 3) = 0;
	(*this)(2, 4) = 0;
	(*this)(2, 5) = 0;
	(*this)(3, 0) = 0;
	(*this)(3, 1) = 0;
	(*this)(3, 2) = 0;
	(*this)(3, 4) = 0;
	(*this)(4, 0) = 0;
	(*this)(4, 1) = 0;
	(*this)(4, 2) = 0;
	(*this)(4, 4) = 0;
	(*this)(5, 0) = 0;
	(*this)(5, 1) = 0;
	(*this)(5, 2) = 0;
	(*this)(5, 3) = 0;
	(*this)(5, 4) = -1;
	(*this)(5, 5) = 0;
}

template <typename TRAIT>
const typename iit::pegasus2::tpl::ForceTransforms<TRAIT>::Type_fr_RF_upperleg_X_fr_RF_hip& 
iit::pegasus2::tpl::ForceTransforms<TRAIT>::Type_fr_RF_upperleg_X_fr_RF_hip::update(const JState& q){ 
	Scalar c__q_RF_HFE_joint__;
	Scalar s__q_RF_HFE_joint__;

	c__q_RF_HFE_joint__ = TRAIT::cos( q[RF_HFE_JOINT]);
	s__q_RF_HFE_joint__ = TRAIT::sin( q[RF_HFE_JOINT]);

	(*this)(0, 0) = -c__q_RF_HFE_joint__;
	(*this)(0, 2) = -s__q_RF_HFE_joint__;
	(*this)(0, 3) = 0.05*s__q_RF_HFE_joint__;
	(*this)(0, 5) = -0.05*c__q_RF_HFE_joint__;
	(*this)(1, 0) = s__q_RF_HFE_joint__;
	(*this)(1, 2) = -c__q_RF_HFE_joint__;
	(*this)(1, 3) = 0.05*c__q_RF_HFE_joint__;
	(*this)(1, 5) = 0.05*s__q_RF_HFE_joint__;
	(*this)(3, 3) = -c__q_RF_HFE_joint__;
	(*this)(3, 5) = -s__q_RF_HFE_joint__;
	(*this)(4, 3) = s__q_RF_HFE_joint__;
	(*this)(4, 5) = -c__q_RF_HFE_joint__;
	return *this;
}

template <typename TRAIT>
iit::pegasus2::tpl::ForceTransforms<TRAIT>::Type_fr_RF_upperleg_X_fr_RF_lowerleg::Type_fr_RF_upperleg_X_fr_RF_lowerleg(){ 
	(*this)(0, 2) = 0;
	(*this)(0, 3) = 0;
	(*this)(0, 4) = 0;
	(*this)(0, 5) = 0;
	(*this)(1, 2) = 0;
	(*this)(1, 3) = 0;
	(*this)(1, 4) = 0;
	(*this)(1, 5) = -0.380000000000000;
	(*this)(2, 0) = 0;
	(*this)(2, 1) = 0;
	(*this)(2, 2) = 1;
	(*this)(2, 5) = 0;
	(*this)(3, 0) = 0;
	(*this)(3, 1) = 0;
	(*this)(3, 2) = 0;
	(*this)(3, 5) = 0;
	(*this)(4, 0) = 0;
	(*this)(4, 1) = 0;
	(*this)(4, 2) = 0;
	(*this)(4, 5) = 0;
	(*this)(5, 0) = 0;
	(*this)(5, 1) = 0;
	(*this)(5, 2) = 0;
	(*this)(5, 3) = 0;
	(*this)(5, 4) = 0;
	(*this)(5, 5) = 1;
}

template <typename TRAIT>
const typename iit::pegasus2::tpl::ForceTransforms<TRAIT>::Type_fr_RF_upperleg_X_fr_RF_lowerleg& 
iit::pegasus2::tpl::ForceTransforms<TRAIT>::Type_fr_RF_upperleg_X_fr_RF_lowerleg::update(const JState& q){ 
	Scalar c__q_RF_KFE_joint__;
	Scalar s__q_RF_KFE_joint__;

	c__q_RF_KFE_joint__ = TRAIT::cos( q[RF_KFE_JOINT]);
	s__q_RF_KFE_joint__ = TRAIT::sin( q[RF_KFE_JOINT]);

	(*this)(0, 0) = c__q_RF_KFE_joint__;
	(*this)(0, 1) = -s__q_RF_KFE_joint__;
	(*this)(1, 0) = s__q_RF_KFE_joint__;
	(*this)(1, 1) = c__q_RF_KFE_joint__;
	(*this)(2, 3) = 0.38*s__q_RF_KFE_joint__;
	(*this)(2, 4) = 0.38*c__q_RF_KFE_joint__;
	(*this)(3, 3) = c__q_RF_KFE_joint__;
	(*this)(3, 4) = -s__q_RF_KFE_joint__;
	(*this)(4, 3) = s__q_RF_KFE_joint__;
	(*this)(4, 4) = c__q_RF_KFE_joint__;
	return *this;
}

template <typename TRAIT>
iit::pegasus2::tpl::ForceTransforms<TRAIT>::Type_fr_RF_lowerleg_X_fr_RF_upperleg::Type_fr_RF_lowerleg_X_fr_RF_upperleg(){ 
	(*this)(0, 2) = 0;
	(*this)(0, 3) = 0;
	(*this)(0, 4) = 0;
	(*this)(1, 2) = 0;
	(*this)(1, 3) = 0;
	(*this)(1, 4) = 0;
	(*this)(2, 0) = 0;
	(*this)(2, 1) = 0;
	(*this)(2, 2) = 1;
	(*this)(2, 3) = 0;
	(*this)(2, 4) = -0.380000000000000;
	(*this)(2, 5) = 0;
	(*this)(3, 0) = 0;
	(*this)(3, 1) = 0;
	(*this)(3, 2) = 0;
	(*this)(3, 5) = 0;
	(*this)(4, 0) = 0;
	(*this)(4, 1) = 0;
	(*this)(4, 2) = 0;
	(*this)(4, 5) = 0;
	(*this)(5, 0) = 0;
	(*this)(5, 1) = 0;
	(*this)(5, 2) = 0;
	(*this)(5, 3) = 0;
	(*this)(5, 4) = 0;
	(*this)(5, 5) = 1;
}

template <typename TRAIT>
const typename iit::pegasus2::tpl::ForceTransforms<TRAIT>::Type_fr_RF_lowerleg_X_fr_RF_upperleg& 
iit::pegasus2::tpl::ForceTransforms<TRAIT>::Type_fr_RF_lowerleg_X_fr_RF_upperleg::update(const JState& q){ 
	Scalar c__q_RF_KFE_joint__;
	Scalar s__q_RF_KFE_joint__;

	c__q_RF_KFE_joint__ = TRAIT::cos( q[RF_KFE_JOINT]);
	s__q_RF_KFE_joint__ = TRAIT::sin( q[RF_KFE_JOINT]);

	(*this)(0, 0) = c__q_RF_KFE_joint__;
	(*this)(0, 1) = s__q_RF_KFE_joint__;
	(*this)(0, 5) = 0.38*s__q_RF_KFE_joint__;
	(*this)(1, 0) = -s__q_RF_KFE_joint__;
	(*this)(1, 1) = c__q_RF_KFE_joint__;
	(*this)(1, 5) = 0.38*c__q_RF_KFE_joint__;
	(*this)(3, 3) = c__q_RF_KFE_joint__;
	(*this)(3, 4) = s__q_RF_KFE_joint__;
	(*this)(4, 3) = -s__q_RF_KFE_joint__;
	(*this)(4, 4) = c__q_RF_KFE_joint__;
	return *this;
}

template <typename TRAIT>
iit::pegasus2::tpl::ForceTransforms<TRAIT>::Type_fr_RH_hip_X_fr_RH_upperleg::Type_fr_RH_hip_X_fr_RH_upperleg(){ 
	(*this)(0, 2) = 0;
	(*this)(0, 5) = 0;
	(*this)(1, 0) = 0;
	(*this)(1, 1) = 0;
	(*this)(1, 2) = -1;
	(*this)(1, 3) = 0;
	(*this)(1, 4) = 0;
	(*this)(1, 5) = 0;
	(*this)(2, 2) = 0;
	(*this)(2, 5) = 0;
	(*this)(3, 0) = 0;
	(*this)(3, 1) = 0;
	(*this)(3, 2) = 0;
	(*this)(3, 5) = 0;
	(*this)(4, 0) = 0;
	(*this)(4, 1) = 0;
	(*this)(4, 2) = 0;
	(*this)(4, 3) = 0;
	(*this)(4, 4) = 0;
	(*this)(4, 5) = -1;
	(*this)(5, 0) = 0;
	(*this)(5, 1) = 0;
	(*this)(5, 2) = 0;
	(*this)(5, 5) = 0;
}

template <typename TRAIT>
const typename iit::pegasus2::tpl::ForceTransforms<TRAIT>::Type_fr_RH_hip_X_fr_RH_upperleg& 
iit::pegasus2::tpl::ForceTransforms<TRAIT>::Type_fr_RH_hip_X_fr_RH_upperleg::update(const JState& q){ 
	Scalar c__q_RH_HFE_joint__;
	Scalar s__q_RH_HFE_joint__;

	c__q_RH_HFE_joint__ = TRAIT::cos( q[RH_HFE_JOINT]);
	s__q_RH_HFE_joint__ = TRAIT::sin( q[RH_HFE_JOINT]);

	(*this)(0, 0) = -c__q_RH_HFE_joint__;
	(*this)(0, 1) = s__q_RH_HFE_joint__;
	(*this)(0, 3) = 0.05*s__q_RH_HFE_joint__;
	(*this)(0, 4) = 0.05*c__q_RH_HFE_joint__;
	(*this)(2, 0) = -s__q_RH_HFE_joint__;
	(*this)(2, 1) = -c__q_RH_HFE_joint__;
	(*this)(2, 3) = -0.05*c__q_RH_HFE_joint__;
	(*this)(2, 4) = 0.05*s__q_RH_HFE_joint__;
	(*this)(3, 3) = -c__q_RH_HFE_joint__;
	(*this)(3, 4) = s__q_RH_HFE_joint__;
	(*this)(5, 3) = -s__q_RH_HFE_joint__;
	(*this)(5, 4) = -c__q_RH_HFE_joint__;
	return *this;
}

template <typename TRAIT>
iit::pegasus2::tpl::ForceTransforms<TRAIT>::Type_fr_RH_upperleg_X_fr_RH_hip::Type_fr_RH_upperleg_X_fr_RH_hip(){ 
	(*this)(0, 1) = 0;
	(*this)(0, 4) = 0;
	(*this)(1, 1) = 0;
	(*this)(1, 4) = 0;
	(*this)(2, 0) = 0;
	(*this)(2, 1) = -1;
	(*this)(2, 2) = 0;
	(*this)(2, 3) = 0;
	(*this)(2, 4) = 0;
	(*this)(2, 5) = 0;
	(*this)(3, 0) = 0;
	(*this)(3, 1) = 0;
	(*this)(3, 2) = 0;
	(*this)(3, 4) = 0;
	(*this)(4, 0) = 0;
	(*this)(4, 1) = 0;
	(*this)(4, 2) = 0;
	(*this)(4, 4) = 0;
	(*this)(5, 0) = 0;
	(*this)(5, 1) = 0;
	(*this)(5, 2) = 0;
	(*this)(5, 3) = 0;
	(*this)(5, 4) = -1;
	(*this)(5, 5) = 0;
}

template <typename TRAIT>
const typename iit::pegasus2::tpl::ForceTransforms<TRAIT>::Type_fr_RH_upperleg_X_fr_RH_hip& 
iit::pegasus2::tpl::ForceTransforms<TRAIT>::Type_fr_RH_upperleg_X_fr_RH_hip::update(const JState& q){ 
	Scalar c__q_RH_HFE_joint__;
	Scalar s__q_RH_HFE_joint__;

	c__q_RH_HFE_joint__ = TRAIT::cos( q[RH_HFE_JOINT]);
	s__q_RH_HFE_joint__ = TRAIT::sin( q[RH_HFE_JOINT]);

	(*this)(0, 0) = -c__q_RH_HFE_joint__;
	(*this)(0, 2) = -s__q_RH_HFE_joint__;
	(*this)(0, 3) = 0.05*s__q_RH_HFE_joint__;
	(*this)(0, 5) = -0.05*c__q_RH_HFE_joint__;
	(*this)(1, 0) = s__q_RH_HFE_joint__;
	(*this)(1, 2) = -c__q_RH_HFE_joint__;
	(*this)(1, 3) = 0.05*c__q_RH_HFE_joint__;
	(*this)(1, 5) = 0.05*s__q_RH_HFE_joint__;
	(*this)(3, 3) = -c__q_RH_HFE_joint__;
	(*this)(3, 5) = -s__q_RH_HFE_joint__;
	(*this)(4, 3) = s__q_RH_HFE_joint__;
	(*this)(4, 5) = -c__q_RH_HFE_joint__;
	return *this;
}

template <typename TRAIT>
iit::pegasus2::tpl::ForceTransforms<TRAIT>::Type_fr_RH_upperleg_X_fr_RH_lowerleg::Type_fr_RH_upperleg_X_fr_RH_lowerleg(){ 
	(*this)(0, 2) = 0;
	(*this)(0, 3) = 0;
	(*this)(0, 4) = 0;
	(*this)(0, 5) = 0;
	(*this)(1, 2) = 0;
	(*this)(1, 3) = 0;
	(*this)(1, 4) = 0;
	(*this)(1, 5) = -0.380000000000000;
	(*this)(2, 0) = 0;
	(*this)(2, 1) = 0;
	(*this)(2, 2) = 1;
	(*this)(2, 5) = 0;
	(*this)(3, 0) = 0;
	(*this)(3, 1) = 0;
	(*this)(3, 2) = 0;
	(*this)(3, 5) = 0;
	(*this)(4, 0) = 0;
	(*this)(4, 1) = 0;
	(*this)(4, 2) = 0;
	(*this)(4, 5) = 0;
	(*this)(5, 0) = 0;
	(*this)(5, 1) = 0;
	(*this)(5, 2) = 0;
	(*this)(5, 3) = 0;
	(*this)(5, 4) = 0;
	(*this)(5, 5) = 1;
}

template <typename TRAIT>
const typename iit::pegasus2::tpl::ForceTransforms<TRAIT>::Type_fr_RH_upperleg_X_fr_RH_lowerleg& 
iit::pegasus2::tpl::ForceTransforms<TRAIT>::Type_fr_RH_upperleg_X_fr_RH_lowerleg::update(const JState& q){ 
	Scalar c__q_RH_KFE_joint__;
	Scalar s__q_RH_KFE_joint__;

	c__q_RH_KFE_joint__ = TRAIT::cos( q[RH_KFE_JOINT]);
	s__q_RH_KFE_joint__ = TRAIT::sin( q[RH_KFE_JOINT]);

	(*this)(0, 0) = c__q_RH_KFE_joint__;
	(*this)(0, 1) = -s__q_RH_KFE_joint__;
	(*this)(1, 0) = s__q_RH_KFE_joint__;
	(*this)(1, 1) = c__q_RH_KFE_joint__;
	(*this)(2, 3) = 0.38*s__q_RH_KFE_joint__;
	(*this)(2, 4) = 0.38*c__q_RH_KFE_joint__;
	(*this)(3, 3) = c__q_RH_KFE_joint__;
	(*this)(3, 4) = -s__q_RH_KFE_joint__;
	(*this)(4, 3) = s__q_RH_KFE_joint__;
	(*this)(4, 4) = c__q_RH_KFE_joint__;
	return *this;
}

template <typename TRAIT>
iit::pegasus2::tpl::ForceTransforms<TRAIT>::Type_fr_RH_lowerleg_X_fr_RH_upperleg::Type_fr_RH_lowerleg_X_fr_RH_upperleg(){ 
	(*this)(0, 2) = 0;
	(*this)(0, 3) = 0;
	(*this)(0, 4) = 0;
	(*this)(1, 2) = 0;
	(*this)(1, 3) = 0;
	(*this)(1, 4) = 0;
	(*this)(2, 0) = 0;
	(*this)(2, 1) = 0;
	(*this)(2, 2) = 1;
	(*this)(2, 3) = 0;
	(*this)(2, 4) = -0.380000000000000;
	(*this)(2, 5) = 0;
	(*this)(3, 0) = 0;
	(*this)(3, 1) = 0;
	(*this)(3, 2) = 0;
	(*this)(3, 5) = 0;
	(*this)(4, 0) = 0;
	(*this)(4, 1) = 0;
	(*this)(4, 2) = 0;
	(*this)(4, 5) = 0;
	(*this)(5, 0) = 0;
	(*this)(5, 1) = 0;
	(*this)(5, 2) = 0;
	(*this)(5, 3) = 0;
	(*this)(5, 4) = 0;
	(*this)(5, 5) = 1;
}

template <typename TRAIT>
const typename iit::pegasus2::tpl::ForceTransforms<TRAIT>::Type_fr_RH_lowerleg_X_fr_RH_upperleg& 
iit::pegasus2::tpl::ForceTransforms<TRAIT>::Type_fr_RH_lowerleg_X_fr_RH_upperleg::update(const JState& q){ 
	Scalar c__q_RH_KFE_joint__;
	Scalar s__q_RH_KFE_joint__;

	c__q_RH_KFE_joint__ = TRAIT::cos( q[RH_KFE_JOINT]);
	s__q_RH_KFE_joint__ = TRAIT::sin( q[RH_KFE_JOINT]);

	(*this)(0, 0) = c__q_RH_KFE_joint__;
	(*this)(0, 1) = s__q_RH_KFE_joint__;
	(*this)(0, 5) = 0.38*s__q_RH_KFE_joint__;
	(*this)(1, 0) = -s__q_RH_KFE_joint__;
	(*this)(1, 1) = c__q_RH_KFE_joint__;
	(*this)(1, 5) = 0.38*c__q_RH_KFE_joint__;
	(*this)(3, 3) = c__q_RH_KFE_joint__;
	(*this)(3, 4) = s__q_RH_KFE_joint__;
	(*this)(4, 3) = -s__q_RH_KFE_joint__;
	(*this)(4, 4) = c__q_RH_KFE_joint__;
	return *this;
}

template <typename TRAIT>
iit::pegasus2::tpl::ForceTransforms<TRAIT>::Type_fr_LH_hip_X_fr_LH_upperleg::Type_fr_LH_hip_X_fr_LH_upperleg(){ 
	(*this)(0, 2) = 0;
	(*this)(0, 5) = 0;
	(*this)(1, 0) = 0;
	(*this)(1, 1) = 0;
	(*this)(1, 2) = -1;
	(*this)(1, 3) = 0;
	(*this)(1, 4) = 0;
	(*this)(1, 5) = 0;
	(*this)(2, 2) = 0;
	(*this)(2, 5) = 0;
	(*this)(3, 0) = 0;
	(*this)(3, 1) = 0;
	(*this)(3, 2) = 0;
	(*this)(3, 5) = 0;
	(*this)(4, 0) = 0;
	(*this)(4, 1) = 0;
	(*this)(4, 2) = 0;
	(*this)(4, 3) = 0;
	(*this)(4, 4) = 0;
	(*this)(4, 5) = -1;
	(*this)(5, 0) = 0;
	(*this)(5, 1) = 0;
	(*this)(5, 2) = 0;
	(*this)(5, 5) = 0;
}

template <typename TRAIT>
const typename iit::pegasus2::tpl::ForceTransforms<TRAIT>::Type_fr_LH_hip_X_fr_LH_upperleg& 
iit::pegasus2::tpl::ForceTransforms<TRAIT>::Type_fr_LH_hip_X_fr_LH_upperleg::update(const JState& q){ 
	Scalar c__q_LH_HFE_joint__;
	Scalar s__q_LH_HFE_joint__;

	c__q_LH_HFE_joint__ = TRAIT::cos( q[LH_HFE_JOINT]);
	s__q_LH_HFE_joint__ = TRAIT::sin( q[LH_HFE_JOINT]);

	(*this)(0, 0) = -c__q_LH_HFE_joint__;
	(*this)(0, 1) = s__q_LH_HFE_joint__;
	(*this)(0, 3) = -0.05*s__q_LH_HFE_joint__;
	(*this)(0, 4) = -0.05*c__q_LH_HFE_joint__;
	(*this)(2, 0) = -s__q_LH_HFE_joint__;
	(*this)(2, 1) = -c__q_LH_HFE_joint__;
	(*this)(2, 3) = 0.05*c__q_LH_HFE_joint__;
	(*this)(2, 4) = -0.05*s__q_LH_HFE_joint__;
	(*this)(3, 3) = -c__q_LH_HFE_joint__;
	(*this)(3, 4) = s__q_LH_HFE_joint__;
	(*this)(5, 3) = -s__q_LH_HFE_joint__;
	(*this)(5, 4) = -c__q_LH_HFE_joint__;
	return *this;
}

template <typename TRAIT>
iit::pegasus2::tpl::ForceTransforms<TRAIT>::Type_fr_LH_upperleg_X_fr_LH_hip::Type_fr_LH_upperleg_X_fr_LH_hip(){ 
	(*this)(0, 1) = 0;
	(*this)(0, 4) = 0;
	(*this)(1, 1) = 0;
	(*this)(1, 4) = 0;
	(*this)(2, 0) = 0;
	(*this)(2, 1) = -1;
	(*this)(2, 2) = 0;
	(*this)(2, 3) = 0;
	(*this)(2, 4) = 0;
	(*this)(2, 5) = 0;
	(*this)(3, 0) = 0;
	(*this)(3, 1) = 0;
	(*this)(3, 2) = 0;
	(*this)(3, 4) = 0;
	(*this)(4, 0) = 0;
	(*this)(4, 1) = 0;
	(*this)(4, 2) = 0;
	(*this)(4, 4) = 0;
	(*this)(5, 0) = 0;
	(*this)(5, 1) = 0;
	(*this)(5, 2) = 0;
	(*this)(5, 3) = 0;
	(*this)(5, 4) = -1;
	(*this)(5, 5) = 0;
}

template <typename TRAIT>
const typename iit::pegasus2::tpl::ForceTransforms<TRAIT>::Type_fr_LH_upperleg_X_fr_LH_hip& 
iit::pegasus2::tpl::ForceTransforms<TRAIT>::Type_fr_LH_upperleg_X_fr_LH_hip::update(const JState& q){ 
	Scalar c__q_LH_HFE_joint__;
	Scalar s__q_LH_HFE_joint__;

	c__q_LH_HFE_joint__ = TRAIT::cos( q[LH_HFE_JOINT]);
	s__q_LH_HFE_joint__ = TRAIT::sin( q[LH_HFE_JOINT]);

	(*this)(0, 0) = -c__q_LH_HFE_joint__;
	(*this)(0, 2) = -s__q_LH_HFE_joint__;
	(*this)(0, 3) = -0.05*s__q_LH_HFE_joint__;
	(*this)(0, 5) = 0.05*c__q_LH_HFE_joint__;
	(*this)(1, 0) = s__q_LH_HFE_joint__;
	(*this)(1, 2) = -c__q_LH_HFE_joint__;
	(*this)(1, 3) = -0.05*c__q_LH_HFE_joint__;
	(*this)(1, 5) = -0.05*s__q_LH_HFE_joint__;
	(*this)(3, 3) = -c__q_LH_HFE_joint__;
	(*this)(3, 5) = -s__q_LH_HFE_joint__;
	(*this)(4, 3) = s__q_LH_HFE_joint__;
	(*this)(4, 5) = -c__q_LH_HFE_joint__;
	return *this;
}

template <typename TRAIT>
iit::pegasus2::tpl::ForceTransforms<TRAIT>::Type_fr_LH_upperleg_X_fr_LH_lowerleg::Type_fr_LH_upperleg_X_fr_LH_lowerleg(){ 
	(*this)(0, 2) = 0;
	(*this)(0, 3) = 0;
	(*this)(0, 4) = 0;
	(*this)(0, 5) = 0;
	(*this)(1, 2) = 0;
	(*this)(1, 3) = 0;
	(*this)(1, 4) = 0;
	(*this)(1, 5) = -0.380000000000000;
	(*this)(2, 0) = 0;
	(*this)(2, 1) = 0;
	(*this)(2, 2) = 1;
	(*this)(2, 5) = 0;
	(*this)(3, 0) = 0;
	(*this)(3, 1) = 0;
	(*this)(3, 2) = 0;
	(*this)(3, 5) = 0;
	(*this)(4, 0) = 0;
	(*this)(4, 1) = 0;
	(*this)(4, 2) = 0;
	(*this)(4, 5) = 0;
	(*this)(5, 0) = 0;
	(*this)(5, 1) = 0;
	(*this)(5, 2) = 0;
	(*this)(5, 3) = 0;
	(*this)(5, 4) = 0;
	(*this)(5, 5) = 1;
}

template <typename TRAIT>
const typename iit::pegasus2::tpl::ForceTransforms<TRAIT>::Type_fr_LH_upperleg_X_fr_LH_lowerleg& 
iit::pegasus2::tpl::ForceTransforms<TRAIT>::Type_fr_LH_upperleg_X_fr_LH_lowerleg::update(const JState& q){ 
	Scalar c__q_LH_KFE_joint__;
	Scalar s__q_LH_KFE_joint__;

	c__q_LH_KFE_joint__ = TRAIT::cos( q[LH_KFE_JOINT]);
	s__q_LH_KFE_joint__ = TRAIT::sin( q[LH_KFE_JOINT]);

	(*this)(0, 0) = c__q_LH_KFE_joint__;
	(*this)(0, 1) = -s__q_LH_KFE_joint__;
	(*this)(1, 0) = s__q_LH_KFE_joint__;
	(*this)(1, 1) = c__q_LH_KFE_joint__;
	(*this)(2, 3) = 0.38*s__q_LH_KFE_joint__;
	(*this)(2, 4) = 0.38*c__q_LH_KFE_joint__;
	(*this)(3, 3) = c__q_LH_KFE_joint__;
	(*this)(3, 4) = -s__q_LH_KFE_joint__;
	(*this)(4, 3) = s__q_LH_KFE_joint__;
	(*this)(4, 4) = c__q_LH_KFE_joint__;
	return *this;
}

template <typename TRAIT>
iit::pegasus2::tpl::ForceTransforms<TRAIT>::Type_fr_LH_lowerleg_X_fr_LH_upperleg::Type_fr_LH_lowerleg_X_fr_LH_upperleg(){ 
	(*this)(0, 2) = 0;
	(*this)(0, 3) = 0;
	(*this)(0, 4) = 0;
	(*this)(1, 2) = 0;
	(*this)(1, 3) = 0;
	(*this)(1, 4) = 0;
	(*this)(2, 0) = 0;
	(*this)(2, 1) = 0;
	(*this)(2, 2) = 1;
	(*this)(2, 3) = 0;
	(*this)(2, 4) = -0.380000000000000;
	(*this)(2, 5) = 0;
	(*this)(3, 0) = 0;
	(*this)(3, 1) = 0;
	(*this)(3, 2) = 0;
	(*this)(3, 5) = 0;
	(*this)(4, 0) = 0;
	(*this)(4, 1) = 0;
	(*this)(4, 2) = 0;
	(*this)(4, 5) = 0;
	(*this)(5, 0) = 0;
	(*this)(5, 1) = 0;
	(*this)(5, 2) = 0;
	(*this)(5, 3) = 0;
	(*this)(5, 4) = 0;
	(*this)(5, 5) = 1;
}

template <typename TRAIT>
const typename iit::pegasus2::tpl::ForceTransforms<TRAIT>::Type_fr_LH_lowerleg_X_fr_LH_upperleg& 
iit::pegasus2::tpl::ForceTransforms<TRAIT>::Type_fr_LH_lowerleg_X_fr_LH_upperleg::update(const JState& q){ 
	Scalar c__q_LH_KFE_joint__;
	Scalar s__q_LH_KFE_joint__;

	c__q_LH_KFE_joint__ = TRAIT::cos( q[LH_KFE_JOINT]);
	s__q_LH_KFE_joint__ = TRAIT::sin( q[LH_KFE_JOINT]);

	(*this)(0, 0) = c__q_LH_KFE_joint__;
	(*this)(0, 1) = s__q_LH_KFE_joint__;
	(*this)(0, 5) = 0.38*s__q_LH_KFE_joint__;
	(*this)(1, 0) = -s__q_LH_KFE_joint__;
	(*this)(1, 1) = c__q_LH_KFE_joint__;
	(*this)(1, 5) = 0.38*c__q_LH_KFE_joint__;
	(*this)(3, 3) = c__q_LH_KFE_joint__;
	(*this)(3, 4) = s__q_LH_KFE_joint__;
	(*this)(4, 3) = -s__q_LH_KFE_joint__;
	(*this)(4, 4) = c__q_LH_KFE_joint__;
	return *this;
}

template <typename TRAIT>
iit::pegasus2::tpl::ForceTransforms<TRAIT>::Type_fr_LF_hip_X_fr_LF_upperleg::Type_fr_LF_hip_X_fr_LF_upperleg(){ 
	(*this)(0, 2) = 0;
	(*this)(0, 5) = 0;
	(*this)(1, 0) = 0;
	(*this)(1, 1) = 0;
	(*this)(1, 2) = -1;
	(*this)(1, 3) = 0;
	(*this)(1, 4) = 0;
	(*this)(1, 5) = 0;
	(*this)(2, 2) = 0;
	(*this)(2, 5) = 0;
	(*this)(3, 0) = 0;
	(*this)(3, 1) = 0;
	(*this)(3, 2) = 0;
	(*this)(3, 5) = 0;
	(*this)(4, 0) = 0;
	(*this)(4, 1) = 0;
	(*this)(4, 2) = 0;
	(*this)(4, 3) = 0;
	(*this)(4, 4) = 0;
	(*this)(4, 5) = -1;
	(*this)(5, 0) = 0;
	(*this)(5, 1) = 0;
	(*this)(5, 2) = 0;
	(*this)(5, 5) = 0;
}

template <typename TRAIT>
const typename iit::pegasus2::tpl::ForceTransforms<TRAIT>::Type_fr_LF_hip_X_fr_LF_upperleg& 
iit::pegasus2::tpl::ForceTransforms<TRAIT>::Type_fr_LF_hip_X_fr_LF_upperleg::update(const JState& q){ 
	Scalar c__q_LF_HFE_joint__;
	Scalar s__q_LF_HFE_joint__;

	c__q_LF_HFE_joint__ = TRAIT::cos( q[LF_HFE_JOINT]);
	s__q_LF_HFE_joint__ = TRAIT::sin( q[LF_HFE_JOINT]);

	(*this)(0, 0) = -c__q_LF_HFE_joint__;
	(*this)(0, 1) = s__q_LF_HFE_joint__;
	(*this)(0, 3) = -0.05*s__q_LF_HFE_joint__;
	(*this)(0, 4) = -0.05*c__q_LF_HFE_joint__;
	(*this)(2, 0) = -s__q_LF_HFE_joint__;
	(*this)(2, 1) = -c__q_LF_HFE_joint__;
	(*this)(2, 3) = 0.05*c__q_LF_HFE_joint__;
	(*this)(2, 4) = -0.05*s__q_LF_HFE_joint__;
	(*this)(3, 3) = -c__q_LF_HFE_joint__;
	(*this)(3, 4) = s__q_LF_HFE_joint__;
	(*this)(5, 3) = -s__q_LF_HFE_joint__;
	(*this)(5, 4) = -c__q_LF_HFE_joint__;
	return *this;
}

template <typename TRAIT>
iit::pegasus2::tpl::ForceTransforms<TRAIT>::Type_fr_LF_upperleg_X_fr_LF_hip::Type_fr_LF_upperleg_X_fr_LF_hip(){ 
	(*this)(0, 1) = 0;
	(*this)(0, 4) = 0;
	(*this)(1, 1) = 0;
	(*this)(1, 4) = 0;
	(*this)(2, 0) = 0;
	(*this)(2, 1) = -1;
	(*this)(2, 2) = 0;
	(*this)(2, 3) = 0;
	(*this)(2, 4) = 0;
	(*this)(2, 5) = 0;
	(*this)(3, 0) = 0;
	(*this)(3, 1) = 0;
	(*this)(3, 2) = 0;
	(*this)(3, 4) = 0;
	(*this)(4, 0) = 0;
	(*this)(4, 1) = 0;
	(*this)(4, 2) = 0;
	(*this)(4, 4) = 0;
	(*this)(5, 0) = 0;
	(*this)(5, 1) = 0;
	(*this)(5, 2) = 0;
	(*this)(5, 3) = 0;
	(*this)(5, 4) = -1;
	(*this)(5, 5) = 0;
}

template <typename TRAIT>
const typename iit::pegasus2::tpl::ForceTransforms<TRAIT>::Type_fr_LF_upperleg_X_fr_LF_hip& 
iit::pegasus2::tpl::ForceTransforms<TRAIT>::Type_fr_LF_upperleg_X_fr_LF_hip::update(const JState& q){ 
	Scalar c__q_LF_HFE_joint__;
	Scalar s__q_LF_HFE_joint__;

	c__q_LF_HFE_joint__ = TRAIT::cos( q[LF_HFE_JOINT]);
	s__q_LF_HFE_joint__ = TRAIT::sin( q[LF_HFE_JOINT]);

	(*this)(0, 0) = -c__q_LF_HFE_joint__;
	(*this)(0, 2) = -s__q_LF_HFE_joint__;
	(*this)(0, 3) = -0.05*s__q_LF_HFE_joint__;
	(*this)(0, 5) = 0.05*c__q_LF_HFE_joint__;
	(*this)(1, 0) = s__q_LF_HFE_joint__;
	(*this)(1, 2) = -c__q_LF_HFE_joint__;
	(*this)(1, 3) = -0.05*c__q_LF_HFE_joint__;
	(*this)(1, 5) = -0.05*s__q_LF_HFE_joint__;
	(*this)(3, 3) = -c__q_LF_HFE_joint__;
	(*this)(3, 5) = -s__q_LF_HFE_joint__;
	(*this)(4, 3) = s__q_LF_HFE_joint__;
	(*this)(4, 5) = -c__q_LF_HFE_joint__;
	return *this;
}

template <typename TRAIT>
iit::pegasus2::tpl::ForceTransforms<TRAIT>::Type_fr_LF_upperleg_X_fr_LF_lowerleg::Type_fr_LF_upperleg_X_fr_LF_lowerleg(){ 
	(*this)(0, 2) = 0;
	(*this)(0, 3) = 0;
	(*this)(0, 4) = 0;
	(*this)(0, 5) = 0;
	(*this)(1, 2) = 0;
	(*this)(1, 3) = 0;
	(*this)(1, 4) = 0;
	(*this)(1, 5) = -0.380000000000000;
	(*this)(2, 0) = 0;
	(*this)(2, 1) = 0;
	(*this)(2, 2) = 1;
	(*this)(2, 5) = 0;
	(*this)(3, 0) = 0;
	(*this)(3, 1) = 0;
	(*this)(3, 2) = 0;
	(*this)(3, 5) = 0;
	(*this)(4, 0) = 0;
	(*this)(4, 1) = 0;
	(*this)(4, 2) = 0;
	(*this)(4, 5) = 0;
	(*this)(5, 0) = 0;
	(*this)(5, 1) = 0;
	(*this)(5, 2) = 0;
	(*this)(5, 3) = 0;
	(*this)(5, 4) = 0;
	(*this)(5, 5) = 1;
}

template <typename TRAIT>
const typename iit::pegasus2::tpl::ForceTransforms<TRAIT>::Type_fr_LF_upperleg_X_fr_LF_lowerleg& 
iit::pegasus2::tpl::ForceTransforms<TRAIT>::Type_fr_LF_upperleg_X_fr_LF_lowerleg::update(const JState& q){ 
	Scalar c__q_LF_KFE_joint__;
	Scalar s__q_LF_KFE_joint__;

	c__q_LF_KFE_joint__ = TRAIT::cos( q[LF_KFE_JOINT]);
	s__q_LF_KFE_joint__ = TRAIT::sin( q[LF_KFE_JOINT]);

	(*this)(0, 0) = c__q_LF_KFE_joint__;
	(*this)(0, 1) = -s__q_LF_KFE_joint__;
	(*this)(1, 0) = s__q_LF_KFE_joint__;
	(*this)(1, 1) = c__q_LF_KFE_joint__;
	(*this)(2, 3) = 0.38*s__q_LF_KFE_joint__;
	(*this)(2, 4) = 0.38*c__q_LF_KFE_joint__;
	(*this)(3, 3) = c__q_LF_KFE_joint__;
	(*this)(3, 4) = -s__q_LF_KFE_joint__;
	(*this)(4, 3) = s__q_LF_KFE_joint__;
	(*this)(4, 4) = c__q_LF_KFE_joint__;
	return *this;
}

template <typename TRAIT>
iit::pegasus2::tpl::ForceTransforms<TRAIT>::Type_fr_LF_lowerleg_X_fr_LF_upperleg::Type_fr_LF_lowerleg_X_fr_LF_upperleg(){ 
	(*this)(0, 2) = 0;
	(*this)(0, 3) = 0;
	(*this)(0, 4) = 0;
	(*this)(1, 2) = 0;
	(*this)(1, 3) = 0;
	(*this)(1, 4) = 0;
	(*this)(2, 0) = 0;
	(*this)(2, 1) = 0;
	(*this)(2, 2) = 1;
	(*this)(2, 3) = 0;
	(*this)(2, 4) = -0.380000000000000;
	(*this)(2, 5) = 0;
	(*this)(3, 0) = 0;
	(*this)(3, 1) = 0;
	(*this)(3, 2) = 0;
	(*this)(3, 5) = 0;
	(*this)(4, 0) = 0;
	(*this)(4, 1) = 0;
	(*this)(4, 2) = 0;
	(*this)(4, 5) = 0;
	(*this)(5, 0) = 0;
	(*this)(5, 1) = 0;
	(*this)(5, 2) = 0;
	(*this)(5, 3) = 0;
	(*this)(5, 4) = 0;
	(*this)(5, 5) = 1;
}

template <typename TRAIT>
const typename iit::pegasus2::tpl::ForceTransforms<TRAIT>::Type_fr_LF_lowerleg_X_fr_LF_upperleg& 
iit::pegasus2::tpl::ForceTransforms<TRAIT>::Type_fr_LF_lowerleg_X_fr_LF_upperleg::update(const JState& q){ 
	Scalar c__q_LF_KFE_joint__;
	Scalar s__q_LF_KFE_joint__;

	c__q_LF_KFE_joint__ = TRAIT::cos( q[LF_KFE_JOINT]);
	s__q_LF_KFE_joint__ = TRAIT::sin( q[LF_KFE_JOINT]);

	(*this)(0, 0) = c__q_LF_KFE_joint__;
	(*this)(0, 1) = s__q_LF_KFE_joint__;
	(*this)(0, 5) = 0.38*s__q_LF_KFE_joint__;
	(*this)(1, 0) = -s__q_LF_KFE_joint__;
	(*this)(1, 1) = c__q_LF_KFE_joint__;
	(*this)(1, 5) = 0.38*c__q_LF_KFE_joint__;
	(*this)(3, 3) = c__q_LF_KFE_joint__;
	(*this)(3, 4) = s__q_LF_KFE_joint__;
	(*this)(4, 3) = -s__q_LF_KFE_joint__;
	(*this)(4, 4) = c__q_LF_KFE_joint__;
	return *this;
}

template <typename TRAIT>
iit::pegasus2::tpl::ForceTransforms<TRAIT>::Type_fr_torso_X_fr_RF_hip::Type_fr_torso_X_fr_RF_hip(){ 
	(*this)(0, 0) = 0;
	(*this)(0, 1) = 0;
	(*this)(0, 2) = -1;
	(*this)(0, 5) = 0;
	(*this)(1, 2) = 0;
	(*this)(1, 5) = 0;
	(*this)(2, 2) = 0;
	(*this)(2, 5) = -0.118500000000000;
	(*this)(3, 0) = 0;
	(*this)(3, 1) = 0;
	(*this)(3, 2) = 0;
	(*this)(3, 3) = 0;
	(*this)(3, 4) = 0;
	(*this)(3, 5) = -1;
	(*this)(4, 0) = 0;
	(*this)(4, 1) = 0;
	(*this)(4, 2) = 0;
	(*this)(4, 5) = 0;
	(*this)(5, 0) = 0;
	(*this)(5, 1) = 0;
	(*this)(5, 2) = 0;
	(*this)(5, 5) = 0;
}

template <typename TRAIT>
const typename iit::pegasus2::tpl::ForceTransforms<TRAIT>::Type_fr_torso_X_fr_RF_hip& 
iit::pegasus2::tpl::ForceTransforms<TRAIT>::Type_fr_torso_X_fr_RF_hip::update(const JState& q){ 
	Scalar c__q_RF_HAA_joint__;
	Scalar s__q_RF_HAA_joint__;

	c__q_RF_HAA_joint__ = TRAIT::cos( q[RF_HAA_JOINT]);
	s__q_RF_HAA_joint__ = TRAIT::sin( q[RF_HAA_JOINT]);

	(*this)(0, 3) = -0.1185*c__q_RF_HAA_joint__;
	(*this)(0, 4) = 0.1185*s__q_RF_HAA_joint__;
	(*this)(1, 0) = s__q_RF_HAA_joint__;
	(*this)(1, 1) = c__q_RF_HAA_joint__;
	(*this)(1, 3) = -0.37*c__q_RF_HAA_joint__;
	(*this)(1, 4) = 0.37*s__q_RF_HAA_joint__;
	(*this)(2, 0) = c__q_RF_HAA_joint__;
	(*this)(2, 1) = -s__q_RF_HAA_joint__;
	(*this)(2, 3) = 0.37*s__q_RF_HAA_joint__;
	(*this)(2, 4) = 0.37*c__q_RF_HAA_joint__;
	(*this)(4, 3) = s__q_RF_HAA_joint__;
	(*this)(4, 4) = c__q_RF_HAA_joint__;
	(*this)(5, 3) = c__q_RF_HAA_joint__;
	(*this)(5, 4) = -s__q_RF_HAA_joint__;
	return *this;
}

template <typename TRAIT>
iit::pegasus2::tpl::ForceTransforms<TRAIT>::Type_fr_RF_hip_X_fr_torso::Type_fr_RF_hip_X_fr_torso(){ 
	(*this)(0, 0) = 0;
	(*this)(1, 0) = 0;
	(*this)(2, 0) = -1;
	(*this)(2, 1) = 0;
	(*this)(2, 2) = 0;
	(*this)(2, 3) = 0;
	(*this)(2, 4) = 0;
	(*this)(2, 5) = -0.118500000000000;
	(*this)(3, 0) = 0;
	(*this)(3, 1) = 0;
	(*this)(3, 2) = 0;
	(*this)(3, 3) = 0;
	(*this)(4, 0) = 0;
	(*this)(4, 1) = 0;
	(*this)(4, 2) = 0;
	(*this)(4, 3) = 0;
	(*this)(5, 0) = 0;
	(*this)(5, 1) = 0;
	(*this)(5, 2) = 0;
	(*this)(5, 3) = -1;
	(*this)(5, 4) = 0;
	(*this)(5, 5) = 0;
}

template <typename TRAIT>
const typename iit::pegasus2::tpl::ForceTransforms<TRAIT>::Type_fr_RF_hip_X_fr_torso& 
iit::pegasus2::tpl::ForceTransforms<TRAIT>::Type_fr_RF_hip_X_fr_torso::update(const JState& q){ 
	Scalar c__q_RF_HAA_joint__;
	Scalar s__q_RF_HAA_joint__;

	c__q_RF_HAA_joint__ = TRAIT::cos( q[RF_HAA_JOINT]);
	s__q_RF_HAA_joint__ = TRAIT::sin( q[RF_HAA_JOINT]);

	(*this)(0, 1) = s__q_RF_HAA_joint__;
	(*this)(0, 2) = c__q_RF_HAA_joint__;
	(*this)(0, 3) = -0.1185*c__q_RF_HAA_joint__;
	(*this)(0, 4) = -0.37*c__q_RF_HAA_joint__;
	(*this)(0, 5) = 0.37*s__q_RF_HAA_joint__;
	(*this)(1, 1) = c__q_RF_HAA_joint__;
	(*this)(1, 2) = -s__q_RF_HAA_joint__;
	(*this)(1, 3) = 0.1185*s__q_RF_HAA_joint__;
	(*this)(1, 4) = 0.37*s__q_RF_HAA_joint__;
	(*this)(1, 5) = 0.37*c__q_RF_HAA_joint__;
	(*this)(3, 4) = s__q_RF_HAA_joint__;
	(*this)(3, 5) = c__q_RF_HAA_joint__;
	(*this)(4, 4) = c__q_RF_HAA_joint__;
	(*this)(4, 5) = -s__q_RF_HAA_joint__;
	return *this;
}

template <typename TRAIT>
iit::pegasus2::tpl::ForceTransforms<TRAIT>::Type_fr_torso_X_fr_RF_upperleg::Type_fr_torso_X_fr_RF_upperleg(){ 
	(*this)(0, 2) = 0;
	(*this)(3, 0) = 0;
	(*this)(3, 1) = 0;
	(*this)(3, 2) = 0;
	(*this)(3, 5) = 0;
	(*this)(4, 0) = 0;
	(*this)(4, 1) = 0;
	(*this)(4, 2) = 0;
	(*this)(5, 0) = 0;
	(*this)(5, 1) = 0;
	(*this)(5, 2) = 0;
}

template <typename TRAIT>
const typename iit::pegasus2::tpl::ForceTransforms<TRAIT>::Type_fr_torso_X_fr_RF_upperleg& 
iit::pegasus2::tpl::ForceTransforms<TRAIT>::Type_fr_torso_X_fr_RF_upperleg::update(const JState& q){ 
	Scalar c__q_RF_HAA_joint__;
	Scalar c__q_RF_HFE_joint__;
	Scalar s__q_RF_HFE_joint__;
	Scalar s__q_RF_HAA_joint__;

	c__q_RF_HAA_joint__ = TRAIT::cos( q[RF_HAA_JOINT]);
	c__q_RF_HFE_joint__ = TRAIT::cos( q[RF_HFE_JOINT]);
	s__q_RF_HFE_joint__ = TRAIT::sin( q[RF_HFE_JOINT]);
	s__q_RF_HAA_joint__ = TRAIT::sin( q[RF_HAA_JOINT]);

	(*this)(0, 0) = s__q_RF_HFE_joint__;
	(*this)(0, 1) = c__q_RF_HFE_joint__;
	(*this)(0, 3) = (0.1185*c__q_RF_HAA_joint__ + 0.05)*c__q_RF_HFE_joint__;
	(*this)(0, 4) = -(0.1185*c__q_RF_HAA_joint__ + 0.05)*s__q_RF_HFE_joint__;
	(*this)(0, 5) = -0.1185*s__q_RF_HAA_joint__;
	(*this)(1, 0) = -s__q_RF_HAA_joint__*c__q_RF_HFE_joint__;
	(*this)(1, 1) = s__q_RF_HAA_joint__*s__q_RF_HFE_joint__;
	(*this)(1, 2) = -c__q_RF_HAA_joint__;
	(*this)(1, 3) = 0.05*s__q_RF_HAA_joint__*s__q_RF_HFE_joint__ + 0.37*c__q_RF_HAA_joint__*c__q_RF_HFE_joint__;
	(*this)(1, 4) = 0.05*s__q_RF_HAA_joint__*c__q_RF_HFE_joint__ - 0.37*s__q_RF_HFE_joint__*c__q_RF_HAA_joint__;
	(*this)(1, 5) = -0.37*s__q_RF_HAA_joint__;
	(*this)(2, 0) = -c__q_RF_HAA_joint__*c__q_RF_HFE_joint__;
	(*this)(2, 1) = s__q_RF_HFE_joint__*c__q_RF_HAA_joint__;
	(*this)(2, 2) = s__q_RF_HAA_joint__;
	(*this)(2, 3) = -0.37*s__q_RF_HAA_joint__*c__q_RF_HFE_joint__ + 0.05*s__q_RF_HFE_joint__*c__q_RF_HAA_joint__ + 0.1185*s__q_RF_HFE_joint__;
	(*this)(2, 4) = 0.37*s__q_RF_HAA_joint__*s__q_RF_HFE_joint__ + 0.05*c__q_RF_HAA_joint__*c__q_RF_HFE_joint__ + 0.1185*c__q_RF_HFE_joint__;
	(*this)(2, 5) = -0.37*c__q_RF_HAA_joint__;
	(*this)(3, 3) = s__q_RF_HFE_joint__;
	(*this)(3, 4) = c__q_RF_HFE_joint__;
	(*this)(4, 3) = -s__q_RF_HAA_joint__*c__q_RF_HFE_joint__;
	(*this)(4, 4) = s__q_RF_HAA_joint__*s__q_RF_HFE_joint__;
	(*this)(4, 5) = -c__q_RF_HAA_joint__;
	(*this)(5, 3) = -c__q_RF_HAA_joint__*c__q_RF_HFE_joint__;
	(*this)(5, 4) = s__q_RF_HFE_joint__*c__q_RF_HAA_joint__;
	(*this)(5, 5) = s__q_RF_HAA_joint__;
	return *this;
}

template <typename TRAIT>
iit::pegasus2::tpl::ForceTransforms<TRAIT>::Type_fr_RF_upperleg_X_fr_torso::Type_fr_RF_upperleg_X_fr_torso(){ 
	(*this)(2, 0) = 0;
	(*this)(3, 0) = 0;
	(*this)(3, 1) = 0;
	(*this)(3, 2) = 0;
	(*this)(4, 0) = 0;
	(*this)(4, 1) = 0;
	(*this)(4, 2) = 0;
	(*this)(5, 0) = 0;
	(*this)(5, 1) = 0;
	(*this)(5, 2) = 0;
	(*this)(5, 3) = 0;
}

template <typename TRAIT>
const typename iit::pegasus2::tpl::ForceTransforms<TRAIT>::Type_fr_RF_upperleg_X_fr_torso& 
iit::pegasus2::tpl::ForceTransforms<TRAIT>::Type_fr_RF_upperleg_X_fr_torso::update(const JState& q){ 
	Scalar c__q_RF_HAA_joint__;
	Scalar c__q_RF_HFE_joint__;
	Scalar s__q_RF_HFE_joint__;
	Scalar s__q_RF_HAA_joint__;

	c__q_RF_HAA_joint__ = TRAIT::cos( q[RF_HAA_JOINT]);
	c__q_RF_HFE_joint__ = TRAIT::cos( q[RF_HFE_JOINT]);
	s__q_RF_HFE_joint__ = TRAIT::sin( q[RF_HFE_JOINT]);
	s__q_RF_HAA_joint__ = TRAIT::sin( q[RF_HAA_JOINT]);

	(*this)(0, 0) = s__q_RF_HFE_joint__;
	(*this)(0, 1) = -s__q_RF_HAA_joint__*c__q_RF_HFE_joint__;
	(*this)(0, 2) = -c__q_RF_HAA_joint__*c__q_RF_HFE_joint__;
	(*this)(0, 3) = (0.1185*c__q_RF_HAA_joint__ + 0.05)*c__q_RF_HFE_joint__;
	(*this)(0, 4) = 0.05*s__q_RF_HAA_joint__*s__q_RF_HFE_joint__ + 0.37*c__q_RF_HAA_joint__*c__q_RF_HFE_joint__;
	(*this)(0, 5) = -0.37*s__q_RF_HAA_joint__*c__q_RF_HFE_joint__ + 0.05*s__q_RF_HFE_joint__*c__q_RF_HAA_joint__ + 0.1185*s__q_RF_HFE_joint__;
	(*this)(1, 0) = c__q_RF_HFE_joint__;
	(*this)(1, 1) = s__q_RF_HAA_joint__*s__q_RF_HFE_joint__;
	(*this)(1, 2) = s__q_RF_HFE_joint__*c__q_RF_HAA_joint__;
	(*this)(1, 3) = -(0.1185*c__q_RF_HAA_joint__ + 0.05)*s__q_RF_HFE_joint__;
	(*this)(1, 4) = 0.05*s__q_RF_HAA_joint__*c__q_RF_HFE_joint__ - 0.37*s__q_RF_HFE_joint__*c__q_RF_HAA_joint__;
	(*this)(1, 5) = 0.37*s__q_RF_HAA_joint__*s__q_RF_HFE_joint__ + 0.05*c__q_RF_HAA_joint__*c__q_RF_HFE_joint__ + 0.1185*c__q_RF_HFE_joint__;
	(*this)(2, 1) = -c__q_RF_HAA_joint__;
	(*this)(2, 2) = s__q_RF_HAA_joint__;
	(*this)(2, 3) = -0.1185*s__q_RF_HAA_joint__;
	(*this)(2, 4) = -0.37*s__q_RF_HAA_joint__;
	(*this)(2, 5) = -0.37*c__q_RF_HAA_joint__;
	(*this)(3, 3) = s__q_RF_HFE_joint__;
	(*this)(3, 4) = -s__q_RF_HAA_joint__*c__q_RF_HFE_joint__;
	(*this)(3, 5) = -c__q_RF_HAA_joint__*c__q_RF_HFE_joint__;
	(*this)(4, 3) = c__q_RF_HFE_joint__;
	(*this)(4, 4) = s__q_RF_HAA_joint__*s__q_RF_HFE_joint__;
	(*this)(4, 5) = s__q_RF_HFE_joint__*c__q_RF_HAA_joint__;
	(*this)(5, 4) = -c__q_RF_HAA_joint__;
	(*this)(5, 5) = s__q_RF_HAA_joint__;
	return *this;
}

template <typename TRAIT>
iit::pegasus2::tpl::ForceTransforms<TRAIT>::Type_fr_torso_X_fr_RF_lowerleg::Type_fr_torso_X_fr_RF_lowerleg(){ 
	(*this)(0, 2) = 0;
	(*this)(3, 0) = 0;
	(*this)(3, 1) = 0;
	(*this)(3, 2) = 0;
	(*this)(3, 5) = 0;
	(*this)(4, 0) = 0;
	(*this)(4, 1) = 0;
	(*this)(4, 2) = 0;
	(*this)(5, 0) = 0;
	(*this)(5, 1) = 0;
	(*this)(5, 2) = 0;
}

template <typename TRAIT>
const typename iit::pegasus2::tpl::ForceTransforms<TRAIT>::Type_fr_torso_X_fr_RF_lowerleg& 
iit::pegasus2::tpl::ForceTransforms<TRAIT>::Type_fr_torso_X_fr_RF_lowerleg::update(const JState& q){ 
	Scalar c__q_RF_KFE_joint__;
	Scalar c__q_RF_HAA_joint__;
	Scalar c__q_RF_HFE_joint__;
	Scalar s__q_RF_HFE_joint__;
	Scalar s__q_RF_HAA_joint__;
	Scalar s__q_RF_KFE_joint__;

	c__q_RF_KFE_joint__ = TRAIT::cos( q[RF_KFE_JOINT]);
	c__q_RF_HAA_joint__ = TRAIT::cos( q[RF_HAA_JOINT]);
	c__q_RF_HFE_joint__ = TRAIT::cos( q[RF_HFE_JOINT]);
	s__q_RF_HFE_joint__ = TRAIT::sin( q[RF_HFE_JOINT]);
	s__q_RF_HAA_joint__ = TRAIT::sin( q[RF_HAA_JOINT]);
	s__q_RF_KFE_joint__ = TRAIT::sin( q[RF_KFE_JOINT]);

	(*this)(0, 0) = s__q_RF_HFE_joint__*c__q_RF_KFE_joint__ + s__q_RF_KFE_joint__*c__q_RF_HFE_joint__;
	(*this)(0, 1) = -s__q_RF_HFE_joint__*s__q_RF_KFE_joint__ + c__q_RF_HFE_joint__*c__q_RF_KFE_joint__;
	(*this)(0, 3) = (-s__q_RF_HFE_joint__*s__q_RF_KFE_joint__ + c__q_RF_HFE_joint__*c__q_RF_KFE_joint__)*(0.1185*c__q_RF_HAA_joint__ + 0.05);
	(*this)(0, 4) = -(s__q_RF_HFE_joint__*c__q_RF_KFE_joint__ + s__q_RF_KFE_joint__*c__q_RF_HFE_joint__)*(0.1185*c__q_RF_HAA_joint__ + 0.05);
	(*this)(0, 5) = -0.1185*s__q_RF_HAA_joint__ - 0.38*c__q_RF_HFE_joint__;
	(*this)(1, 0) = -(-s__q_RF_HFE_joint__*s__q_RF_KFE_joint__ + c__q_RF_HFE_joint__*c__q_RF_KFE_joint__)*s__q_RF_HAA_joint__;
	(*this)(1, 1) = (s__q_RF_HFE_joint__*c__q_RF_KFE_joint__ + s__q_RF_KFE_joint__*c__q_RF_HFE_joint__)*s__q_RF_HAA_joint__;
	(*this)(1, 2) = -c__q_RF_HAA_joint__;
	(*this)(1, 3) = 0.21*(s__q_RF_HAA_joint__*s__q_RF_KFE_joint__ + c__q_RF_HAA_joint__*c__q_RF_KFE_joint__)*c__q_RF_HFE_joint__ - 0.21*(-s__q_RF_HAA_joint__*c__q_RF_KFE_joint__ + s__q_RF_KFE_joint__*c__q_RF_HAA_joint__)*s__q_RF_HFE_joint__ + 0.16*(-s__q_RF_HFE_joint__*s__q_RF_KFE_joint__ + c__q_RF_HFE_joint__*c__q_RF_KFE_joint__)*c__q_RF_HAA_joint__ - 0.16*(s__q_RF_HFE_joint__*c__q_RF_KFE_joint__ + s__q_RF_KFE_joint__*c__q_RF_HFE_joint__)*s__q_RF_HAA_joint__ - 0.38*s__q_RF_KFE_joint__*c__q_RF_HAA_joint__;
	(*this)(1, 4) = -0.21*(s__q_RF_HAA_joint__*s__q_RF_KFE_joint__ + c__q_RF_HAA_joint__*c__q_RF_KFE_joint__)*s__q_RF_HFE_joint__ - 0.21*(-s__q_RF_HAA_joint__*c__q_RF_KFE_joint__ + s__q_RF_KFE_joint__*c__q_RF_HAA_joint__)*c__q_RF_HFE_joint__ - 0.16*(-s__q_RF_HFE_joint__*s__q_RF_KFE_joint__ + c__q_RF_HFE_joint__*c__q_RF_KFE_joint__)*s__q_RF_HAA_joint__ - 0.16*(s__q_RF_HFE_joint__*c__q_RF_KFE_joint__ + s__q_RF_KFE_joint__*c__q_RF_HFE_joint__)*c__q_RF_HAA_joint__ - 0.38*c__q_RF_HAA_joint__*c__q_RF_KFE_joint__;
	(*this)(1, 5) = -(0.38*s__q_RF_HFE_joint__ + 0.37)*s__q_RF_HAA_joint__;
	(*this)(2, 0) = -(-s__q_RF_HFE_joint__*s__q_RF_KFE_joint__ + c__q_RF_HFE_joint__*c__q_RF_KFE_joint__)*c__q_RF_HAA_joint__;
	(*this)(2, 1) = (s__q_RF_HFE_joint__*c__q_RF_KFE_joint__ + s__q_RF_KFE_joint__*c__q_RF_HFE_joint__)*c__q_RF_HAA_joint__;
	(*this)(2, 2) = s__q_RF_HAA_joint__;
	(*this)(2, 3) = -0.37*(-s__q_RF_HFE_joint__*s__q_RF_KFE_joint__ + c__q_RF_HFE_joint__*c__q_RF_KFE_joint__)*s__q_RF_HAA_joint__ + 0.05*(s__q_RF_HFE_joint__*c__q_RF_KFE_joint__ + s__q_RF_KFE_joint__*c__q_RF_HFE_joint__)*c__q_RF_HAA_joint__ + 0.38*s__q_RF_HAA_joint__*s__q_RF_KFE_joint__ + 0.1185*s__q_RF_HFE_joint__*c__q_RF_KFE_joint__ + 0.1185*s__q_RF_KFE_joint__*c__q_RF_HFE_joint__;
	(*this)(2, 4) = 0.05*(-s__q_RF_HFE_joint__*s__q_RF_KFE_joint__ + c__q_RF_HFE_joint__*c__q_RF_KFE_joint__)*c__q_RF_HAA_joint__ + 0.37*(s__q_RF_HFE_joint__*c__q_RF_KFE_joint__ + s__q_RF_KFE_joint__*c__q_RF_HFE_joint__)*s__q_RF_HAA_joint__ + 0.38*s__q_RF_HAA_joint__*c__q_RF_KFE_joint__ - 0.1185*s__q_RF_HFE_joint__*s__q_RF_KFE_joint__ + 0.1185*c__q_RF_HFE_joint__*c__q_RF_KFE_joint__;
	(*this)(2, 5) = -(0.38*s__q_RF_HFE_joint__ + 0.37)*c__q_RF_HAA_joint__;
	(*this)(3, 3) = s__q_RF_HFE_joint__*c__q_RF_KFE_joint__ + s__q_RF_KFE_joint__*c__q_RF_HFE_joint__;
	(*this)(3, 4) = -s__q_RF_HFE_joint__*s__q_RF_KFE_joint__ + c__q_RF_HFE_joint__*c__q_RF_KFE_joint__;
	(*this)(4, 3) = -(-s__q_RF_HFE_joint__*s__q_RF_KFE_joint__ + c__q_RF_HFE_joint__*c__q_RF_KFE_joint__)*s__q_RF_HAA_joint__;
	(*this)(4, 4) = (s__q_RF_HFE_joint__*c__q_RF_KFE_joint__ + s__q_RF_KFE_joint__*c__q_RF_HFE_joint__)*s__q_RF_HAA_joint__;
	(*this)(4, 5) = -c__q_RF_HAA_joint__;
	(*this)(5, 3) = -(-s__q_RF_HFE_joint__*s__q_RF_KFE_joint__ + c__q_RF_HFE_joint__*c__q_RF_KFE_joint__)*c__q_RF_HAA_joint__;
	(*this)(5, 4) = (s__q_RF_HFE_joint__*c__q_RF_KFE_joint__ + s__q_RF_KFE_joint__*c__q_RF_HFE_joint__)*c__q_RF_HAA_joint__;
	(*this)(5, 5) = s__q_RF_HAA_joint__;
	return *this;
}

template <typename TRAIT>
iit::pegasus2::tpl::ForceTransforms<TRAIT>::Type_fr_RF_lowerleg_X_fr_torso::Type_fr_RF_lowerleg_X_fr_torso(){ 
	(*this)(2, 0) = 0;
	(*this)(3, 0) = 0;
	(*this)(3, 1) = 0;
	(*this)(3, 2) = 0;
	(*this)(4, 0) = 0;
	(*this)(4, 1) = 0;
	(*this)(4, 2) = 0;
	(*this)(5, 0) = 0;
	(*this)(5, 1) = 0;
	(*this)(5, 2) = 0;
	(*this)(5, 3) = 0;
}

template <typename TRAIT>
const typename iit::pegasus2::tpl::ForceTransforms<TRAIT>::Type_fr_RF_lowerleg_X_fr_torso& 
iit::pegasus2::tpl::ForceTransforms<TRAIT>::Type_fr_RF_lowerleg_X_fr_torso::update(const JState& q){ 
	Scalar c__q_RF_KFE_joint__;
	Scalar c__q_RF_HAA_joint__;
	Scalar c__q_RF_HFE_joint__;
	Scalar s__q_RF_HFE_joint__;
	Scalar s__q_RF_HAA_joint__;
	Scalar s__q_RF_KFE_joint__;

	c__q_RF_KFE_joint__ = TRAIT::cos( q[RF_KFE_JOINT]);
	c__q_RF_HAA_joint__ = TRAIT::cos( q[RF_HAA_JOINT]);
	c__q_RF_HFE_joint__ = TRAIT::cos( q[RF_HFE_JOINT]);
	s__q_RF_HFE_joint__ = TRAIT::sin( q[RF_HFE_JOINT]);
	s__q_RF_HAA_joint__ = TRAIT::sin( q[RF_HAA_JOINT]);
	s__q_RF_KFE_joint__ = TRAIT::sin( q[RF_KFE_JOINT]);

	(*this)(0, 0) = s__q_RF_HFE_joint__*c__q_RF_KFE_joint__ + s__q_RF_KFE_joint__*c__q_RF_HFE_joint__;
	(*this)(0, 1) = -(-s__q_RF_HFE_joint__*s__q_RF_KFE_joint__ + c__q_RF_HFE_joint__*c__q_RF_KFE_joint__)*s__q_RF_HAA_joint__;
	(*this)(0, 2) = -(-s__q_RF_HFE_joint__*s__q_RF_KFE_joint__ + c__q_RF_HFE_joint__*c__q_RF_KFE_joint__)*c__q_RF_HAA_joint__;
	(*this)(0, 3) = (-s__q_RF_HFE_joint__*s__q_RF_KFE_joint__ + c__q_RF_HFE_joint__*c__q_RF_KFE_joint__)*(0.1185*c__q_RF_HAA_joint__ + 0.05);
	(*this)(0, 4) = 0.37*(-s__q_RF_HFE_joint__*s__q_RF_KFE_joint__ + c__q_RF_HFE_joint__*c__q_RF_KFE_joint__)*c__q_RF_HAA_joint__ + 0.05*(s__q_RF_HFE_joint__*c__q_RF_KFE_joint__ + s__q_RF_KFE_joint__*c__q_RF_HFE_joint__)*s__q_RF_HAA_joint__ - 0.38*s__q_RF_KFE_joint__*c__q_RF_HAA_joint__;
	(*this)(0, 5) = -0.37*(-s__q_RF_HFE_joint__*s__q_RF_KFE_joint__ + c__q_RF_HFE_joint__*c__q_RF_KFE_joint__)*s__q_RF_HAA_joint__ + 0.05*(s__q_RF_HFE_joint__*c__q_RF_KFE_joint__ + s__q_RF_KFE_joint__*c__q_RF_HFE_joint__)*c__q_RF_HAA_joint__ + 0.38*s__q_RF_HAA_joint__*s__q_RF_KFE_joint__ + 0.1185*s__q_RF_HFE_joint__*c__q_RF_KFE_joint__ + 0.1185*s__q_RF_KFE_joint__*c__q_RF_HFE_joint__;
	(*this)(1, 0) = -s__q_RF_HFE_joint__*s__q_RF_KFE_joint__ + c__q_RF_HFE_joint__*c__q_RF_KFE_joint__;
	(*this)(1, 1) = (s__q_RF_HFE_joint__*c__q_RF_KFE_joint__ + s__q_RF_KFE_joint__*c__q_RF_HFE_joint__)*s__q_RF_HAA_joint__;
	(*this)(1, 2) = (s__q_RF_HFE_joint__*c__q_RF_KFE_joint__ + s__q_RF_KFE_joint__*c__q_RF_HFE_joint__)*c__q_RF_HAA_joint__;
	(*this)(1, 3) = -(s__q_RF_HFE_joint__*c__q_RF_KFE_joint__ + s__q_RF_KFE_joint__*c__q_RF_HFE_joint__)*(0.1185*c__q_RF_HAA_joint__ + 0.05);
	(*this)(1, 4) = 0.05*(-s__q_RF_HFE_joint__*s__q_RF_KFE_joint__ + c__q_RF_HFE_joint__*c__q_RF_KFE_joint__)*s__q_RF_HAA_joint__ - 0.37*(s__q_RF_HFE_joint__*c__q_RF_KFE_joint__ + s__q_RF_KFE_joint__*c__q_RF_HFE_joint__)*c__q_RF_HAA_joint__ - 0.38*c__q_RF_HAA_joint__*c__q_RF_KFE_joint__;
	(*this)(1, 5) = 0.05*(-s__q_RF_HFE_joint__*s__q_RF_KFE_joint__ + c__q_RF_HFE_joint__*c__q_RF_KFE_joint__)*c__q_RF_HAA_joint__ + 0.37*(s__q_RF_HFE_joint__*c__q_RF_KFE_joint__ + s__q_RF_KFE_joint__*c__q_RF_HFE_joint__)*s__q_RF_HAA_joint__ + 0.38*s__q_RF_HAA_joint__*c__q_RF_KFE_joint__ - 0.1185*s__q_RF_HFE_joint__*s__q_RF_KFE_joint__ + 0.1185*c__q_RF_HFE_joint__*c__q_RF_KFE_joint__;
	(*this)(2, 1) = -c__q_RF_HAA_joint__;
	(*this)(2, 2) = s__q_RF_HAA_joint__;
	(*this)(2, 3) = -0.1185*s__q_RF_HAA_joint__ - 0.38*c__q_RF_HFE_joint__;
	(*this)(2, 4) = -(0.38*s__q_RF_HFE_joint__ + 0.37)*s__q_RF_HAA_joint__;
	(*this)(2, 5) = -(0.38*s__q_RF_HFE_joint__ + 0.37)*c__q_RF_HAA_joint__;
	(*this)(3, 3) = s__q_RF_HFE_joint__*c__q_RF_KFE_joint__ + s__q_RF_KFE_joint__*c__q_RF_HFE_joint__;
	(*this)(3, 4) = -(-s__q_RF_HFE_joint__*s__q_RF_KFE_joint__ + c__q_RF_HFE_joint__*c__q_RF_KFE_joint__)*s__q_RF_HAA_joint__;
	(*this)(3, 5) = -(-s__q_RF_HFE_joint__*s__q_RF_KFE_joint__ + c__q_RF_HFE_joint__*c__q_RF_KFE_joint__)*c__q_RF_HAA_joint__;
	(*this)(4, 3) = -s__q_RF_HFE_joint__*s__q_RF_KFE_joint__ + c__q_RF_HFE_joint__*c__q_RF_KFE_joint__;
	(*this)(4, 4) = (s__q_RF_HFE_joint__*c__q_RF_KFE_joint__ + s__q_RF_KFE_joint__*c__q_RF_HFE_joint__)*s__q_RF_HAA_joint__;
	(*this)(4, 5) = (s__q_RF_HFE_joint__*c__q_RF_KFE_joint__ + s__q_RF_KFE_joint__*c__q_RF_HFE_joint__)*c__q_RF_HAA_joint__;
	(*this)(5, 4) = -c__q_RF_HAA_joint__;
	(*this)(5, 5) = s__q_RF_HAA_joint__;
	return *this;
}

template <typename TRAIT>
iit::pegasus2::tpl::ForceTransforms<TRAIT>::Type_fr_torso_X_fr_RH_hip::Type_fr_torso_X_fr_RH_hip(){ 
	(*this)(0, 0) = 0;
	(*this)(0, 1) = 0;
	(*this)(0, 2) = -1;
	(*this)(0, 5) = 0;
	(*this)(1, 2) = 0;
	(*this)(1, 5) = 0;
	(*this)(2, 2) = 0;
	(*this)(2, 5) = -0.118500000000000;
	(*this)(3, 0) = 0;
	(*this)(3, 1) = 0;
	(*this)(3, 2) = 0;
	(*this)(3, 3) = 0;
	(*this)(3, 4) = 0;
	(*this)(3, 5) = -1;
	(*this)(4, 0) = 0;
	(*this)(4, 1) = 0;
	(*this)(4, 2) = 0;
	(*this)(4, 5) = 0;
	(*this)(5, 0) = 0;
	(*this)(5, 1) = 0;
	(*this)(5, 2) = 0;
	(*this)(5, 5) = 0;
}

template <typename TRAIT>
const typename iit::pegasus2::tpl::ForceTransforms<TRAIT>::Type_fr_torso_X_fr_RH_hip& 
iit::pegasus2::tpl::ForceTransforms<TRAIT>::Type_fr_torso_X_fr_RH_hip::update(const JState& q){ 
	Scalar c__q_RH_HAA_joint__;
	Scalar s__q_RH_HAA_joint__;

	c__q_RH_HAA_joint__ = TRAIT::cos( q[RH_HAA_JOINT]);
	s__q_RH_HAA_joint__ = TRAIT::sin( q[RH_HAA_JOINT]);

	(*this)(0, 3) = -0.1185*c__q_RH_HAA_joint__;
	(*this)(0, 4) = 0.1185*s__q_RH_HAA_joint__;
	(*this)(1, 0) = s__q_RH_HAA_joint__;
	(*this)(1, 1) = c__q_RH_HAA_joint__;
	(*this)(1, 3) = 0.37*c__q_RH_HAA_joint__;
	(*this)(1, 4) = -0.37*s__q_RH_HAA_joint__;
	(*this)(2, 0) = c__q_RH_HAA_joint__;
	(*this)(2, 1) = -s__q_RH_HAA_joint__;
	(*this)(2, 3) = -0.37*s__q_RH_HAA_joint__;
	(*this)(2, 4) = -0.37*c__q_RH_HAA_joint__;
	(*this)(4, 3) = s__q_RH_HAA_joint__;
	(*this)(4, 4) = c__q_RH_HAA_joint__;
	(*this)(5, 3) = c__q_RH_HAA_joint__;
	(*this)(5, 4) = -s__q_RH_HAA_joint__;
	return *this;
}

template <typename TRAIT>
iit::pegasus2::tpl::ForceTransforms<TRAIT>::Type_fr_RH_hip_X_fr_torso::Type_fr_RH_hip_X_fr_torso(){ 
	(*this)(0, 0) = 0;
	(*this)(1, 0) = 0;
	(*this)(2, 0) = -1;
	(*this)(2, 1) = 0;
	(*this)(2, 2) = 0;
	(*this)(2, 3) = 0;
	(*this)(2, 4) = 0;
	(*this)(2, 5) = -0.118500000000000;
	(*this)(3, 0) = 0;
	(*this)(3, 1) = 0;
	(*this)(3, 2) = 0;
	(*this)(3, 3) = 0;
	(*this)(4, 0) = 0;
	(*this)(4, 1) = 0;
	(*this)(4, 2) = 0;
	(*this)(4, 3) = 0;
	(*this)(5, 0) = 0;
	(*this)(5, 1) = 0;
	(*this)(5, 2) = 0;
	(*this)(5, 3) = -1;
	(*this)(5, 4) = 0;
	(*this)(5, 5) = 0;
}

template <typename TRAIT>
const typename iit::pegasus2::tpl::ForceTransforms<TRAIT>::Type_fr_RH_hip_X_fr_torso& 
iit::pegasus2::tpl::ForceTransforms<TRAIT>::Type_fr_RH_hip_X_fr_torso::update(const JState& q){ 
	Scalar c__q_RH_HAA_joint__;
	Scalar s__q_RH_HAA_joint__;

	c__q_RH_HAA_joint__ = TRAIT::cos( q[RH_HAA_JOINT]);
	s__q_RH_HAA_joint__ = TRAIT::sin( q[RH_HAA_JOINT]);

	(*this)(0, 1) = s__q_RH_HAA_joint__;
	(*this)(0, 2) = c__q_RH_HAA_joint__;
	(*this)(0, 3) = -0.1185*c__q_RH_HAA_joint__;
	(*this)(0, 4) = 0.37*c__q_RH_HAA_joint__;
	(*this)(0, 5) = -0.37*s__q_RH_HAA_joint__;
	(*this)(1, 1) = c__q_RH_HAA_joint__;
	(*this)(1, 2) = -s__q_RH_HAA_joint__;
	(*this)(1, 3) = 0.1185*s__q_RH_HAA_joint__;
	(*this)(1, 4) = -0.37*s__q_RH_HAA_joint__;
	(*this)(1, 5) = -0.37*c__q_RH_HAA_joint__;
	(*this)(3, 4) = s__q_RH_HAA_joint__;
	(*this)(3, 5) = c__q_RH_HAA_joint__;
	(*this)(4, 4) = c__q_RH_HAA_joint__;
	(*this)(4, 5) = -s__q_RH_HAA_joint__;
	return *this;
}

template <typename TRAIT>
iit::pegasus2::tpl::ForceTransforms<TRAIT>::Type_fr_torso_X_fr_RH_upperleg::Type_fr_torso_X_fr_RH_upperleg(){ 
	(*this)(0, 2) = 0;
	(*this)(3, 0) = 0;
	(*this)(3, 1) = 0;
	(*this)(3, 2) = 0;
	(*this)(3, 5) = 0;
	(*this)(4, 0) = 0;
	(*this)(4, 1) = 0;
	(*this)(4, 2) = 0;
	(*this)(5, 0) = 0;
	(*this)(5, 1) = 0;
	(*this)(5, 2) = 0;
}

template <typename TRAIT>
const typename iit::pegasus2::tpl::ForceTransforms<TRAIT>::Type_fr_torso_X_fr_RH_upperleg& 
iit::pegasus2::tpl::ForceTransforms<TRAIT>::Type_fr_torso_X_fr_RH_upperleg::update(const JState& q){ 
	Scalar c__q_RH_HFE_joint__;
	Scalar c__q_RH_HAA_joint__;
	Scalar s__q_RH_HAA_joint__;
	Scalar s__q_RH_HFE_joint__;

	c__q_RH_HFE_joint__ = TRAIT::cos( q[RH_HFE_JOINT]);
	c__q_RH_HAA_joint__ = TRAIT::cos( q[RH_HAA_JOINT]);
	s__q_RH_HAA_joint__ = TRAIT::sin( q[RH_HAA_JOINT]);
	s__q_RH_HFE_joint__ = TRAIT::sin( q[RH_HFE_JOINT]);

	(*this)(0, 0) = s__q_RH_HFE_joint__;
	(*this)(0, 1) = c__q_RH_HFE_joint__;
	(*this)(0, 3) = (0.1185*c__q_RH_HAA_joint__ + 0.05)*c__q_RH_HFE_joint__;
	(*this)(0, 4) = -(0.1185*c__q_RH_HAA_joint__ + 0.05)*s__q_RH_HFE_joint__;
	(*this)(0, 5) = -0.1185*s__q_RH_HAA_joint__;
	(*this)(1, 0) = -s__q_RH_HAA_joint__*c__q_RH_HFE_joint__;
	(*this)(1, 1) = s__q_RH_HAA_joint__*s__q_RH_HFE_joint__;
	(*this)(1, 2) = -c__q_RH_HAA_joint__;
	(*this)(1, 3) = 0.05*s__q_RH_HAA_joint__*s__q_RH_HFE_joint__ - 0.37*c__q_RH_HAA_joint__*c__q_RH_HFE_joint__;
	(*this)(1, 4) = 0.05*s__q_RH_HAA_joint__*c__q_RH_HFE_joint__ + 0.37*s__q_RH_HFE_joint__*c__q_RH_HAA_joint__;
	(*this)(1, 5) = 0.37*s__q_RH_HAA_joint__;
	(*this)(2, 0) = -c__q_RH_HAA_joint__*c__q_RH_HFE_joint__;
	(*this)(2, 1) = s__q_RH_HFE_joint__*c__q_RH_HAA_joint__;
	(*this)(2, 2) = s__q_RH_HAA_joint__;
	(*this)(2, 3) = 0.37*s__q_RH_HAA_joint__*c__q_RH_HFE_joint__ + 0.05*s__q_RH_HFE_joint__*c__q_RH_HAA_joint__ + 0.1185*s__q_RH_HFE_joint__;
	(*this)(2, 4) = -0.37*s__q_RH_HAA_joint__*s__q_RH_HFE_joint__ + 0.05*c__q_RH_HAA_joint__*c__q_RH_HFE_joint__ + 0.1185*c__q_RH_HFE_joint__;
	(*this)(2, 5) = 0.37*c__q_RH_HAA_joint__;
	(*this)(3, 3) = s__q_RH_HFE_joint__;
	(*this)(3, 4) = c__q_RH_HFE_joint__;
	(*this)(4, 3) = -s__q_RH_HAA_joint__*c__q_RH_HFE_joint__;
	(*this)(4, 4) = s__q_RH_HAA_joint__*s__q_RH_HFE_joint__;
	(*this)(4, 5) = -c__q_RH_HAA_joint__;
	(*this)(5, 3) = -c__q_RH_HAA_joint__*c__q_RH_HFE_joint__;
	(*this)(5, 4) = s__q_RH_HFE_joint__*c__q_RH_HAA_joint__;
	(*this)(5, 5) = s__q_RH_HAA_joint__;
	return *this;
}

template <typename TRAIT>
iit::pegasus2::tpl::ForceTransforms<TRAIT>::Type_fr_RH_upperleg_X_fr_torso::Type_fr_RH_upperleg_X_fr_torso(){ 
	(*this)(2, 0) = 0;
	(*this)(3, 0) = 0;
	(*this)(3, 1) = 0;
	(*this)(3, 2) = 0;
	(*this)(4, 0) = 0;
	(*this)(4, 1) = 0;
	(*this)(4, 2) = 0;
	(*this)(5, 0) = 0;
	(*this)(5, 1) = 0;
	(*this)(5, 2) = 0;
	(*this)(5, 3) = 0;
}

template <typename TRAIT>
const typename iit::pegasus2::tpl::ForceTransforms<TRAIT>::Type_fr_RH_upperleg_X_fr_torso& 
iit::pegasus2::tpl::ForceTransforms<TRAIT>::Type_fr_RH_upperleg_X_fr_torso::update(const JState& q){ 
	Scalar c__q_RH_HFE_joint__;
	Scalar c__q_RH_HAA_joint__;
	Scalar s__q_RH_HAA_joint__;
	Scalar s__q_RH_HFE_joint__;

	c__q_RH_HFE_joint__ = TRAIT::cos( q[RH_HFE_JOINT]);
	c__q_RH_HAA_joint__ = TRAIT::cos( q[RH_HAA_JOINT]);
	s__q_RH_HAA_joint__ = TRAIT::sin( q[RH_HAA_JOINT]);
	s__q_RH_HFE_joint__ = TRAIT::sin( q[RH_HFE_JOINT]);

	(*this)(0, 0) = s__q_RH_HFE_joint__;
	(*this)(0, 1) = -s__q_RH_HAA_joint__*c__q_RH_HFE_joint__;
	(*this)(0, 2) = -c__q_RH_HAA_joint__*c__q_RH_HFE_joint__;
	(*this)(0, 3) = (0.1185*c__q_RH_HAA_joint__ + 0.05)*c__q_RH_HFE_joint__;
	(*this)(0, 4) = 0.05*s__q_RH_HAA_joint__*s__q_RH_HFE_joint__ - 0.37*c__q_RH_HAA_joint__*c__q_RH_HFE_joint__;
	(*this)(0, 5) = 0.37*s__q_RH_HAA_joint__*c__q_RH_HFE_joint__ + 0.05*s__q_RH_HFE_joint__*c__q_RH_HAA_joint__ + 0.1185*s__q_RH_HFE_joint__;
	(*this)(1, 0) = c__q_RH_HFE_joint__;
	(*this)(1, 1) = s__q_RH_HAA_joint__*s__q_RH_HFE_joint__;
	(*this)(1, 2) = s__q_RH_HFE_joint__*c__q_RH_HAA_joint__;
	(*this)(1, 3) = -(0.1185*c__q_RH_HAA_joint__ + 0.05)*s__q_RH_HFE_joint__;
	(*this)(1, 4) = 0.05*s__q_RH_HAA_joint__*c__q_RH_HFE_joint__ + 0.37*s__q_RH_HFE_joint__*c__q_RH_HAA_joint__;
	(*this)(1, 5) = -0.37*s__q_RH_HAA_joint__*s__q_RH_HFE_joint__ + 0.05*c__q_RH_HAA_joint__*c__q_RH_HFE_joint__ + 0.1185*c__q_RH_HFE_joint__;
	(*this)(2, 1) = -c__q_RH_HAA_joint__;
	(*this)(2, 2) = s__q_RH_HAA_joint__;
	(*this)(2, 3) = -0.1185*s__q_RH_HAA_joint__;
	(*this)(2, 4) = 0.37*s__q_RH_HAA_joint__;
	(*this)(2, 5) = 0.37*c__q_RH_HAA_joint__;
	(*this)(3, 3) = s__q_RH_HFE_joint__;
	(*this)(3, 4) = -s__q_RH_HAA_joint__*c__q_RH_HFE_joint__;
	(*this)(3, 5) = -c__q_RH_HAA_joint__*c__q_RH_HFE_joint__;
	(*this)(4, 3) = c__q_RH_HFE_joint__;
	(*this)(4, 4) = s__q_RH_HAA_joint__*s__q_RH_HFE_joint__;
	(*this)(4, 5) = s__q_RH_HFE_joint__*c__q_RH_HAA_joint__;
	(*this)(5, 4) = -c__q_RH_HAA_joint__;
	(*this)(5, 5) = s__q_RH_HAA_joint__;
	return *this;
}

template <typename TRAIT>
iit::pegasus2::tpl::ForceTransforms<TRAIT>::Type_fr_torso_X_fr_RH_lowerleg::Type_fr_torso_X_fr_RH_lowerleg(){ 
	(*this)(0, 2) = 0;
	(*this)(3, 0) = 0;
	(*this)(3, 1) = 0;
	(*this)(3, 2) = 0;
	(*this)(3, 5) = 0;
	(*this)(4, 0) = 0;
	(*this)(4, 1) = 0;
	(*this)(4, 2) = 0;
	(*this)(5, 0) = 0;
	(*this)(5, 1) = 0;
	(*this)(5, 2) = 0;
}

template <typename TRAIT>
const typename iit::pegasus2::tpl::ForceTransforms<TRAIT>::Type_fr_torso_X_fr_RH_lowerleg& 
iit::pegasus2::tpl::ForceTransforms<TRAIT>::Type_fr_torso_X_fr_RH_lowerleg::update(const JState& q){ 
	Scalar c__q_RH_HFE_joint__;
	Scalar c__q_RH_HAA_joint__;
	Scalar c__q_RH_KFE_joint__;
	Scalar s__q_RH_KFE_joint__;
	Scalar s__q_RH_HAA_joint__;
	Scalar s__q_RH_HFE_joint__;

	c__q_RH_HFE_joint__ = TRAIT::cos( q[RH_HFE_JOINT]);
	c__q_RH_HAA_joint__ = TRAIT::cos( q[RH_HAA_JOINT]);
	c__q_RH_KFE_joint__ = TRAIT::cos( q[RH_KFE_JOINT]);
	s__q_RH_KFE_joint__ = TRAIT::sin( q[RH_KFE_JOINT]);
	s__q_RH_HAA_joint__ = TRAIT::sin( q[RH_HAA_JOINT]);
	s__q_RH_HFE_joint__ = TRAIT::sin( q[RH_HFE_JOINT]);

	(*this)(0, 0) = s__q_RH_HFE_joint__*c__q_RH_KFE_joint__ + s__q_RH_KFE_joint__*c__q_RH_HFE_joint__;
	(*this)(0, 1) = -s__q_RH_HFE_joint__*s__q_RH_KFE_joint__ + c__q_RH_HFE_joint__*c__q_RH_KFE_joint__;
	(*this)(0, 3) = (-s__q_RH_HFE_joint__*s__q_RH_KFE_joint__ + c__q_RH_HFE_joint__*c__q_RH_KFE_joint__)*(0.1185*c__q_RH_HAA_joint__ + 0.05);
	(*this)(0, 4) = -(s__q_RH_HFE_joint__*c__q_RH_KFE_joint__ + s__q_RH_KFE_joint__*c__q_RH_HFE_joint__)*(0.1185*c__q_RH_HAA_joint__ + 0.05);
	(*this)(0, 5) = -0.1185*s__q_RH_HAA_joint__ - 0.38*c__q_RH_HFE_joint__;
	(*this)(1, 0) = -(-s__q_RH_HFE_joint__*s__q_RH_KFE_joint__ + c__q_RH_HFE_joint__*c__q_RH_KFE_joint__)*s__q_RH_HAA_joint__;
	(*this)(1, 1) = (s__q_RH_HFE_joint__*c__q_RH_KFE_joint__ + s__q_RH_KFE_joint__*c__q_RH_HFE_joint__)*s__q_RH_HAA_joint__;
	(*this)(1, 2) = -c__q_RH_HAA_joint__;
	(*this)(1, 3) = -0.37*(-s__q_RH_HFE_joint__*s__q_RH_KFE_joint__ + c__q_RH_HFE_joint__*c__q_RH_KFE_joint__)*c__q_RH_HAA_joint__ + 0.05*(s__q_RH_HFE_joint__*c__q_RH_KFE_joint__ + s__q_RH_KFE_joint__*c__q_RH_HFE_joint__)*s__q_RH_HAA_joint__ - 0.38*s__q_RH_KFE_joint__*c__q_RH_HAA_joint__;
	(*this)(1, 4) = 0.16*(s__q_RH_HAA_joint__*s__q_RH_KFE_joint__ + c__q_RH_HAA_joint__*c__q_RH_KFE_joint__)*s__q_RH_HFE_joint__ + 0.16*(-s__q_RH_HAA_joint__*c__q_RH_KFE_joint__ + s__q_RH_KFE_joint__*c__q_RH_HAA_joint__)*c__q_RH_HFE_joint__ + 0.21*(-s__q_RH_HFE_joint__*s__q_RH_KFE_joint__ + c__q_RH_HFE_joint__*c__q_RH_KFE_joint__)*s__q_RH_HAA_joint__ + 0.21*(s__q_RH_HFE_joint__*c__q_RH_KFE_joint__ + s__q_RH_KFE_joint__*c__q_RH_HFE_joint__)*c__q_RH_HAA_joint__ - 0.38*c__q_RH_HAA_joint__*c__q_RH_KFE_joint__;
	(*this)(1, 5) = (0.37 - 0.38*s__q_RH_HFE_joint__)*s__q_RH_HAA_joint__;
	(*this)(2, 0) = -(-s__q_RH_HFE_joint__*s__q_RH_KFE_joint__ + c__q_RH_HFE_joint__*c__q_RH_KFE_joint__)*c__q_RH_HAA_joint__;
	(*this)(2, 1) = (s__q_RH_HFE_joint__*c__q_RH_KFE_joint__ + s__q_RH_KFE_joint__*c__q_RH_HFE_joint__)*c__q_RH_HAA_joint__;
	(*this)(2, 2) = s__q_RH_HAA_joint__;
	(*this)(2, 3) = 0.37*(-s__q_RH_HFE_joint__*s__q_RH_KFE_joint__ + c__q_RH_HFE_joint__*c__q_RH_KFE_joint__)*s__q_RH_HAA_joint__ + 0.05*(s__q_RH_HFE_joint__*c__q_RH_KFE_joint__ + s__q_RH_KFE_joint__*c__q_RH_HFE_joint__)*c__q_RH_HAA_joint__ + 0.38*s__q_RH_HAA_joint__*s__q_RH_KFE_joint__ + 0.1185*s__q_RH_HFE_joint__*c__q_RH_KFE_joint__ + 0.1185*s__q_RH_KFE_joint__*c__q_RH_HFE_joint__;
	(*this)(2, 4) = 0.05*(-s__q_RH_HFE_joint__*s__q_RH_KFE_joint__ + c__q_RH_HFE_joint__*c__q_RH_KFE_joint__)*c__q_RH_HAA_joint__ - 0.37*(s__q_RH_HFE_joint__*c__q_RH_KFE_joint__ + s__q_RH_KFE_joint__*c__q_RH_HFE_joint__)*s__q_RH_HAA_joint__ + 0.38*s__q_RH_HAA_joint__*c__q_RH_KFE_joint__ - 0.1185*s__q_RH_HFE_joint__*s__q_RH_KFE_joint__ + 0.1185*c__q_RH_HFE_joint__*c__q_RH_KFE_joint__;
	(*this)(2, 5) = (0.37 - 0.38*s__q_RH_HFE_joint__)*c__q_RH_HAA_joint__;
	(*this)(3, 3) = s__q_RH_HFE_joint__*c__q_RH_KFE_joint__ + s__q_RH_KFE_joint__*c__q_RH_HFE_joint__;
	(*this)(3, 4) = -s__q_RH_HFE_joint__*s__q_RH_KFE_joint__ + c__q_RH_HFE_joint__*c__q_RH_KFE_joint__;
	(*this)(4, 3) = -(-s__q_RH_HFE_joint__*s__q_RH_KFE_joint__ + c__q_RH_HFE_joint__*c__q_RH_KFE_joint__)*s__q_RH_HAA_joint__;
	(*this)(4, 4) = (s__q_RH_HFE_joint__*c__q_RH_KFE_joint__ + s__q_RH_KFE_joint__*c__q_RH_HFE_joint__)*s__q_RH_HAA_joint__;
	(*this)(4, 5) = -c__q_RH_HAA_joint__;
	(*this)(5, 3) = -(-s__q_RH_HFE_joint__*s__q_RH_KFE_joint__ + c__q_RH_HFE_joint__*c__q_RH_KFE_joint__)*c__q_RH_HAA_joint__;
	(*this)(5, 4) = (s__q_RH_HFE_joint__*c__q_RH_KFE_joint__ + s__q_RH_KFE_joint__*c__q_RH_HFE_joint__)*c__q_RH_HAA_joint__;
	(*this)(5, 5) = s__q_RH_HAA_joint__;
	return *this;
}

template <typename TRAIT>
iit::pegasus2::tpl::ForceTransforms<TRAIT>::Type_fr_RH_lowerleg_X_fr_torso::Type_fr_RH_lowerleg_X_fr_torso(){ 
	(*this)(2, 0) = 0;
	(*this)(3, 0) = 0;
	(*this)(3, 1) = 0;
	(*this)(3, 2) = 0;
	(*this)(4, 0) = 0;
	(*this)(4, 1) = 0;
	(*this)(4, 2) = 0;
	(*this)(5, 0) = 0;
	(*this)(5, 1) = 0;
	(*this)(5, 2) = 0;
	(*this)(5, 3) = 0;
}

template <typename TRAIT>
const typename iit::pegasus2::tpl::ForceTransforms<TRAIT>::Type_fr_RH_lowerleg_X_fr_torso& 
iit::pegasus2::tpl::ForceTransforms<TRAIT>::Type_fr_RH_lowerleg_X_fr_torso::update(const JState& q){ 
	Scalar c__q_RH_HFE_joint__;
	Scalar c__q_RH_HAA_joint__;
	Scalar c__q_RH_KFE_joint__;
	Scalar s__q_RH_KFE_joint__;
	Scalar s__q_RH_HAA_joint__;
	Scalar s__q_RH_HFE_joint__;

	c__q_RH_HFE_joint__ = TRAIT::cos( q[RH_HFE_JOINT]);
	c__q_RH_HAA_joint__ = TRAIT::cos( q[RH_HAA_JOINT]);
	c__q_RH_KFE_joint__ = TRAIT::cos( q[RH_KFE_JOINT]);
	s__q_RH_KFE_joint__ = TRAIT::sin( q[RH_KFE_JOINT]);
	s__q_RH_HAA_joint__ = TRAIT::sin( q[RH_HAA_JOINT]);
	s__q_RH_HFE_joint__ = TRAIT::sin( q[RH_HFE_JOINT]);

	(*this)(0, 0) = s__q_RH_HFE_joint__*c__q_RH_KFE_joint__ + s__q_RH_KFE_joint__*c__q_RH_HFE_joint__;
	(*this)(0, 1) = -(-s__q_RH_HFE_joint__*s__q_RH_KFE_joint__ + c__q_RH_HFE_joint__*c__q_RH_KFE_joint__)*s__q_RH_HAA_joint__;
	(*this)(0, 2) = -(-s__q_RH_HFE_joint__*s__q_RH_KFE_joint__ + c__q_RH_HFE_joint__*c__q_RH_KFE_joint__)*c__q_RH_HAA_joint__;
	(*this)(0, 3) = (-s__q_RH_HFE_joint__*s__q_RH_KFE_joint__ + c__q_RH_HFE_joint__*c__q_RH_KFE_joint__)*(0.1185*c__q_RH_HAA_joint__ + 0.05);
	(*this)(0, 4) = -0.37*(-s__q_RH_HFE_joint__*s__q_RH_KFE_joint__ + c__q_RH_HFE_joint__*c__q_RH_KFE_joint__)*c__q_RH_HAA_joint__ + 0.05*(s__q_RH_HFE_joint__*c__q_RH_KFE_joint__ + s__q_RH_KFE_joint__*c__q_RH_HFE_joint__)*s__q_RH_HAA_joint__ - 0.38*s__q_RH_KFE_joint__*c__q_RH_HAA_joint__;
	(*this)(0, 5) = 0.37*(-s__q_RH_HFE_joint__*s__q_RH_KFE_joint__ + c__q_RH_HFE_joint__*c__q_RH_KFE_joint__)*s__q_RH_HAA_joint__ + 0.05*(s__q_RH_HFE_joint__*c__q_RH_KFE_joint__ + s__q_RH_KFE_joint__*c__q_RH_HFE_joint__)*c__q_RH_HAA_joint__ + 0.38*s__q_RH_HAA_joint__*s__q_RH_KFE_joint__ + 0.1185*s__q_RH_HFE_joint__*c__q_RH_KFE_joint__ + 0.1185*s__q_RH_KFE_joint__*c__q_RH_HFE_joint__;
	(*this)(1, 0) = -s__q_RH_HFE_joint__*s__q_RH_KFE_joint__ + c__q_RH_HFE_joint__*c__q_RH_KFE_joint__;
	(*this)(1, 1) = (s__q_RH_HFE_joint__*c__q_RH_KFE_joint__ + s__q_RH_KFE_joint__*c__q_RH_HFE_joint__)*s__q_RH_HAA_joint__;
	(*this)(1, 2) = (s__q_RH_HFE_joint__*c__q_RH_KFE_joint__ + s__q_RH_KFE_joint__*c__q_RH_HFE_joint__)*c__q_RH_HAA_joint__;
	(*this)(1, 3) = -(s__q_RH_HFE_joint__*c__q_RH_KFE_joint__ + s__q_RH_KFE_joint__*c__q_RH_HFE_joint__)*(0.1185*c__q_RH_HAA_joint__ + 0.05);
	(*this)(1, 4) = 0.05*(-s__q_RH_HFE_joint__*s__q_RH_KFE_joint__ + c__q_RH_HFE_joint__*c__q_RH_KFE_joint__)*s__q_RH_HAA_joint__ + 0.37*(s__q_RH_HFE_joint__*c__q_RH_KFE_joint__ + s__q_RH_KFE_joint__*c__q_RH_HFE_joint__)*c__q_RH_HAA_joint__ - 0.38*c__q_RH_HAA_joint__*c__q_RH_KFE_joint__;
	(*this)(1, 5) = 0.05*(-s__q_RH_HFE_joint__*s__q_RH_KFE_joint__ + c__q_RH_HFE_joint__*c__q_RH_KFE_joint__)*c__q_RH_HAA_joint__ - 0.37*(s__q_RH_HFE_joint__*c__q_RH_KFE_joint__ + s__q_RH_KFE_joint__*c__q_RH_HFE_joint__)*s__q_RH_HAA_joint__ + 0.38*s__q_RH_HAA_joint__*c__q_RH_KFE_joint__ - 0.1185*s__q_RH_HFE_joint__*s__q_RH_KFE_joint__ + 0.1185*c__q_RH_HFE_joint__*c__q_RH_KFE_joint__;
	(*this)(2, 1) = -c__q_RH_HAA_joint__;
	(*this)(2, 2) = s__q_RH_HAA_joint__;
	(*this)(2, 3) = -0.1185*s__q_RH_HAA_joint__ - 0.38*c__q_RH_HFE_joint__;
	(*this)(2, 4) = (0.37 - 0.38*s__q_RH_HFE_joint__)*s__q_RH_HAA_joint__;
	(*this)(2, 5) = (0.37 - 0.38*s__q_RH_HFE_joint__)*c__q_RH_HAA_joint__;
	(*this)(3, 3) = s__q_RH_HFE_joint__*c__q_RH_KFE_joint__ + s__q_RH_KFE_joint__*c__q_RH_HFE_joint__;
	(*this)(3, 4) = -(-s__q_RH_HFE_joint__*s__q_RH_KFE_joint__ + c__q_RH_HFE_joint__*c__q_RH_KFE_joint__)*s__q_RH_HAA_joint__;
	(*this)(3, 5) = -(-s__q_RH_HFE_joint__*s__q_RH_KFE_joint__ + c__q_RH_HFE_joint__*c__q_RH_KFE_joint__)*c__q_RH_HAA_joint__;
	(*this)(4, 3) = -s__q_RH_HFE_joint__*s__q_RH_KFE_joint__ + c__q_RH_HFE_joint__*c__q_RH_KFE_joint__;
	(*this)(4, 4) = (s__q_RH_HFE_joint__*c__q_RH_KFE_joint__ + s__q_RH_KFE_joint__*c__q_RH_HFE_joint__)*s__q_RH_HAA_joint__;
	(*this)(4, 5) = (s__q_RH_HFE_joint__*c__q_RH_KFE_joint__ + s__q_RH_KFE_joint__*c__q_RH_HFE_joint__)*c__q_RH_HAA_joint__;
	(*this)(5, 4) = -c__q_RH_HAA_joint__;
	(*this)(5, 5) = s__q_RH_HAA_joint__;
	return *this;
}

template <typename TRAIT>
iit::pegasus2::tpl::ForceTransforms<TRAIT>::Type_fr_torso_X_fr_LH_hip::Type_fr_torso_X_fr_LH_hip(){ 
	(*this)(0, 0) = 0;
	(*this)(0, 1) = 0;
	(*this)(0, 2) = -1;
	(*this)(0, 5) = 0;
	(*this)(1, 2) = 0;
	(*this)(1, 5) = 0;
	(*this)(2, 2) = 0;
	(*this)(2, 5) = 0.118500000000000;
	(*this)(3, 0) = 0;
	(*this)(3, 1) = 0;
	(*this)(3, 2) = 0;
	(*this)(3, 3) = 0;
	(*this)(3, 4) = 0;
	(*this)(3, 5) = -1;
	(*this)(4, 0) = 0;
	(*this)(4, 1) = 0;
	(*this)(4, 2) = 0;
	(*this)(4, 5) = 0;
	(*this)(5, 0) = 0;
	(*this)(5, 1) = 0;
	(*this)(5, 2) = 0;
	(*this)(5, 5) = 0;
}

template <typename TRAIT>
const typename iit::pegasus2::tpl::ForceTransforms<TRAIT>::Type_fr_torso_X_fr_LH_hip& 
iit::pegasus2::tpl::ForceTransforms<TRAIT>::Type_fr_torso_X_fr_LH_hip::update(const JState& q){ 
	Scalar c__q_LH_HAA_joint__;
	Scalar s__q_LH_HAA_joint__;

	c__q_LH_HAA_joint__ = TRAIT::cos( q[LH_HAA_JOINT]);
	s__q_LH_HAA_joint__ = TRAIT::sin( q[LH_HAA_JOINT]);

	(*this)(0, 3) = 0.1185*c__q_LH_HAA_joint__;
	(*this)(0, 4) = -0.1185*s__q_LH_HAA_joint__;
	(*this)(1, 0) = s__q_LH_HAA_joint__;
	(*this)(1, 1) = c__q_LH_HAA_joint__;
	(*this)(1, 3) = 0.37*c__q_LH_HAA_joint__;
	(*this)(1, 4) = -0.37*s__q_LH_HAA_joint__;
	(*this)(2, 0) = c__q_LH_HAA_joint__;
	(*this)(2, 1) = -s__q_LH_HAA_joint__;
	(*this)(2, 3) = -0.37*s__q_LH_HAA_joint__;
	(*this)(2, 4) = -0.37*c__q_LH_HAA_joint__;
	(*this)(4, 3) = s__q_LH_HAA_joint__;
	(*this)(4, 4) = c__q_LH_HAA_joint__;
	(*this)(5, 3) = c__q_LH_HAA_joint__;
	(*this)(5, 4) = -s__q_LH_HAA_joint__;
	return *this;
}

template <typename TRAIT>
iit::pegasus2::tpl::ForceTransforms<TRAIT>::Type_fr_LH_hip_X_fr_torso::Type_fr_LH_hip_X_fr_torso(){ 
	(*this)(0, 0) = 0;
	(*this)(1, 0) = 0;
	(*this)(2, 0) = -1;
	(*this)(2, 1) = 0;
	(*this)(2, 2) = 0;
	(*this)(2, 3) = 0;
	(*this)(2, 4) = 0;
	(*this)(2, 5) = 0.118500000000000;
	(*this)(3, 0) = 0;
	(*this)(3, 1) = 0;
	(*this)(3, 2) = 0;
	(*this)(3, 3) = 0;
	(*this)(4, 0) = 0;
	(*this)(4, 1) = 0;
	(*this)(4, 2) = 0;
	(*this)(4, 3) = 0;
	(*this)(5, 0) = 0;
	(*this)(5, 1) = 0;
	(*this)(5, 2) = 0;
	(*this)(5, 3) = -1;
	(*this)(5, 4) = 0;
	(*this)(5, 5) = 0;
}

template <typename TRAIT>
const typename iit::pegasus2::tpl::ForceTransforms<TRAIT>::Type_fr_LH_hip_X_fr_torso& 
iit::pegasus2::tpl::ForceTransforms<TRAIT>::Type_fr_LH_hip_X_fr_torso::update(const JState& q){ 
	Scalar c__q_LH_HAA_joint__;
	Scalar s__q_LH_HAA_joint__;

	c__q_LH_HAA_joint__ = TRAIT::cos( q[LH_HAA_JOINT]);
	s__q_LH_HAA_joint__ = TRAIT::sin( q[LH_HAA_JOINT]);

	(*this)(0, 1) = s__q_LH_HAA_joint__;
	(*this)(0, 2) = c__q_LH_HAA_joint__;
	(*this)(0, 3) = 0.1185*c__q_LH_HAA_joint__;
	(*this)(0, 4) = 0.37*c__q_LH_HAA_joint__;
	(*this)(0, 5) = -0.37*s__q_LH_HAA_joint__;
	(*this)(1, 1) = c__q_LH_HAA_joint__;
	(*this)(1, 2) = -s__q_LH_HAA_joint__;
	(*this)(1, 3) = -0.1185*s__q_LH_HAA_joint__;
	(*this)(1, 4) = -0.37*s__q_LH_HAA_joint__;
	(*this)(1, 5) = -0.37*c__q_LH_HAA_joint__;
	(*this)(3, 4) = s__q_LH_HAA_joint__;
	(*this)(3, 5) = c__q_LH_HAA_joint__;
	(*this)(4, 4) = c__q_LH_HAA_joint__;
	(*this)(4, 5) = -s__q_LH_HAA_joint__;
	return *this;
}

template <typename TRAIT>
iit::pegasus2::tpl::ForceTransforms<TRAIT>::Type_fr_torso_X_fr_LH_upperleg::Type_fr_torso_X_fr_LH_upperleg(){ 
	(*this)(0, 2) = 0;
	(*this)(3, 0) = 0;
	(*this)(3, 1) = 0;
	(*this)(3, 2) = 0;
	(*this)(3, 5) = 0;
	(*this)(4, 0) = 0;
	(*this)(4, 1) = 0;
	(*this)(4, 2) = 0;
	(*this)(5, 0) = 0;
	(*this)(5, 1) = 0;
	(*this)(5, 2) = 0;
}

template <typename TRAIT>
const typename iit::pegasus2::tpl::ForceTransforms<TRAIT>::Type_fr_torso_X_fr_LH_upperleg& 
iit::pegasus2::tpl::ForceTransforms<TRAIT>::Type_fr_torso_X_fr_LH_upperleg::update(const JState& q){ 
	Scalar c__q_LH_HAA_joint__;
	Scalar c__q_LH_HFE_joint__;
	Scalar s__q_LH_HFE_joint__;
	Scalar s__q_LH_HAA_joint__;

	c__q_LH_HAA_joint__ = TRAIT::cos( q[LH_HAA_JOINT]);
	c__q_LH_HFE_joint__ = TRAIT::cos( q[LH_HFE_JOINT]);
	s__q_LH_HFE_joint__ = TRAIT::sin( q[LH_HFE_JOINT]);
	s__q_LH_HAA_joint__ = TRAIT::sin( q[LH_HAA_JOINT]);

	(*this)(0, 0) = s__q_LH_HFE_joint__;
	(*this)(0, 1) = c__q_LH_HFE_joint__;
	(*this)(0, 3) = -(0.1185*c__q_LH_HAA_joint__ + 0.05)*c__q_LH_HFE_joint__;
	(*this)(0, 4) = (0.1185*c__q_LH_HAA_joint__ + 0.05)*s__q_LH_HFE_joint__;
	(*this)(0, 5) = 0.1185*s__q_LH_HAA_joint__;
	(*this)(1, 0) = -s__q_LH_HAA_joint__*c__q_LH_HFE_joint__;
	(*this)(1, 1) = s__q_LH_HAA_joint__*s__q_LH_HFE_joint__;
	(*this)(1, 2) = -c__q_LH_HAA_joint__;
	(*this)(1, 3) = -0.05*s__q_LH_HAA_joint__*s__q_LH_HFE_joint__ - 0.37*c__q_LH_HAA_joint__*c__q_LH_HFE_joint__;
	(*this)(1, 4) = -0.05*s__q_LH_HAA_joint__*c__q_LH_HFE_joint__ + 0.37*s__q_LH_HFE_joint__*c__q_LH_HAA_joint__;
	(*this)(1, 5) = 0.37*s__q_LH_HAA_joint__;
	(*this)(2, 0) = -c__q_LH_HAA_joint__*c__q_LH_HFE_joint__;
	(*this)(2, 1) = s__q_LH_HFE_joint__*c__q_LH_HAA_joint__;
	(*this)(2, 2) = s__q_LH_HAA_joint__;
	(*this)(2, 3) = 0.37*s__q_LH_HAA_joint__*c__q_LH_HFE_joint__ - 0.05*s__q_LH_HFE_joint__*c__q_LH_HAA_joint__ - 0.1185*s__q_LH_HFE_joint__;
	(*this)(2, 4) = -0.37*s__q_LH_HAA_joint__*s__q_LH_HFE_joint__ - 0.05*c__q_LH_HAA_joint__*c__q_LH_HFE_joint__ - 0.1185*c__q_LH_HFE_joint__;
	(*this)(2, 5) = 0.37*c__q_LH_HAA_joint__;
	(*this)(3, 3) = s__q_LH_HFE_joint__;
	(*this)(3, 4) = c__q_LH_HFE_joint__;
	(*this)(4, 3) = -s__q_LH_HAA_joint__*c__q_LH_HFE_joint__;
	(*this)(4, 4) = s__q_LH_HAA_joint__*s__q_LH_HFE_joint__;
	(*this)(4, 5) = -c__q_LH_HAA_joint__;
	(*this)(5, 3) = -c__q_LH_HAA_joint__*c__q_LH_HFE_joint__;
	(*this)(5, 4) = s__q_LH_HFE_joint__*c__q_LH_HAA_joint__;
	(*this)(5, 5) = s__q_LH_HAA_joint__;
	return *this;
}

template <typename TRAIT>
iit::pegasus2::tpl::ForceTransforms<TRAIT>::Type_fr_LH_upperleg_X_fr_torso::Type_fr_LH_upperleg_X_fr_torso(){ 
	(*this)(2, 0) = 0;
	(*this)(3, 0) = 0;
	(*this)(3, 1) = 0;
	(*this)(3, 2) = 0;
	(*this)(4, 0) = 0;
	(*this)(4, 1) = 0;
	(*this)(4, 2) = 0;
	(*this)(5, 0) = 0;
	(*this)(5, 1) = 0;
	(*this)(5, 2) = 0;
	(*this)(5, 3) = 0;
}

template <typename TRAIT>
const typename iit::pegasus2::tpl::ForceTransforms<TRAIT>::Type_fr_LH_upperleg_X_fr_torso& 
iit::pegasus2::tpl::ForceTransforms<TRAIT>::Type_fr_LH_upperleg_X_fr_torso::update(const JState& q){ 
	Scalar c__q_LH_HAA_joint__;
	Scalar c__q_LH_HFE_joint__;
	Scalar s__q_LH_HFE_joint__;
	Scalar s__q_LH_HAA_joint__;

	c__q_LH_HAA_joint__ = TRAIT::cos( q[LH_HAA_JOINT]);
	c__q_LH_HFE_joint__ = TRAIT::cos( q[LH_HFE_JOINT]);
	s__q_LH_HFE_joint__ = TRAIT::sin( q[LH_HFE_JOINT]);
	s__q_LH_HAA_joint__ = TRAIT::sin( q[LH_HAA_JOINT]);

	(*this)(0, 0) = s__q_LH_HFE_joint__;
	(*this)(0, 1) = -s__q_LH_HAA_joint__*c__q_LH_HFE_joint__;
	(*this)(0, 2) = -c__q_LH_HAA_joint__*c__q_LH_HFE_joint__;
	(*this)(0, 3) = -(0.1185*c__q_LH_HAA_joint__ + 0.05)*c__q_LH_HFE_joint__;
	(*this)(0, 4) = -0.05*s__q_LH_HAA_joint__*s__q_LH_HFE_joint__ - 0.37*c__q_LH_HAA_joint__*c__q_LH_HFE_joint__;
	(*this)(0, 5) = 0.37*s__q_LH_HAA_joint__*c__q_LH_HFE_joint__ - 0.05*s__q_LH_HFE_joint__*c__q_LH_HAA_joint__ - 0.1185*s__q_LH_HFE_joint__;
	(*this)(1, 0) = c__q_LH_HFE_joint__;
	(*this)(1, 1) = s__q_LH_HAA_joint__*s__q_LH_HFE_joint__;
	(*this)(1, 2) = s__q_LH_HFE_joint__*c__q_LH_HAA_joint__;
	(*this)(1, 3) = (0.1185*c__q_LH_HAA_joint__ + 0.05)*s__q_LH_HFE_joint__;
	(*this)(1, 4) = -0.05*s__q_LH_HAA_joint__*c__q_LH_HFE_joint__ + 0.37*s__q_LH_HFE_joint__*c__q_LH_HAA_joint__;
	(*this)(1, 5) = -0.37*s__q_LH_HAA_joint__*s__q_LH_HFE_joint__ - 0.05*c__q_LH_HAA_joint__*c__q_LH_HFE_joint__ - 0.1185*c__q_LH_HFE_joint__;
	(*this)(2, 1) = -c__q_LH_HAA_joint__;
	(*this)(2, 2) = s__q_LH_HAA_joint__;
	(*this)(2, 3) = 0.1185*s__q_LH_HAA_joint__;
	(*this)(2, 4) = 0.37*s__q_LH_HAA_joint__;
	(*this)(2, 5) = 0.37*c__q_LH_HAA_joint__;
	(*this)(3, 3) = s__q_LH_HFE_joint__;
	(*this)(3, 4) = -s__q_LH_HAA_joint__*c__q_LH_HFE_joint__;
	(*this)(3, 5) = -c__q_LH_HAA_joint__*c__q_LH_HFE_joint__;
	(*this)(4, 3) = c__q_LH_HFE_joint__;
	(*this)(4, 4) = s__q_LH_HAA_joint__*s__q_LH_HFE_joint__;
	(*this)(4, 5) = s__q_LH_HFE_joint__*c__q_LH_HAA_joint__;
	(*this)(5, 4) = -c__q_LH_HAA_joint__;
	(*this)(5, 5) = s__q_LH_HAA_joint__;
	return *this;
}

template <typename TRAIT>
iit::pegasus2::tpl::ForceTransforms<TRAIT>::Type_fr_torso_X_fr_LH_lowerleg::Type_fr_torso_X_fr_LH_lowerleg(){ 
	(*this)(0, 2) = 0;
	(*this)(3, 0) = 0;
	(*this)(3, 1) = 0;
	(*this)(3, 2) = 0;
	(*this)(3, 5) = 0;
	(*this)(4, 0) = 0;
	(*this)(4, 1) = 0;
	(*this)(4, 2) = 0;
	(*this)(5, 0) = 0;
	(*this)(5, 1) = 0;
	(*this)(5, 2) = 0;
}

template <typename TRAIT>
const typename iit::pegasus2::tpl::ForceTransforms<TRAIT>::Type_fr_torso_X_fr_LH_lowerleg& 
iit::pegasus2::tpl::ForceTransforms<TRAIT>::Type_fr_torso_X_fr_LH_lowerleg::update(const JState& q){ 
	Scalar c__q_LH_KFE_joint__;
	Scalar c__q_LH_HAA_joint__;
	Scalar c__q_LH_HFE_joint__;
	Scalar s__q_LH_HFE_joint__;
	Scalar s__q_LH_HAA_joint__;
	Scalar s__q_LH_KFE_joint__;

	c__q_LH_KFE_joint__ = TRAIT::cos( q[LH_KFE_JOINT]);
	c__q_LH_HAA_joint__ = TRAIT::cos( q[LH_HAA_JOINT]);
	c__q_LH_HFE_joint__ = TRAIT::cos( q[LH_HFE_JOINT]);
	s__q_LH_HFE_joint__ = TRAIT::sin( q[LH_HFE_JOINT]);
	s__q_LH_HAA_joint__ = TRAIT::sin( q[LH_HAA_JOINT]);
	s__q_LH_KFE_joint__ = TRAIT::sin( q[LH_KFE_JOINT]);

	(*this)(0, 0) = s__q_LH_HFE_joint__*c__q_LH_KFE_joint__ + s__q_LH_KFE_joint__*c__q_LH_HFE_joint__;
	(*this)(0, 1) = -s__q_LH_HFE_joint__*s__q_LH_KFE_joint__ + c__q_LH_HFE_joint__*c__q_LH_KFE_joint__;
	(*this)(0, 3) = -(-s__q_LH_HFE_joint__*s__q_LH_KFE_joint__ + c__q_LH_HFE_joint__*c__q_LH_KFE_joint__)*(0.1185*c__q_LH_HAA_joint__ + 0.05);
	(*this)(0, 4) = (s__q_LH_HFE_joint__*c__q_LH_KFE_joint__ + s__q_LH_KFE_joint__*c__q_LH_HFE_joint__)*(0.1185*c__q_LH_HAA_joint__ + 0.05);
	(*this)(0, 5) = 0.1185*s__q_LH_HAA_joint__ - 0.38*c__q_LH_HFE_joint__;
	(*this)(1, 0) = -(-s__q_LH_HFE_joint__*s__q_LH_KFE_joint__ + c__q_LH_HFE_joint__*c__q_LH_KFE_joint__)*s__q_LH_HAA_joint__;
	(*this)(1, 1) = (s__q_LH_HFE_joint__*c__q_LH_KFE_joint__ + s__q_LH_KFE_joint__*c__q_LH_HFE_joint__)*s__q_LH_HAA_joint__;
	(*this)(1, 2) = -c__q_LH_HAA_joint__;
	(*this)(1, 3) = -0.21*(s__q_LH_HAA_joint__*s__q_LH_KFE_joint__ + c__q_LH_HAA_joint__*c__q_LH_KFE_joint__)*c__q_LH_HFE_joint__ + 0.21*(-s__q_LH_HAA_joint__*c__q_LH_KFE_joint__ + s__q_LH_KFE_joint__*c__q_LH_HAA_joint__)*s__q_LH_HFE_joint__ - 0.16*(-s__q_LH_HFE_joint__*s__q_LH_KFE_joint__ + c__q_LH_HFE_joint__*c__q_LH_KFE_joint__)*c__q_LH_HAA_joint__ + 0.16*(s__q_LH_HFE_joint__*c__q_LH_KFE_joint__ + s__q_LH_KFE_joint__*c__q_LH_HFE_joint__)*s__q_LH_HAA_joint__ - 0.38*s__q_LH_KFE_joint__*c__q_LH_HAA_joint__;
	(*this)(1, 4) = 0.21*(s__q_LH_HAA_joint__*s__q_LH_KFE_joint__ + c__q_LH_HAA_joint__*c__q_LH_KFE_joint__)*s__q_LH_HFE_joint__ + 0.21*(-s__q_LH_HAA_joint__*c__q_LH_KFE_joint__ + s__q_LH_KFE_joint__*c__q_LH_HAA_joint__)*c__q_LH_HFE_joint__ + 0.16*(-s__q_LH_HFE_joint__*s__q_LH_KFE_joint__ + c__q_LH_HFE_joint__*c__q_LH_KFE_joint__)*s__q_LH_HAA_joint__ + 0.16*(s__q_LH_HFE_joint__*c__q_LH_KFE_joint__ + s__q_LH_KFE_joint__*c__q_LH_HFE_joint__)*c__q_LH_HAA_joint__ - 0.38*c__q_LH_HAA_joint__*c__q_LH_KFE_joint__;
	(*this)(1, 5) = (0.37 - 0.38*s__q_LH_HFE_joint__)*s__q_LH_HAA_joint__;
	(*this)(2, 0) = -(-s__q_LH_HFE_joint__*s__q_LH_KFE_joint__ + c__q_LH_HFE_joint__*c__q_LH_KFE_joint__)*c__q_LH_HAA_joint__;
	(*this)(2, 1) = (s__q_LH_HFE_joint__*c__q_LH_KFE_joint__ + s__q_LH_KFE_joint__*c__q_LH_HFE_joint__)*c__q_LH_HAA_joint__;
	(*this)(2, 2) = s__q_LH_HAA_joint__;
	(*this)(2, 3) = 0.37*(-s__q_LH_HFE_joint__*s__q_LH_KFE_joint__ + c__q_LH_HFE_joint__*c__q_LH_KFE_joint__)*s__q_LH_HAA_joint__ - 0.05*(s__q_LH_HFE_joint__*c__q_LH_KFE_joint__ + s__q_LH_KFE_joint__*c__q_LH_HFE_joint__)*c__q_LH_HAA_joint__ + 0.38*s__q_LH_HAA_joint__*s__q_LH_KFE_joint__ - 0.1185*s__q_LH_HFE_joint__*c__q_LH_KFE_joint__ - 0.1185*s__q_LH_KFE_joint__*c__q_LH_HFE_joint__;
	(*this)(2, 4) = -0.05*(-s__q_LH_HFE_joint__*s__q_LH_KFE_joint__ + c__q_LH_HFE_joint__*c__q_LH_KFE_joint__)*c__q_LH_HAA_joint__ - 0.37*(s__q_LH_HFE_joint__*c__q_LH_KFE_joint__ + s__q_LH_KFE_joint__*c__q_LH_HFE_joint__)*s__q_LH_HAA_joint__ + 0.38*s__q_LH_HAA_joint__*c__q_LH_KFE_joint__ + 0.1185*s__q_LH_HFE_joint__*s__q_LH_KFE_joint__ - 0.1185*c__q_LH_HFE_joint__*c__q_LH_KFE_joint__;
	(*this)(2, 5) = (0.37 - 0.38*s__q_LH_HFE_joint__)*c__q_LH_HAA_joint__;
	(*this)(3, 3) = s__q_LH_HFE_joint__*c__q_LH_KFE_joint__ + s__q_LH_KFE_joint__*c__q_LH_HFE_joint__;
	(*this)(3, 4) = -s__q_LH_HFE_joint__*s__q_LH_KFE_joint__ + c__q_LH_HFE_joint__*c__q_LH_KFE_joint__;
	(*this)(4, 3) = -(-s__q_LH_HFE_joint__*s__q_LH_KFE_joint__ + c__q_LH_HFE_joint__*c__q_LH_KFE_joint__)*s__q_LH_HAA_joint__;
	(*this)(4, 4) = (s__q_LH_HFE_joint__*c__q_LH_KFE_joint__ + s__q_LH_KFE_joint__*c__q_LH_HFE_joint__)*s__q_LH_HAA_joint__;
	(*this)(4, 5) = -c__q_LH_HAA_joint__;
	(*this)(5, 3) = -(-s__q_LH_HFE_joint__*s__q_LH_KFE_joint__ + c__q_LH_HFE_joint__*c__q_LH_KFE_joint__)*c__q_LH_HAA_joint__;
	(*this)(5, 4) = (s__q_LH_HFE_joint__*c__q_LH_KFE_joint__ + s__q_LH_KFE_joint__*c__q_LH_HFE_joint__)*c__q_LH_HAA_joint__;
	(*this)(5, 5) = s__q_LH_HAA_joint__;
	return *this;
}

template <typename TRAIT>
iit::pegasus2::tpl::ForceTransforms<TRAIT>::Type_fr_LH_lowerleg_X_fr_torso::Type_fr_LH_lowerleg_X_fr_torso(){ 
	(*this)(2, 0) = 0;
	(*this)(3, 0) = 0;
	(*this)(3, 1) = 0;
	(*this)(3, 2) = 0;
	(*this)(4, 0) = 0;
	(*this)(4, 1) = 0;
	(*this)(4, 2) = 0;
	(*this)(5, 0) = 0;
	(*this)(5, 1) = 0;
	(*this)(5, 2) = 0;
	(*this)(5, 3) = 0;
}

template <typename TRAIT>
const typename iit::pegasus2::tpl::ForceTransforms<TRAIT>::Type_fr_LH_lowerleg_X_fr_torso& 
iit::pegasus2::tpl::ForceTransforms<TRAIT>::Type_fr_LH_lowerleg_X_fr_torso::update(const JState& q){ 
	Scalar c__q_LH_KFE_joint__;
	Scalar c__q_LH_HAA_joint__;
	Scalar c__q_LH_HFE_joint__;
	Scalar s__q_LH_HFE_joint__;
	Scalar s__q_LH_HAA_joint__;
	Scalar s__q_LH_KFE_joint__;

	c__q_LH_KFE_joint__ = TRAIT::cos( q[LH_KFE_JOINT]);
	c__q_LH_HAA_joint__ = TRAIT::cos( q[LH_HAA_JOINT]);
	c__q_LH_HFE_joint__ = TRAIT::cos( q[LH_HFE_JOINT]);
	s__q_LH_HFE_joint__ = TRAIT::sin( q[LH_HFE_JOINT]);
	s__q_LH_HAA_joint__ = TRAIT::sin( q[LH_HAA_JOINT]);
	s__q_LH_KFE_joint__ = TRAIT::sin( q[LH_KFE_JOINT]);

	(*this)(0, 0) = s__q_LH_HFE_joint__*c__q_LH_KFE_joint__ + s__q_LH_KFE_joint__*c__q_LH_HFE_joint__;
	(*this)(0, 1) = -(-s__q_LH_HFE_joint__*s__q_LH_KFE_joint__ + c__q_LH_HFE_joint__*c__q_LH_KFE_joint__)*s__q_LH_HAA_joint__;
	(*this)(0, 2) = -(-s__q_LH_HFE_joint__*s__q_LH_KFE_joint__ + c__q_LH_HFE_joint__*c__q_LH_KFE_joint__)*c__q_LH_HAA_joint__;
	(*this)(0, 3) = -(-s__q_LH_HFE_joint__*s__q_LH_KFE_joint__ + c__q_LH_HFE_joint__*c__q_LH_KFE_joint__)*(0.1185*c__q_LH_HAA_joint__ + 0.05);
	(*this)(0, 4) = -0.37*(-s__q_LH_HFE_joint__*s__q_LH_KFE_joint__ + c__q_LH_HFE_joint__*c__q_LH_KFE_joint__)*c__q_LH_HAA_joint__ - 0.05*(s__q_LH_HFE_joint__*c__q_LH_KFE_joint__ + s__q_LH_KFE_joint__*c__q_LH_HFE_joint__)*s__q_LH_HAA_joint__ - 0.38*s__q_LH_KFE_joint__*c__q_LH_HAA_joint__;
	(*this)(0, 5) = 0.37*(-s__q_LH_HFE_joint__*s__q_LH_KFE_joint__ + c__q_LH_HFE_joint__*c__q_LH_KFE_joint__)*s__q_LH_HAA_joint__ - 0.05*(s__q_LH_HFE_joint__*c__q_LH_KFE_joint__ + s__q_LH_KFE_joint__*c__q_LH_HFE_joint__)*c__q_LH_HAA_joint__ + 0.38*s__q_LH_HAA_joint__*s__q_LH_KFE_joint__ - 0.1185*s__q_LH_HFE_joint__*c__q_LH_KFE_joint__ - 0.1185*s__q_LH_KFE_joint__*c__q_LH_HFE_joint__;
	(*this)(1, 0) = -s__q_LH_HFE_joint__*s__q_LH_KFE_joint__ + c__q_LH_HFE_joint__*c__q_LH_KFE_joint__;
	(*this)(1, 1) = (s__q_LH_HFE_joint__*c__q_LH_KFE_joint__ + s__q_LH_KFE_joint__*c__q_LH_HFE_joint__)*s__q_LH_HAA_joint__;
	(*this)(1, 2) = (s__q_LH_HFE_joint__*c__q_LH_KFE_joint__ + s__q_LH_KFE_joint__*c__q_LH_HFE_joint__)*c__q_LH_HAA_joint__;
	(*this)(1, 3) = (s__q_LH_HFE_joint__*c__q_LH_KFE_joint__ + s__q_LH_KFE_joint__*c__q_LH_HFE_joint__)*(0.1185*c__q_LH_HAA_joint__ + 0.05);
	(*this)(1, 4) = -0.05*(-s__q_LH_HFE_joint__*s__q_LH_KFE_joint__ + c__q_LH_HFE_joint__*c__q_LH_KFE_joint__)*s__q_LH_HAA_joint__ + 0.37*(s__q_LH_HFE_joint__*c__q_LH_KFE_joint__ + s__q_LH_KFE_joint__*c__q_LH_HFE_joint__)*c__q_LH_HAA_joint__ - 0.38*c__q_LH_HAA_joint__*c__q_LH_KFE_joint__;
	(*this)(1, 5) = -0.05*(-s__q_LH_HFE_joint__*s__q_LH_KFE_joint__ + c__q_LH_HFE_joint__*c__q_LH_KFE_joint__)*c__q_LH_HAA_joint__ - 0.37*(s__q_LH_HFE_joint__*c__q_LH_KFE_joint__ + s__q_LH_KFE_joint__*c__q_LH_HFE_joint__)*s__q_LH_HAA_joint__ + 0.38*s__q_LH_HAA_joint__*c__q_LH_KFE_joint__ + 0.1185*s__q_LH_HFE_joint__*s__q_LH_KFE_joint__ - 0.1185*c__q_LH_HFE_joint__*c__q_LH_KFE_joint__;
	(*this)(2, 1) = -c__q_LH_HAA_joint__;
	(*this)(2, 2) = s__q_LH_HAA_joint__;
	(*this)(2, 3) = 0.1185*s__q_LH_HAA_joint__ - 0.38*c__q_LH_HFE_joint__;
	(*this)(2, 4) = (0.37 - 0.38*s__q_LH_HFE_joint__)*s__q_LH_HAA_joint__;
	(*this)(2, 5) = (0.37 - 0.38*s__q_LH_HFE_joint__)*c__q_LH_HAA_joint__;
	(*this)(3, 3) = s__q_LH_HFE_joint__*c__q_LH_KFE_joint__ + s__q_LH_KFE_joint__*c__q_LH_HFE_joint__;
	(*this)(3, 4) = -(-s__q_LH_HFE_joint__*s__q_LH_KFE_joint__ + c__q_LH_HFE_joint__*c__q_LH_KFE_joint__)*s__q_LH_HAA_joint__;
	(*this)(3, 5) = -(-s__q_LH_HFE_joint__*s__q_LH_KFE_joint__ + c__q_LH_HFE_joint__*c__q_LH_KFE_joint__)*c__q_LH_HAA_joint__;
	(*this)(4, 3) = -s__q_LH_HFE_joint__*s__q_LH_KFE_joint__ + c__q_LH_HFE_joint__*c__q_LH_KFE_joint__;
	(*this)(4, 4) = (s__q_LH_HFE_joint__*c__q_LH_KFE_joint__ + s__q_LH_KFE_joint__*c__q_LH_HFE_joint__)*s__q_LH_HAA_joint__;
	(*this)(4, 5) = (s__q_LH_HFE_joint__*c__q_LH_KFE_joint__ + s__q_LH_KFE_joint__*c__q_LH_HFE_joint__)*c__q_LH_HAA_joint__;
	(*this)(5, 4) = -c__q_LH_HAA_joint__;
	(*this)(5, 5) = s__q_LH_HAA_joint__;
	return *this;
}

template <typename TRAIT>
iit::pegasus2::tpl::ForceTransforms<TRAIT>::Type_fr_torso_X_fr_LF_hip::Type_fr_torso_X_fr_LF_hip(){ 
	(*this)(0, 0) = 0;
	(*this)(0, 1) = 0;
	(*this)(0, 2) = -1;
	(*this)(0, 5) = 0;
	(*this)(1, 2) = 0;
	(*this)(1, 5) = 0;
	(*this)(2, 2) = 0;
	(*this)(2, 5) = 0.118500000000000;
	(*this)(3, 0) = 0;
	(*this)(3, 1) = 0;
	(*this)(3, 2) = 0;
	(*this)(3, 3) = 0;
	(*this)(3, 4) = 0;
	(*this)(3, 5) = -1;
	(*this)(4, 0) = 0;
	(*this)(4, 1) = 0;
	(*this)(4, 2) = 0;
	(*this)(4, 5) = 0;
	(*this)(5, 0) = 0;
	(*this)(5, 1) = 0;
	(*this)(5, 2) = 0;
	(*this)(5, 5) = 0;
}

template <typename TRAIT>
const typename iit::pegasus2::tpl::ForceTransforms<TRAIT>::Type_fr_torso_X_fr_LF_hip& 
iit::pegasus2::tpl::ForceTransforms<TRAIT>::Type_fr_torso_X_fr_LF_hip::update(const JState& q){ 
	Scalar c__q_LF_HAA_joint__;
	Scalar s__q_LF_HAA_joint__;

	c__q_LF_HAA_joint__ = TRAIT::cos( q[LF_HAA_JOINT]);
	s__q_LF_HAA_joint__ = TRAIT::sin( q[LF_HAA_JOINT]);

	(*this)(0, 3) = 0.1185*c__q_LF_HAA_joint__;
	(*this)(0, 4) = -0.1185*s__q_LF_HAA_joint__;
	(*this)(1, 0) = s__q_LF_HAA_joint__;
	(*this)(1, 1) = c__q_LF_HAA_joint__;
	(*this)(1, 3) = -0.37*c__q_LF_HAA_joint__;
	(*this)(1, 4) = 0.37*s__q_LF_HAA_joint__;
	(*this)(2, 0) = c__q_LF_HAA_joint__;
	(*this)(2, 1) = -s__q_LF_HAA_joint__;
	(*this)(2, 3) = 0.37*s__q_LF_HAA_joint__;
	(*this)(2, 4) = 0.37*c__q_LF_HAA_joint__;
	(*this)(4, 3) = s__q_LF_HAA_joint__;
	(*this)(4, 4) = c__q_LF_HAA_joint__;
	(*this)(5, 3) = c__q_LF_HAA_joint__;
	(*this)(5, 4) = -s__q_LF_HAA_joint__;
	return *this;
}

template <typename TRAIT>
iit::pegasus2::tpl::ForceTransforms<TRAIT>::Type_fr_LF_hip_X_fr_torso::Type_fr_LF_hip_X_fr_torso(){ 
	(*this)(0, 0) = 0;
	(*this)(1, 0) = 0;
	(*this)(2, 0) = -1;
	(*this)(2, 1) = 0;
	(*this)(2, 2) = 0;
	(*this)(2, 3) = 0;
	(*this)(2, 4) = 0;
	(*this)(2, 5) = 0.118500000000000;
	(*this)(3, 0) = 0;
	(*this)(3, 1) = 0;
	(*this)(3, 2) = 0;
	(*this)(3, 3) = 0;
	(*this)(4, 0) = 0;
	(*this)(4, 1) = 0;
	(*this)(4, 2) = 0;
	(*this)(4, 3) = 0;
	(*this)(5, 0) = 0;
	(*this)(5, 1) = 0;
	(*this)(5, 2) = 0;
	(*this)(5, 3) = -1;
	(*this)(5, 4) = 0;
	(*this)(5, 5) = 0;
}

template <typename TRAIT>
const typename iit::pegasus2::tpl::ForceTransforms<TRAIT>::Type_fr_LF_hip_X_fr_torso& 
iit::pegasus2::tpl::ForceTransforms<TRAIT>::Type_fr_LF_hip_X_fr_torso::update(const JState& q){ 
	Scalar c__q_LF_HAA_joint__;
	Scalar s__q_LF_HAA_joint__;

	c__q_LF_HAA_joint__ = TRAIT::cos( q[LF_HAA_JOINT]);
	s__q_LF_HAA_joint__ = TRAIT::sin( q[LF_HAA_JOINT]);

	(*this)(0, 1) = s__q_LF_HAA_joint__;
	(*this)(0, 2) = c__q_LF_HAA_joint__;
	(*this)(0, 3) = 0.1185*c__q_LF_HAA_joint__;
	(*this)(0, 4) = -0.37*c__q_LF_HAA_joint__;
	(*this)(0, 5) = 0.37*s__q_LF_HAA_joint__;
	(*this)(1, 1) = c__q_LF_HAA_joint__;
	(*this)(1, 2) = -s__q_LF_HAA_joint__;
	(*this)(1, 3) = -0.1185*s__q_LF_HAA_joint__;
	(*this)(1, 4) = 0.37*s__q_LF_HAA_joint__;
	(*this)(1, 5) = 0.37*c__q_LF_HAA_joint__;
	(*this)(3, 4) = s__q_LF_HAA_joint__;
	(*this)(3, 5) = c__q_LF_HAA_joint__;
	(*this)(4, 4) = c__q_LF_HAA_joint__;
	(*this)(4, 5) = -s__q_LF_HAA_joint__;
	return *this;
}

template <typename TRAIT>
iit::pegasus2::tpl::ForceTransforms<TRAIT>::Type_fr_torso_X_fr_LF_upperleg::Type_fr_torso_X_fr_LF_upperleg(){ 
	(*this)(0, 2) = 0;
	(*this)(3, 0) = 0;
	(*this)(3, 1) = 0;
	(*this)(3, 2) = 0;
	(*this)(3, 5) = 0;
	(*this)(4, 0) = 0;
	(*this)(4, 1) = 0;
	(*this)(4, 2) = 0;
	(*this)(5, 0) = 0;
	(*this)(5, 1) = 0;
	(*this)(5, 2) = 0;
}

template <typename TRAIT>
const typename iit::pegasus2::tpl::ForceTransforms<TRAIT>::Type_fr_torso_X_fr_LF_upperleg& 
iit::pegasus2::tpl::ForceTransforms<TRAIT>::Type_fr_torso_X_fr_LF_upperleg::update(const JState& q){ 
	Scalar c__q_LF_HFE_joint__;
	Scalar c__q_LF_HAA_joint__;
	Scalar s__q_LF_HAA_joint__;
	Scalar s__q_LF_HFE_joint__;

	c__q_LF_HFE_joint__ = TRAIT::cos( q[LF_HFE_JOINT]);
	c__q_LF_HAA_joint__ = TRAIT::cos( q[LF_HAA_JOINT]);
	s__q_LF_HAA_joint__ = TRAIT::sin( q[LF_HAA_JOINT]);
	s__q_LF_HFE_joint__ = TRAIT::sin( q[LF_HFE_JOINT]);

	(*this)(0, 0) = s__q_LF_HFE_joint__;
	(*this)(0, 1) = c__q_LF_HFE_joint__;
	(*this)(0, 3) = -(0.1185*c__q_LF_HAA_joint__ + 0.05)*c__q_LF_HFE_joint__;
	(*this)(0, 4) = (0.1185*c__q_LF_HAA_joint__ + 0.05)*s__q_LF_HFE_joint__;
	(*this)(0, 5) = 0.1185*s__q_LF_HAA_joint__;
	(*this)(1, 0) = -s__q_LF_HAA_joint__*c__q_LF_HFE_joint__;
	(*this)(1, 1) = s__q_LF_HAA_joint__*s__q_LF_HFE_joint__;
	(*this)(1, 2) = -c__q_LF_HAA_joint__;
	(*this)(1, 3) = -0.05*s__q_LF_HAA_joint__*s__q_LF_HFE_joint__ + 0.37*c__q_LF_HAA_joint__*c__q_LF_HFE_joint__;
	(*this)(1, 4) = -0.05*s__q_LF_HAA_joint__*c__q_LF_HFE_joint__ - 0.37*s__q_LF_HFE_joint__*c__q_LF_HAA_joint__;
	(*this)(1, 5) = -0.37*s__q_LF_HAA_joint__;
	(*this)(2, 0) = -c__q_LF_HAA_joint__*c__q_LF_HFE_joint__;
	(*this)(2, 1) = s__q_LF_HFE_joint__*c__q_LF_HAA_joint__;
	(*this)(2, 2) = s__q_LF_HAA_joint__;
	(*this)(2, 3) = -0.37*s__q_LF_HAA_joint__*c__q_LF_HFE_joint__ - 0.05*s__q_LF_HFE_joint__*c__q_LF_HAA_joint__ - 0.1185*s__q_LF_HFE_joint__;
	(*this)(2, 4) = 0.37*s__q_LF_HAA_joint__*s__q_LF_HFE_joint__ - 0.05*c__q_LF_HAA_joint__*c__q_LF_HFE_joint__ - 0.1185*c__q_LF_HFE_joint__;
	(*this)(2, 5) = -0.37*c__q_LF_HAA_joint__;
	(*this)(3, 3) = s__q_LF_HFE_joint__;
	(*this)(3, 4) = c__q_LF_HFE_joint__;
	(*this)(4, 3) = -s__q_LF_HAA_joint__*c__q_LF_HFE_joint__;
	(*this)(4, 4) = s__q_LF_HAA_joint__*s__q_LF_HFE_joint__;
	(*this)(4, 5) = -c__q_LF_HAA_joint__;
	(*this)(5, 3) = -c__q_LF_HAA_joint__*c__q_LF_HFE_joint__;
	(*this)(5, 4) = s__q_LF_HFE_joint__*c__q_LF_HAA_joint__;
	(*this)(5, 5) = s__q_LF_HAA_joint__;
	return *this;
}

template <typename TRAIT>
iit::pegasus2::tpl::ForceTransforms<TRAIT>::Type_fr_LF_upperleg_X_fr_torso::Type_fr_LF_upperleg_X_fr_torso(){ 
	(*this)(2, 0) = 0;
	(*this)(3, 0) = 0;
	(*this)(3, 1) = 0;
	(*this)(3, 2) = 0;
	(*this)(4, 0) = 0;
	(*this)(4, 1) = 0;
	(*this)(4, 2) = 0;
	(*this)(5, 0) = 0;
	(*this)(5, 1) = 0;
	(*this)(5, 2) = 0;
	(*this)(5, 3) = 0;
}

template <typename TRAIT>
const typename iit::pegasus2::tpl::ForceTransforms<TRAIT>::Type_fr_LF_upperleg_X_fr_torso& 
iit::pegasus2::tpl::ForceTransforms<TRAIT>::Type_fr_LF_upperleg_X_fr_torso::update(const JState& q){ 
	Scalar c__q_LF_HFE_joint__;
	Scalar c__q_LF_HAA_joint__;
	Scalar s__q_LF_HAA_joint__;
	Scalar s__q_LF_HFE_joint__;

	c__q_LF_HFE_joint__ = TRAIT::cos( q[LF_HFE_JOINT]);
	c__q_LF_HAA_joint__ = TRAIT::cos( q[LF_HAA_JOINT]);
	s__q_LF_HAA_joint__ = TRAIT::sin( q[LF_HAA_JOINT]);
	s__q_LF_HFE_joint__ = TRAIT::sin( q[LF_HFE_JOINT]);

	(*this)(0, 0) = s__q_LF_HFE_joint__;
	(*this)(0, 1) = -s__q_LF_HAA_joint__*c__q_LF_HFE_joint__;
	(*this)(0, 2) = -c__q_LF_HAA_joint__*c__q_LF_HFE_joint__;
	(*this)(0, 3) = -(0.1185*c__q_LF_HAA_joint__ + 0.05)*c__q_LF_HFE_joint__;
	(*this)(0, 4) = -0.05*s__q_LF_HAA_joint__*s__q_LF_HFE_joint__ + 0.37*c__q_LF_HAA_joint__*c__q_LF_HFE_joint__;
	(*this)(0, 5) = -0.37*s__q_LF_HAA_joint__*c__q_LF_HFE_joint__ - 0.05*s__q_LF_HFE_joint__*c__q_LF_HAA_joint__ - 0.1185*s__q_LF_HFE_joint__;
	(*this)(1, 0) = c__q_LF_HFE_joint__;
	(*this)(1, 1) = s__q_LF_HAA_joint__*s__q_LF_HFE_joint__;
	(*this)(1, 2) = s__q_LF_HFE_joint__*c__q_LF_HAA_joint__;
	(*this)(1, 3) = (0.1185*c__q_LF_HAA_joint__ + 0.05)*s__q_LF_HFE_joint__;
	(*this)(1, 4) = -0.05*s__q_LF_HAA_joint__*c__q_LF_HFE_joint__ - 0.37*s__q_LF_HFE_joint__*c__q_LF_HAA_joint__;
	(*this)(1, 5) = 0.37*s__q_LF_HAA_joint__*s__q_LF_HFE_joint__ - 0.05*c__q_LF_HAA_joint__*c__q_LF_HFE_joint__ - 0.1185*c__q_LF_HFE_joint__;
	(*this)(2, 1) = -c__q_LF_HAA_joint__;
	(*this)(2, 2) = s__q_LF_HAA_joint__;
	(*this)(2, 3) = 0.1185*s__q_LF_HAA_joint__;
	(*this)(2, 4) = -0.37*s__q_LF_HAA_joint__;
	(*this)(2, 5) = -0.37*c__q_LF_HAA_joint__;
	(*this)(3, 3) = s__q_LF_HFE_joint__;
	(*this)(3, 4) = -s__q_LF_HAA_joint__*c__q_LF_HFE_joint__;
	(*this)(3, 5) = -c__q_LF_HAA_joint__*c__q_LF_HFE_joint__;
	(*this)(4, 3) = c__q_LF_HFE_joint__;
	(*this)(4, 4) = s__q_LF_HAA_joint__*s__q_LF_HFE_joint__;
	(*this)(4, 5) = s__q_LF_HFE_joint__*c__q_LF_HAA_joint__;
	(*this)(5, 4) = -c__q_LF_HAA_joint__;
	(*this)(5, 5) = s__q_LF_HAA_joint__;
	return *this;
}

template <typename TRAIT>
iit::pegasus2::tpl::ForceTransforms<TRAIT>::Type_fr_torso_X_fr_LF_lowerleg::Type_fr_torso_X_fr_LF_lowerleg(){ 
	(*this)(0, 2) = 0;
	(*this)(3, 0) = 0;
	(*this)(3, 1) = 0;
	(*this)(3, 2) = 0;
	(*this)(3, 5) = 0;
	(*this)(4, 0) = 0;
	(*this)(4, 1) = 0;
	(*this)(4, 2) = 0;
	(*this)(5, 0) = 0;
	(*this)(5, 1) = 0;
	(*this)(5, 2) = 0;
}

template <typename TRAIT>
const typename iit::pegasus2::tpl::ForceTransforms<TRAIT>::Type_fr_torso_X_fr_LF_lowerleg& 
iit::pegasus2::tpl::ForceTransforms<TRAIT>::Type_fr_torso_X_fr_LF_lowerleg::update(const JState& q){ 
	Scalar c__q_LF_KFE_joint__;
	Scalar c__q_LF_HFE_joint__;
	Scalar c__q_LF_HAA_joint__;
	Scalar s__q_LF_HAA_joint__;
	Scalar s__q_LF_KFE_joint__;
	Scalar s__q_LF_HFE_joint__;

	c__q_LF_KFE_joint__ = TRAIT::cos( q[LF_KFE_JOINT]);
	c__q_LF_HFE_joint__ = TRAIT::cos( q[LF_HFE_JOINT]);
	c__q_LF_HAA_joint__ = TRAIT::cos( q[LF_HAA_JOINT]);
	s__q_LF_HAA_joint__ = TRAIT::sin( q[LF_HAA_JOINT]);
	s__q_LF_KFE_joint__ = TRAIT::sin( q[LF_KFE_JOINT]);
	s__q_LF_HFE_joint__ = TRAIT::sin( q[LF_HFE_JOINT]);

	(*this)(0, 0) = s__q_LF_HFE_joint__*c__q_LF_KFE_joint__ + s__q_LF_KFE_joint__*c__q_LF_HFE_joint__;
	(*this)(0, 1) = -s__q_LF_HFE_joint__*s__q_LF_KFE_joint__ + c__q_LF_HFE_joint__*c__q_LF_KFE_joint__;
	(*this)(0, 3) = -(-s__q_LF_HFE_joint__*s__q_LF_KFE_joint__ + c__q_LF_HFE_joint__*c__q_LF_KFE_joint__)*(0.1185*c__q_LF_HAA_joint__ + 0.05);
	(*this)(0, 4) = (s__q_LF_HFE_joint__*c__q_LF_KFE_joint__ + s__q_LF_KFE_joint__*c__q_LF_HFE_joint__)*(0.1185*c__q_LF_HAA_joint__ + 0.05);
	(*this)(0, 5) = 0.1185*s__q_LF_HAA_joint__ - 0.38*c__q_LF_HFE_joint__;
	(*this)(1, 0) = -(-s__q_LF_HFE_joint__*s__q_LF_KFE_joint__ + c__q_LF_HFE_joint__*c__q_LF_KFE_joint__)*s__q_LF_HAA_joint__;
	(*this)(1, 1) = (s__q_LF_HFE_joint__*c__q_LF_KFE_joint__ + s__q_LF_KFE_joint__*c__q_LF_HFE_joint__)*s__q_LF_HAA_joint__;
	(*this)(1, 2) = -c__q_LF_HAA_joint__;
	(*this)(1, 3) = 0.16*(s__q_LF_HAA_joint__*s__q_LF_KFE_joint__ + c__q_LF_HAA_joint__*c__q_LF_KFE_joint__)*c__q_LF_HFE_joint__ - 0.16*(-s__q_LF_HAA_joint__*c__q_LF_KFE_joint__ + s__q_LF_KFE_joint__*c__q_LF_HAA_joint__)*s__q_LF_HFE_joint__ + 0.21*(-s__q_LF_HFE_joint__*s__q_LF_KFE_joint__ + c__q_LF_HFE_joint__*c__q_LF_KFE_joint__)*c__q_LF_HAA_joint__ - 0.21*(s__q_LF_HFE_joint__*c__q_LF_KFE_joint__ + s__q_LF_KFE_joint__*c__q_LF_HFE_joint__)*s__q_LF_HAA_joint__ - 0.38*s__q_LF_KFE_joint__*c__q_LF_HAA_joint__;
	(*this)(1, 4) = -0.16*(s__q_LF_HAA_joint__*s__q_LF_KFE_joint__ + c__q_LF_HAA_joint__*c__q_LF_KFE_joint__)*s__q_LF_HFE_joint__ - 0.16*(-s__q_LF_HAA_joint__*c__q_LF_KFE_joint__ + s__q_LF_KFE_joint__*c__q_LF_HAA_joint__)*c__q_LF_HFE_joint__ - 0.21*(-s__q_LF_HFE_joint__*s__q_LF_KFE_joint__ + c__q_LF_HFE_joint__*c__q_LF_KFE_joint__)*s__q_LF_HAA_joint__ - 0.21*(s__q_LF_HFE_joint__*c__q_LF_KFE_joint__ + s__q_LF_KFE_joint__*c__q_LF_HFE_joint__)*c__q_LF_HAA_joint__ - 0.38*c__q_LF_HAA_joint__*c__q_LF_KFE_joint__;
	(*this)(1, 5) = -(0.38*s__q_LF_HFE_joint__ + 0.37)*s__q_LF_HAA_joint__;
	(*this)(2, 0) = -(-s__q_LF_HFE_joint__*s__q_LF_KFE_joint__ + c__q_LF_HFE_joint__*c__q_LF_KFE_joint__)*c__q_LF_HAA_joint__;
	(*this)(2, 1) = (s__q_LF_HFE_joint__*c__q_LF_KFE_joint__ + s__q_LF_KFE_joint__*c__q_LF_HFE_joint__)*c__q_LF_HAA_joint__;
	(*this)(2, 2) = s__q_LF_HAA_joint__;
	(*this)(2, 3) = -0.37*(-s__q_LF_HFE_joint__*s__q_LF_KFE_joint__ + c__q_LF_HFE_joint__*c__q_LF_KFE_joint__)*s__q_LF_HAA_joint__ - 0.05*(s__q_LF_HFE_joint__*c__q_LF_KFE_joint__ + s__q_LF_KFE_joint__*c__q_LF_HFE_joint__)*c__q_LF_HAA_joint__ + 0.38*s__q_LF_HAA_joint__*s__q_LF_KFE_joint__ - 0.1185*s__q_LF_HFE_joint__*c__q_LF_KFE_joint__ - 0.1185*s__q_LF_KFE_joint__*c__q_LF_HFE_joint__;
	(*this)(2, 4) = -0.05*(-s__q_LF_HFE_joint__*s__q_LF_KFE_joint__ + c__q_LF_HFE_joint__*c__q_LF_KFE_joint__)*c__q_LF_HAA_joint__ + 0.37*(s__q_LF_HFE_joint__*c__q_LF_KFE_joint__ + s__q_LF_KFE_joint__*c__q_LF_HFE_joint__)*s__q_LF_HAA_joint__ + 0.38*s__q_LF_HAA_joint__*c__q_LF_KFE_joint__ + 0.1185*s__q_LF_HFE_joint__*s__q_LF_KFE_joint__ - 0.1185*c__q_LF_HFE_joint__*c__q_LF_KFE_joint__;
	(*this)(2, 5) = -(0.38*s__q_LF_HFE_joint__ + 0.37)*c__q_LF_HAA_joint__;
	(*this)(3, 3) = s__q_LF_HFE_joint__*c__q_LF_KFE_joint__ + s__q_LF_KFE_joint__*c__q_LF_HFE_joint__;
	(*this)(3, 4) = -s__q_LF_HFE_joint__*s__q_LF_KFE_joint__ + c__q_LF_HFE_joint__*c__q_LF_KFE_joint__;
	(*this)(4, 3) = -(-s__q_LF_HFE_joint__*s__q_LF_KFE_joint__ + c__q_LF_HFE_joint__*c__q_LF_KFE_joint__)*s__q_LF_HAA_joint__;
	(*this)(4, 4) = (s__q_LF_HFE_joint__*c__q_LF_KFE_joint__ + s__q_LF_KFE_joint__*c__q_LF_HFE_joint__)*s__q_LF_HAA_joint__;
	(*this)(4, 5) = -c__q_LF_HAA_joint__;
	(*this)(5, 3) = -(-s__q_LF_HFE_joint__*s__q_LF_KFE_joint__ + c__q_LF_HFE_joint__*c__q_LF_KFE_joint__)*c__q_LF_HAA_joint__;
	(*this)(5, 4) = (s__q_LF_HFE_joint__*c__q_LF_KFE_joint__ + s__q_LF_KFE_joint__*c__q_LF_HFE_joint__)*c__q_LF_HAA_joint__;
	(*this)(5, 5) = s__q_LF_HAA_joint__;
	return *this;
}

template <typename TRAIT>
iit::pegasus2::tpl::ForceTransforms<TRAIT>::Type_fr_LF_lowerleg_X_fr_torso::Type_fr_LF_lowerleg_X_fr_torso(){ 
	(*this)(2, 0) = 0;
	(*this)(3, 0) = 0;
	(*this)(3, 1) = 0;
	(*this)(3, 2) = 0;
	(*this)(4, 0) = 0;
	(*this)(4, 1) = 0;
	(*this)(4, 2) = 0;
	(*this)(5, 0) = 0;
	(*this)(5, 1) = 0;
	(*this)(5, 2) = 0;
	(*this)(5, 3) = 0;
}

template <typename TRAIT>
const typename iit::pegasus2::tpl::ForceTransforms<TRAIT>::Type_fr_LF_lowerleg_X_fr_torso& 
iit::pegasus2::tpl::ForceTransforms<TRAIT>::Type_fr_LF_lowerleg_X_fr_torso::update(const JState& q){ 
	Scalar c__q_LF_KFE_joint__;
	Scalar c__q_LF_HFE_joint__;
	Scalar c__q_LF_HAA_joint__;
	Scalar s__q_LF_HAA_joint__;
	Scalar s__q_LF_KFE_joint__;
	Scalar s__q_LF_HFE_joint__;

	c__q_LF_KFE_joint__ = TRAIT::cos( q[LF_KFE_JOINT]);
	c__q_LF_HFE_joint__ = TRAIT::cos( q[LF_HFE_JOINT]);
	c__q_LF_HAA_joint__ = TRAIT::cos( q[LF_HAA_JOINT]);
	s__q_LF_HAA_joint__ = TRAIT::sin( q[LF_HAA_JOINT]);
	s__q_LF_KFE_joint__ = TRAIT::sin( q[LF_KFE_JOINT]);
	s__q_LF_HFE_joint__ = TRAIT::sin( q[LF_HFE_JOINT]);

	(*this)(0, 0) = s__q_LF_HFE_joint__*c__q_LF_KFE_joint__ + s__q_LF_KFE_joint__*c__q_LF_HFE_joint__;
	(*this)(0, 1) = -(-s__q_LF_HFE_joint__*s__q_LF_KFE_joint__ + c__q_LF_HFE_joint__*c__q_LF_KFE_joint__)*s__q_LF_HAA_joint__;
	(*this)(0, 2) = -(-s__q_LF_HFE_joint__*s__q_LF_KFE_joint__ + c__q_LF_HFE_joint__*c__q_LF_KFE_joint__)*c__q_LF_HAA_joint__;
	(*this)(0, 3) = -(-s__q_LF_HFE_joint__*s__q_LF_KFE_joint__ + c__q_LF_HFE_joint__*c__q_LF_KFE_joint__)*(0.1185*c__q_LF_HAA_joint__ + 0.05);
	(*this)(0, 4) = 0.37*(-s__q_LF_HFE_joint__*s__q_LF_KFE_joint__ + c__q_LF_HFE_joint__*c__q_LF_KFE_joint__)*c__q_LF_HAA_joint__ - 0.05*(s__q_LF_HFE_joint__*c__q_LF_KFE_joint__ + s__q_LF_KFE_joint__*c__q_LF_HFE_joint__)*s__q_LF_HAA_joint__ - 0.38*s__q_LF_KFE_joint__*c__q_LF_HAA_joint__;
	(*this)(0, 5) = -0.37*(-s__q_LF_HFE_joint__*s__q_LF_KFE_joint__ + c__q_LF_HFE_joint__*c__q_LF_KFE_joint__)*s__q_LF_HAA_joint__ - 0.05*(s__q_LF_HFE_joint__*c__q_LF_KFE_joint__ + s__q_LF_KFE_joint__*c__q_LF_HFE_joint__)*c__q_LF_HAA_joint__ + 0.38*s__q_LF_HAA_joint__*s__q_LF_KFE_joint__ - 0.1185*s__q_LF_HFE_joint__*c__q_LF_KFE_joint__ - 0.1185*s__q_LF_KFE_joint__*c__q_LF_HFE_joint__;
	(*this)(1, 0) = -s__q_LF_HFE_joint__*s__q_LF_KFE_joint__ + c__q_LF_HFE_joint__*c__q_LF_KFE_joint__;
	(*this)(1, 1) = (s__q_LF_HFE_joint__*c__q_LF_KFE_joint__ + s__q_LF_KFE_joint__*c__q_LF_HFE_joint__)*s__q_LF_HAA_joint__;
	(*this)(1, 2) = (s__q_LF_HFE_joint__*c__q_LF_KFE_joint__ + s__q_LF_KFE_joint__*c__q_LF_HFE_joint__)*c__q_LF_HAA_joint__;
	(*this)(1, 3) = (s__q_LF_HFE_joint__*c__q_LF_KFE_joint__ + s__q_LF_KFE_joint__*c__q_LF_HFE_joint__)*(0.1185*c__q_LF_HAA_joint__ + 0.05);
	(*this)(1, 4) = -0.05*(-s__q_LF_HFE_joint__*s__q_LF_KFE_joint__ + c__q_LF_HFE_joint__*c__q_LF_KFE_joint__)*s__q_LF_HAA_joint__ - 0.37*(s__q_LF_HFE_joint__*c__q_LF_KFE_joint__ + s__q_LF_KFE_joint__*c__q_LF_HFE_joint__)*c__q_LF_HAA_joint__ - 0.38*c__q_LF_HAA_joint__*c__q_LF_KFE_joint__;
	(*this)(1, 5) = -0.05*(-s__q_LF_HFE_joint__*s__q_LF_KFE_joint__ + c__q_LF_HFE_joint__*c__q_LF_KFE_joint__)*c__q_LF_HAA_joint__ + 0.37*(s__q_LF_HFE_joint__*c__q_LF_KFE_joint__ + s__q_LF_KFE_joint__*c__q_LF_HFE_joint__)*s__q_LF_HAA_joint__ + 0.38*s__q_LF_HAA_joint__*c__q_LF_KFE_joint__ + 0.1185*s__q_LF_HFE_joint__*s__q_LF_KFE_joint__ - 0.1185*c__q_LF_HFE_joint__*c__q_LF_KFE_joint__;
	(*this)(2, 1) = -c__q_LF_HAA_joint__;
	(*this)(2, 2) = s__q_LF_HAA_joint__;
	(*this)(2, 3) = 0.1185*s__q_LF_HAA_joint__ - 0.38*c__q_LF_HFE_joint__;
	(*this)(2, 4) = -(0.38*s__q_LF_HFE_joint__ + 0.37)*s__q_LF_HAA_joint__;
	(*this)(2, 5) = -(0.38*s__q_LF_HFE_joint__ + 0.37)*c__q_LF_HAA_joint__;
	(*this)(3, 3) = s__q_LF_HFE_joint__*c__q_LF_KFE_joint__ + s__q_LF_KFE_joint__*c__q_LF_HFE_joint__;
	(*this)(3, 4) = -(-s__q_LF_HFE_joint__*s__q_LF_KFE_joint__ + c__q_LF_HFE_joint__*c__q_LF_KFE_joint__)*s__q_LF_HAA_joint__;
	(*this)(3, 5) = -(-s__q_LF_HFE_joint__*s__q_LF_KFE_joint__ + c__q_LF_HFE_joint__*c__q_LF_KFE_joint__)*c__q_LF_HAA_joint__;
	(*this)(4, 3) = -s__q_LF_HFE_joint__*s__q_LF_KFE_joint__ + c__q_LF_HFE_joint__*c__q_LF_KFE_joint__;
	(*this)(4, 4) = (s__q_LF_HFE_joint__*c__q_LF_KFE_joint__ + s__q_LF_KFE_joint__*c__q_LF_HFE_joint__)*s__q_LF_HAA_joint__;
	(*this)(4, 5) = (s__q_LF_HFE_joint__*c__q_LF_KFE_joint__ + s__q_LF_KFE_joint__*c__q_LF_HFE_joint__)*c__q_LF_HAA_joint__;
	(*this)(5, 4) = -c__q_LF_HAA_joint__;
	(*this)(5, 5) = s__q_LF_HAA_joint__;
	return *this;
}

template <typename TRAIT>
iit::pegasus2::tpl::ForceTransforms<TRAIT>::Type_fr_torso_X_fr_RF_foot::Type_fr_torso_X_fr_RF_foot(){ 
	(*this)(0, 2) = 0;
	(*this)(3, 0) = 0;
	(*this)(3, 1) = 0;
	(*this)(3, 2) = 0;
	(*this)(3, 5) = 0;
	(*this)(4, 0) = 0;
	(*this)(4, 1) = 0;
	(*this)(4, 2) = 0;
	(*this)(5, 0) = 0;
	(*this)(5, 1) = 0;
	(*this)(5, 2) = 0;
}

template <typename TRAIT>
const typename iit::pegasus2::tpl::ForceTransforms<TRAIT>::Type_fr_torso_X_fr_RF_foot& 
iit::pegasus2::tpl::ForceTransforms<TRAIT>::Type_fr_torso_X_fr_RF_foot::update(const JState& q){ 
	Scalar c__q_RF_KFE_joint__;
	Scalar c__q_RF_HAA_joint__;
	Scalar c__q_RF_HFE_joint__;
	Scalar s__q_RF_HFE_joint__;
	Scalar s__q_RF_HAA_joint__;
	Scalar s__q_RF_KFE_joint__;

	c__q_RF_KFE_joint__ = TRAIT::cos( q[RF_KFE_JOINT]);
	c__q_RF_HAA_joint__ = TRAIT::cos( q[RF_HAA_JOINT]);
	c__q_RF_HFE_joint__ = TRAIT::cos( q[RF_HFE_JOINT]);
	s__q_RF_HFE_joint__ = TRAIT::sin( q[RF_HFE_JOINT]);
	s__q_RF_HAA_joint__ = TRAIT::sin( q[RF_HAA_JOINT]);
	s__q_RF_KFE_joint__ = TRAIT::sin( q[RF_KFE_JOINT]);

	(*this)(0, 0) = s__q_RF_HFE_joint__*c__q_RF_KFE_joint__ + s__q_RF_KFE_joint__*c__q_RF_HFE_joint__;
	(*this)(0, 1) = -s__q_RF_HFE_joint__*s__q_RF_KFE_joint__ + c__q_RF_HFE_joint__*c__q_RF_KFE_joint__;
	(*this)(0, 3) = (-s__q_RF_HFE_joint__*s__q_RF_KFE_joint__ + c__q_RF_HFE_joint__*c__q_RF_KFE_joint__)*(0.1185*c__q_RF_HAA_joint__ + 0.05);
	(*this)(0, 4) = -(s__q_RF_HFE_joint__*c__q_RF_KFE_joint__ + s__q_RF_KFE_joint__*c__q_RF_HFE_joint__)*(0.1185*c__q_RF_HAA_joint__ + 0.05);
	(*this)(0, 5) = -0.1185*s__q_RF_HAA_joint__ + 0.3612*s__q_RF_HFE_joint__*s__q_RF_KFE_joint__ - 0.3612*c__q_RF_HFE_joint__*c__q_RF_KFE_joint__ - 0.38*c__q_RF_HFE_joint__;
	(*this)(1, 0) = -(-s__q_RF_HFE_joint__*s__q_RF_KFE_joint__ + c__q_RF_HFE_joint__*c__q_RF_KFE_joint__)*s__q_RF_HAA_joint__;
	(*this)(1, 1) = (s__q_RF_HFE_joint__*c__q_RF_KFE_joint__ + s__q_RF_KFE_joint__*c__q_RF_HFE_joint__)*s__q_RF_HAA_joint__;
	(*this)(1, 2) = -c__q_RF_HAA_joint__;
	(*this)(1, 3) = 0.21*(s__q_RF_HAA_joint__*s__q_RF_KFE_joint__ + c__q_RF_HAA_joint__*c__q_RF_KFE_joint__)*c__q_RF_HFE_joint__ - 0.21*(-s__q_RF_HAA_joint__*c__q_RF_KFE_joint__ + s__q_RF_KFE_joint__*c__q_RF_HAA_joint__)*s__q_RF_HFE_joint__ + 0.16*(-s__q_RF_HFE_joint__*s__q_RF_KFE_joint__ + c__q_RF_HFE_joint__*c__q_RF_KFE_joint__)*c__q_RF_HAA_joint__ - 0.16*(s__q_RF_HFE_joint__*c__q_RF_KFE_joint__ + s__q_RF_KFE_joint__*c__q_RF_HFE_joint__)*s__q_RF_HAA_joint__ - 0.38*s__q_RF_KFE_joint__*c__q_RF_HAA_joint__;
	(*this)(1, 4) = -0.21*(s__q_RF_HAA_joint__*s__q_RF_KFE_joint__ + c__q_RF_HAA_joint__*c__q_RF_KFE_joint__)*s__q_RF_HFE_joint__ - 0.21*(-s__q_RF_HAA_joint__*c__q_RF_KFE_joint__ + s__q_RF_KFE_joint__*c__q_RF_HAA_joint__)*c__q_RF_HFE_joint__ - 0.16*(-s__q_RF_HFE_joint__*s__q_RF_KFE_joint__ + c__q_RF_HFE_joint__*c__q_RF_KFE_joint__)*s__q_RF_HAA_joint__ - 0.16*(s__q_RF_HFE_joint__*c__q_RF_KFE_joint__ + s__q_RF_KFE_joint__*c__q_RF_HFE_joint__)*c__q_RF_HAA_joint__ - 0.38*c__q_RF_HAA_joint__*c__q_RF_KFE_joint__ - 0.3612*c__q_RF_HAA_joint__;
	(*this)(1, 5) = -(0.3612*s__q_RF_HFE_joint__*c__q_RF_KFE_joint__ + 0.38*s__q_RF_HFE_joint__ + 0.3612*s__q_RF_KFE_joint__*c__q_RF_HFE_joint__ + 0.37)*s__q_RF_HAA_joint__;
	(*this)(2, 0) = -(-s__q_RF_HFE_joint__*s__q_RF_KFE_joint__ + c__q_RF_HFE_joint__*c__q_RF_KFE_joint__)*c__q_RF_HAA_joint__;
	(*this)(2, 1) = (s__q_RF_HFE_joint__*c__q_RF_KFE_joint__ + s__q_RF_KFE_joint__*c__q_RF_HFE_joint__)*c__q_RF_HAA_joint__;
	(*this)(2, 2) = s__q_RF_HAA_joint__;
	(*this)(2, 3) = -0.37*(-s__q_RF_HFE_joint__*s__q_RF_KFE_joint__ + c__q_RF_HFE_joint__*c__q_RF_KFE_joint__)*s__q_RF_HAA_joint__ + 0.05*(s__q_RF_HFE_joint__*c__q_RF_KFE_joint__ + s__q_RF_KFE_joint__*c__q_RF_HFE_joint__)*c__q_RF_HAA_joint__ + 0.38*s__q_RF_HAA_joint__*s__q_RF_KFE_joint__ + 0.1185*s__q_RF_HFE_joint__*c__q_RF_KFE_joint__ + 0.1185*s__q_RF_KFE_joint__*c__q_RF_HFE_joint__;
	(*this)(2, 4) = 0.05*(-s__q_RF_HFE_joint__*s__q_RF_KFE_joint__ + c__q_RF_HFE_joint__*c__q_RF_KFE_joint__)*c__q_RF_HAA_joint__ + 0.37*(s__q_RF_HFE_joint__*c__q_RF_KFE_joint__ + s__q_RF_KFE_joint__*c__q_RF_HFE_joint__)*s__q_RF_HAA_joint__ + 0.38*s__q_RF_HAA_joint__*c__q_RF_KFE_joint__ + 0.3612*s__q_RF_HAA_joint__ - 0.1185*s__q_RF_HFE_joint__*s__q_RF_KFE_joint__ + 0.1185*c__q_RF_HFE_joint__*c__q_RF_KFE_joint__;
	(*this)(2, 5) = -(0.3612*s__q_RF_HFE_joint__*c__q_RF_KFE_joint__ + 0.38*s__q_RF_HFE_joint__ + 0.3612*s__q_RF_KFE_joint__*c__q_RF_HFE_joint__ + 0.37)*c__q_RF_HAA_joint__;
	(*this)(3, 3) = s__q_RF_HFE_joint__*c__q_RF_KFE_joint__ + s__q_RF_KFE_joint__*c__q_RF_HFE_joint__;
	(*this)(3, 4) = -s__q_RF_HFE_joint__*s__q_RF_KFE_joint__ + c__q_RF_HFE_joint__*c__q_RF_KFE_joint__;
	(*this)(4, 3) = -(-s__q_RF_HFE_joint__*s__q_RF_KFE_joint__ + c__q_RF_HFE_joint__*c__q_RF_KFE_joint__)*s__q_RF_HAA_joint__;
	(*this)(4, 4) = (s__q_RF_HFE_joint__*c__q_RF_KFE_joint__ + s__q_RF_KFE_joint__*c__q_RF_HFE_joint__)*s__q_RF_HAA_joint__;
	(*this)(4, 5) = -c__q_RF_HAA_joint__;
	(*this)(5, 3) = -(-s__q_RF_HFE_joint__*s__q_RF_KFE_joint__ + c__q_RF_HFE_joint__*c__q_RF_KFE_joint__)*c__q_RF_HAA_joint__;
	(*this)(5, 4) = (s__q_RF_HFE_joint__*c__q_RF_KFE_joint__ + s__q_RF_KFE_joint__*c__q_RF_HFE_joint__)*c__q_RF_HAA_joint__;
	(*this)(5, 5) = s__q_RF_HAA_joint__;
	return *this;
}

template <typename TRAIT>
iit::pegasus2::tpl::ForceTransforms<TRAIT>::Type_fr_RF_foot_X_fr_torso::Type_fr_RF_foot_X_fr_torso(){ 
	(*this)(2, 0) = 0;
	(*this)(3, 0) = 0;
	(*this)(3, 1) = 0;
	(*this)(3, 2) = 0;
	(*this)(4, 0) = 0;
	(*this)(4, 1) = 0;
	(*this)(4, 2) = 0;
	(*this)(5, 0) = 0;
	(*this)(5, 1) = 0;
	(*this)(5, 2) = 0;
	(*this)(5, 3) = 0;
}

template <typename TRAIT>
const typename iit::pegasus2::tpl::ForceTransforms<TRAIT>::Type_fr_RF_foot_X_fr_torso& 
iit::pegasus2::tpl::ForceTransforms<TRAIT>::Type_fr_RF_foot_X_fr_torso::update(const JState& q){ 
	Scalar c__q_RF_KFE_joint__;
	Scalar c__q_RF_HAA_joint__;
	Scalar c__q_RF_HFE_joint__;
	Scalar s__q_RF_HFE_joint__;
	Scalar s__q_RF_HAA_joint__;
	Scalar s__q_RF_KFE_joint__;

	c__q_RF_KFE_joint__ = TRAIT::cos( q[RF_KFE_JOINT]);
	c__q_RF_HAA_joint__ = TRAIT::cos( q[RF_HAA_JOINT]);
	c__q_RF_HFE_joint__ = TRAIT::cos( q[RF_HFE_JOINT]);
	s__q_RF_HFE_joint__ = TRAIT::sin( q[RF_HFE_JOINT]);
	s__q_RF_HAA_joint__ = TRAIT::sin( q[RF_HAA_JOINT]);
	s__q_RF_KFE_joint__ = TRAIT::sin( q[RF_KFE_JOINT]);

	(*this)(0, 0) = s__q_RF_HFE_joint__*c__q_RF_KFE_joint__ + s__q_RF_KFE_joint__*c__q_RF_HFE_joint__;
	(*this)(0, 1) = -(-s__q_RF_HFE_joint__*s__q_RF_KFE_joint__ + c__q_RF_HFE_joint__*c__q_RF_KFE_joint__)*s__q_RF_HAA_joint__;
	(*this)(0, 2) = -(-s__q_RF_HFE_joint__*s__q_RF_KFE_joint__ + c__q_RF_HFE_joint__*c__q_RF_KFE_joint__)*c__q_RF_HAA_joint__;
	(*this)(0, 3) = (-s__q_RF_HFE_joint__*s__q_RF_KFE_joint__ + c__q_RF_HFE_joint__*c__q_RF_KFE_joint__)*(0.1185*c__q_RF_HAA_joint__ + 0.05);
	(*this)(0, 4) = 0.37*(-s__q_RF_HFE_joint__*s__q_RF_KFE_joint__ + c__q_RF_HFE_joint__*c__q_RF_KFE_joint__)*c__q_RF_HAA_joint__ + 0.05*(s__q_RF_HFE_joint__*c__q_RF_KFE_joint__ + s__q_RF_KFE_joint__*c__q_RF_HFE_joint__)*s__q_RF_HAA_joint__ - 0.38*s__q_RF_KFE_joint__*c__q_RF_HAA_joint__;
	(*this)(0, 5) = -0.37*(-s__q_RF_HFE_joint__*s__q_RF_KFE_joint__ + c__q_RF_HFE_joint__*c__q_RF_KFE_joint__)*s__q_RF_HAA_joint__ + 0.05*(s__q_RF_HFE_joint__*c__q_RF_KFE_joint__ + s__q_RF_KFE_joint__*c__q_RF_HFE_joint__)*c__q_RF_HAA_joint__ + 0.38*s__q_RF_HAA_joint__*s__q_RF_KFE_joint__ + 0.1185*s__q_RF_HFE_joint__*c__q_RF_KFE_joint__ + 0.1185*s__q_RF_KFE_joint__*c__q_RF_HFE_joint__;
	(*this)(1, 0) = -s__q_RF_HFE_joint__*s__q_RF_KFE_joint__ + c__q_RF_HFE_joint__*c__q_RF_KFE_joint__;
	(*this)(1, 1) = (s__q_RF_HFE_joint__*c__q_RF_KFE_joint__ + s__q_RF_KFE_joint__*c__q_RF_HFE_joint__)*s__q_RF_HAA_joint__;
	(*this)(1, 2) = (s__q_RF_HFE_joint__*c__q_RF_KFE_joint__ + s__q_RF_KFE_joint__*c__q_RF_HFE_joint__)*c__q_RF_HAA_joint__;
	(*this)(1, 3) = -(s__q_RF_HFE_joint__*c__q_RF_KFE_joint__ + s__q_RF_KFE_joint__*c__q_RF_HFE_joint__)*(0.1185*c__q_RF_HAA_joint__ + 0.05);
	(*this)(1, 4) = 0.05*(-s__q_RF_HFE_joint__*s__q_RF_KFE_joint__ + c__q_RF_HFE_joint__*c__q_RF_KFE_joint__)*s__q_RF_HAA_joint__ - 0.37*(s__q_RF_HFE_joint__*c__q_RF_KFE_joint__ + s__q_RF_KFE_joint__*c__q_RF_HFE_joint__)*c__q_RF_HAA_joint__ - (0.38*c__q_RF_KFE_joint__ - 0.3612)*c__q_RF_HAA_joint__;
	(*this)(1, 5) = 0.05*(-s__q_RF_HFE_joint__*s__q_RF_KFE_joint__ + c__q_RF_HFE_joint__*c__q_RF_KFE_joint__)*c__q_RF_HAA_joint__ + 0.37*(s__q_RF_HFE_joint__*c__q_RF_KFE_joint__ + s__q_RF_KFE_joint__*c__q_RF_HFE_joint__)*s__q_RF_HAA_joint__ + (0.38*c__q_RF_KFE_joint__ - 0.3612)*s__q_RF_HAA_joint__ - 0.1185*s__q_RF_HFE_joint__*s__q_RF_KFE_joint__ + 0.1185*c__q_RF_HFE_joint__*c__q_RF_KFE_joint__;
	(*this)(2, 1) = -c__q_RF_HAA_joint__;
	(*this)(2, 2) = s__q_RF_HAA_joint__;
	(*this)(2, 3) = -0.1185*s__q_RF_HAA_joint__ - 0.3612*s__q_RF_HFE_joint__*s__q_RF_KFE_joint__ + 0.3612*c__q_RF_HFE_joint__*c__q_RF_KFE_joint__ - 0.38*c__q_RF_HFE_joint__;
	(*this)(2, 4) = (0.3612*s__q_RF_HFE_joint__*c__q_RF_KFE_joint__ - 0.38*s__q_RF_HFE_joint__ + 0.3612*s__q_RF_KFE_joint__*c__q_RF_HFE_joint__ - 0.37)*s__q_RF_HAA_joint__;
	(*this)(2, 5) = (0.3612*s__q_RF_HFE_joint__*c__q_RF_KFE_joint__ - 0.38*s__q_RF_HFE_joint__ + 0.3612*s__q_RF_KFE_joint__*c__q_RF_HFE_joint__ - 0.37)*c__q_RF_HAA_joint__;
	(*this)(3, 3) = s__q_RF_HFE_joint__*c__q_RF_KFE_joint__ + s__q_RF_KFE_joint__*c__q_RF_HFE_joint__;
	(*this)(3, 4) = -(-s__q_RF_HFE_joint__*s__q_RF_KFE_joint__ + c__q_RF_HFE_joint__*c__q_RF_KFE_joint__)*s__q_RF_HAA_joint__;
	(*this)(3, 5) = -(-s__q_RF_HFE_joint__*s__q_RF_KFE_joint__ + c__q_RF_HFE_joint__*c__q_RF_KFE_joint__)*c__q_RF_HAA_joint__;
	(*this)(4, 3) = -s__q_RF_HFE_joint__*s__q_RF_KFE_joint__ + c__q_RF_HFE_joint__*c__q_RF_KFE_joint__;
	(*this)(4, 4) = (s__q_RF_HFE_joint__*c__q_RF_KFE_joint__ + s__q_RF_KFE_joint__*c__q_RF_HFE_joint__)*s__q_RF_HAA_joint__;
	(*this)(4, 5) = (s__q_RF_HFE_joint__*c__q_RF_KFE_joint__ + s__q_RF_KFE_joint__*c__q_RF_HFE_joint__)*c__q_RF_HAA_joint__;
	(*this)(5, 4) = -c__q_RF_HAA_joint__;
	(*this)(5, 5) = s__q_RF_HAA_joint__;
	return *this;
}

template <typename TRAIT>
iit::pegasus2::tpl::ForceTransforms<TRAIT>::Type_fr_torso_X_fr_RH_foot::Type_fr_torso_X_fr_RH_foot(){ 
	(*this)(0, 2) = 0;
	(*this)(3, 0) = 0;
	(*this)(3, 1) = 0;
	(*this)(3, 2) = 0;
	(*this)(3, 5) = 0;
	(*this)(4, 0) = 0;
	(*this)(4, 1) = 0;
	(*this)(4, 2) = 0;
	(*this)(5, 0) = 0;
	(*this)(5, 1) = 0;
	(*this)(5, 2) = 0;
}

template <typename TRAIT>
const typename iit::pegasus2::tpl::ForceTransforms<TRAIT>::Type_fr_torso_X_fr_RH_foot& 
iit::pegasus2::tpl::ForceTransforms<TRAIT>::Type_fr_torso_X_fr_RH_foot::update(const JState& q){ 
	Scalar c__q_RH_HFE_joint__;
	Scalar c__q_RH_HAA_joint__;
	Scalar c__q_RH_KFE_joint__;
	Scalar s__q_RH_KFE_joint__;
	Scalar s__q_RH_HAA_joint__;
	Scalar s__q_RH_HFE_joint__;

	c__q_RH_HFE_joint__ = TRAIT::cos( q[RH_HFE_JOINT]);
	c__q_RH_HAA_joint__ = TRAIT::cos( q[RH_HAA_JOINT]);
	c__q_RH_KFE_joint__ = TRAIT::cos( q[RH_KFE_JOINT]);
	s__q_RH_KFE_joint__ = TRAIT::sin( q[RH_KFE_JOINT]);
	s__q_RH_HAA_joint__ = TRAIT::sin( q[RH_HAA_JOINT]);
	s__q_RH_HFE_joint__ = TRAIT::sin( q[RH_HFE_JOINT]);

	(*this)(0, 0) = s__q_RH_HFE_joint__*c__q_RH_KFE_joint__ + s__q_RH_KFE_joint__*c__q_RH_HFE_joint__;
	(*this)(0, 1) = -s__q_RH_HFE_joint__*s__q_RH_KFE_joint__ + c__q_RH_HFE_joint__*c__q_RH_KFE_joint__;
	(*this)(0, 3) = (-s__q_RH_HFE_joint__*s__q_RH_KFE_joint__ + c__q_RH_HFE_joint__*c__q_RH_KFE_joint__)*(0.1185*c__q_RH_HAA_joint__ + 0.05);
	(*this)(0, 4) = -(s__q_RH_HFE_joint__*c__q_RH_KFE_joint__ + s__q_RH_KFE_joint__*c__q_RH_HFE_joint__)*(0.1185*c__q_RH_HAA_joint__ + 0.05);
	(*this)(0, 5) = -0.1185*s__q_RH_HAA_joint__ + 0.3612*s__q_RH_HFE_joint__*s__q_RH_KFE_joint__ - 0.3612*c__q_RH_HFE_joint__*c__q_RH_KFE_joint__ - 0.38*c__q_RH_HFE_joint__;
	(*this)(1, 0) = -(-s__q_RH_HFE_joint__*s__q_RH_KFE_joint__ + c__q_RH_HFE_joint__*c__q_RH_KFE_joint__)*s__q_RH_HAA_joint__;
	(*this)(1, 1) = (s__q_RH_HFE_joint__*c__q_RH_KFE_joint__ + s__q_RH_KFE_joint__*c__q_RH_HFE_joint__)*s__q_RH_HAA_joint__;
	(*this)(1, 2) = -c__q_RH_HAA_joint__;
	(*this)(1, 3) = -0.37*(-s__q_RH_HFE_joint__*s__q_RH_KFE_joint__ + c__q_RH_HFE_joint__*c__q_RH_KFE_joint__)*c__q_RH_HAA_joint__ + 0.05*(s__q_RH_HFE_joint__*c__q_RH_KFE_joint__ + s__q_RH_KFE_joint__*c__q_RH_HFE_joint__)*s__q_RH_HAA_joint__ - 0.38*s__q_RH_KFE_joint__*c__q_RH_HAA_joint__;
	(*this)(1, 4) = 0.16*(s__q_RH_HAA_joint__*s__q_RH_KFE_joint__ + c__q_RH_HAA_joint__*c__q_RH_KFE_joint__)*s__q_RH_HFE_joint__ + 0.16*(-s__q_RH_HAA_joint__*c__q_RH_KFE_joint__ + s__q_RH_KFE_joint__*c__q_RH_HAA_joint__)*c__q_RH_HFE_joint__ + 0.21*(-s__q_RH_HFE_joint__*s__q_RH_KFE_joint__ + c__q_RH_HFE_joint__*c__q_RH_KFE_joint__)*s__q_RH_HAA_joint__ + 0.21*(s__q_RH_HFE_joint__*c__q_RH_KFE_joint__ + s__q_RH_KFE_joint__*c__q_RH_HFE_joint__)*c__q_RH_HAA_joint__ - 0.38*c__q_RH_HAA_joint__*c__q_RH_KFE_joint__ - 0.3612*c__q_RH_HAA_joint__;
	(*this)(1, 5) = (-0.3612*s__q_RH_HFE_joint__*c__q_RH_KFE_joint__ - 0.38*s__q_RH_HFE_joint__ - 0.3612*s__q_RH_KFE_joint__*c__q_RH_HFE_joint__ + 0.37)*s__q_RH_HAA_joint__;
	(*this)(2, 0) = -(-s__q_RH_HFE_joint__*s__q_RH_KFE_joint__ + c__q_RH_HFE_joint__*c__q_RH_KFE_joint__)*c__q_RH_HAA_joint__;
	(*this)(2, 1) = (s__q_RH_HFE_joint__*c__q_RH_KFE_joint__ + s__q_RH_KFE_joint__*c__q_RH_HFE_joint__)*c__q_RH_HAA_joint__;
	(*this)(2, 2) = s__q_RH_HAA_joint__;
	(*this)(2, 3) = 0.37*(-s__q_RH_HFE_joint__*s__q_RH_KFE_joint__ + c__q_RH_HFE_joint__*c__q_RH_KFE_joint__)*s__q_RH_HAA_joint__ + 0.05*(s__q_RH_HFE_joint__*c__q_RH_KFE_joint__ + s__q_RH_KFE_joint__*c__q_RH_HFE_joint__)*c__q_RH_HAA_joint__ + 0.38*s__q_RH_HAA_joint__*s__q_RH_KFE_joint__ + 0.1185*s__q_RH_HFE_joint__*c__q_RH_KFE_joint__ + 0.1185*s__q_RH_KFE_joint__*c__q_RH_HFE_joint__;
	(*this)(2, 4) = 0.05*(-s__q_RH_HFE_joint__*s__q_RH_KFE_joint__ + c__q_RH_HFE_joint__*c__q_RH_KFE_joint__)*c__q_RH_HAA_joint__ - 0.37*(s__q_RH_HFE_joint__*c__q_RH_KFE_joint__ + s__q_RH_KFE_joint__*c__q_RH_HFE_joint__)*s__q_RH_HAA_joint__ + 0.38*s__q_RH_HAA_joint__*c__q_RH_KFE_joint__ + 0.3612*s__q_RH_HAA_joint__ - 0.1185*s__q_RH_HFE_joint__*s__q_RH_KFE_joint__ + 0.1185*c__q_RH_HFE_joint__*c__q_RH_KFE_joint__;
	(*this)(2, 5) = (-0.3612*s__q_RH_HFE_joint__*c__q_RH_KFE_joint__ - 0.38*s__q_RH_HFE_joint__ - 0.3612*s__q_RH_KFE_joint__*c__q_RH_HFE_joint__ + 0.37)*c__q_RH_HAA_joint__;
	(*this)(3, 3) = s__q_RH_HFE_joint__*c__q_RH_KFE_joint__ + s__q_RH_KFE_joint__*c__q_RH_HFE_joint__;
	(*this)(3, 4) = -s__q_RH_HFE_joint__*s__q_RH_KFE_joint__ + c__q_RH_HFE_joint__*c__q_RH_KFE_joint__;
	(*this)(4, 3) = -(-s__q_RH_HFE_joint__*s__q_RH_KFE_joint__ + c__q_RH_HFE_joint__*c__q_RH_KFE_joint__)*s__q_RH_HAA_joint__;
	(*this)(4, 4) = (s__q_RH_HFE_joint__*c__q_RH_KFE_joint__ + s__q_RH_KFE_joint__*c__q_RH_HFE_joint__)*s__q_RH_HAA_joint__;
	(*this)(4, 5) = -c__q_RH_HAA_joint__;
	(*this)(5, 3) = -(-s__q_RH_HFE_joint__*s__q_RH_KFE_joint__ + c__q_RH_HFE_joint__*c__q_RH_KFE_joint__)*c__q_RH_HAA_joint__;
	(*this)(5, 4) = (s__q_RH_HFE_joint__*c__q_RH_KFE_joint__ + s__q_RH_KFE_joint__*c__q_RH_HFE_joint__)*c__q_RH_HAA_joint__;
	(*this)(5, 5) = s__q_RH_HAA_joint__;
	return *this;
}

template <typename TRAIT>
iit::pegasus2::tpl::ForceTransforms<TRAIT>::Type_fr_RH_foot_X_fr_torso::Type_fr_RH_foot_X_fr_torso(){ 
	(*this)(2, 0) = 0;
	(*this)(3, 0) = 0;
	(*this)(3, 1) = 0;
	(*this)(3, 2) = 0;
	(*this)(4, 0) = 0;
	(*this)(4, 1) = 0;
	(*this)(4, 2) = 0;
	(*this)(5, 0) = 0;
	(*this)(5, 1) = 0;
	(*this)(5, 2) = 0;
	(*this)(5, 3) = 0;
}

template <typename TRAIT>
const typename iit::pegasus2::tpl::ForceTransforms<TRAIT>::Type_fr_RH_foot_X_fr_torso& 
iit::pegasus2::tpl::ForceTransforms<TRAIT>::Type_fr_RH_foot_X_fr_torso::update(const JState& q){ 
	Scalar c__q_RH_HFE_joint__;
	Scalar c__q_RH_HAA_joint__;
	Scalar c__q_RH_KFE_joint__;
	Scalar s__q_RH_KFE_joint__;
	Scalar s__q_RH_HAA_joint__;
	Scalar s__q_RH_HFE_joint__;

	c__q_RH_HFE_joint__ = TRAIT::cos( q[RH_HFE_JOINT]);
	c__q_RH_HAA_joint__ = TRAIT::cos( q[RH_HAA_JOINT]);
	c__q_RH_KFE_joint__ = TRAIT::cos( q[RH_KFE_JOINT]);
	s__q_RH_KFE_joint__ = TRAIT::sin( q[RH_KFE_JOINT]);
	s__q_RH_HAA_joint__ = TRAIT::sin( q[RH_HAA_JOINT]);
	s__q_RH_HFE_joint__ = TRAIT::sin( q[RH_HFE_JOINT]);

	(*this)(0, 0) = s__q_RH_HFE_joint__*c__q_RH_KFE_joint__ + s__q_RH_KFE_joint__*c__q_RH_HFE_joint__;
	(*this)(0, 1) = -(-s__q_RH_HFE_joint__*s__q_RH_KFE_joint__ + c__q_RH_HFE_joint__*c__q_RH_KFE_joint__)*s__q_RH_HAA_joint__;
	(*this)(0, 2) = -(-s__q_RH_HFE_joint__*s__q_RH_KFE_joint__ + c__q_RH_HFE_joint__*c__q_RH_KFE_joint__)*c__q_RH_HAA_joint__;
	(*this)(0, 3) = (-s__q_RH_HFE_joint__*s__q_RH_KFE_joint__ + c__q_RH_HFE_joint__*c__q_RH_KFE_joint__)*(0.1185*c__q_RH_HAA_joint__ + 0.05);
	(*this)(0, 4) = -0.37*(-s__q_RH_HFE_joint__*s__q_RH_KFE_joint__ + c__q_RH_HFE_joint__*c__q_RH_KFE_joint__)*c__q_RH_HAA_joint__ + 0.05*(s__q_RH_HFE_joint__*c__q_RH_KFE_joint__ + s__q_RH_KFE_joint__*c__q_RH_HFE_joint__)*s__q_RH_HAA_joint__ - 0.38*s__q_RH_KFE_joint__*c__q_RH_HAA_joint__;
	(*this)(0, 5) = 0.37*(-s__q_RH_HFE_joint__*s__q_RH_KFE_joint__ + c__q_RH_HFE_joint__*c__q_RH_KFE_joint__)*s__q_RH_HAA_joint__ + 0.05*(s__q_RH_HFE_joint__*c__q_RH_KFE_joint__ + s__q_RH_KFE_joint__*c__q_RH_HFE_joint__)*c__q_RH_HAA_joint__ + 0.38*s__q_RH_HAA_joint__*s__q_RH_KFE_joint__ + 0.1185*s__q_RH_HFE_joint__*c__q_RH_KFE_joint__ + 0.1185*s__q_RH_KFE_joint__*c__q_RH_HFE_joint__;
	(*this)(1, 0) = -s__q_RH_HFE_joint__*s__q_RH_KFE_joint__ + c__q_RH_HFE_joint__*c__q_RH_KFE_joint__;
	(*this)(1, 1) = (s__q_RH_HFE_joint__*c__q_RH_KFE_joint__ + s__q_RH_KFE_joint__*c__q_RH_HFE_joint__)*s__q_RH_HAA_joint__;
	(*this)(1, 2) = (s__q_RH_HFE_joint__*c__q_RH_KFE_joint__ + s__q_RH_KFE_joint__*c__q_RH_HFE_joint__)*c__q_RH_HAA_joint__;
	(*this)(1, 3) = -(s__q_RH_HFE_joint__*c__q_RH_KFE_joint__ + s__q_RH_KFE_joint__*c__q_RH_HFE_joint__)*(0.1185*c__q_RH_HAA_joint__ + 0.05);
	(*this)(1, 4) = 0.05*(-s__q_RH_HFE_joint__*s__q_RH_KFE_joint__ + c__q_RH_HFE_joint__*c__q_RH_KFE_joint__)*s__q_RH_HAA_joint__ + 0.37*(s__q_RH_HFE_joint__*c__q_RH_KFE_joint__ + s__q_RH_KFE_joint__*c__q_RH_HFE_joint__)*c__q_RH_HAA_joint__ - (0.38*c__q_RH_KFE_joint__ - 0.3612)*c__q_RH_HAA_joint__;
	(*this)(1, 5) = 0.05*(-s__q_RH_HFE_joint__*s__q_RH_KFE_joint__ + c__q_RH_HFE_joint__*c__q_RH_KFE_joint__)*c__q_RH_HAA_joint__ - 0.37*(s__q_RH_HFE_joint__*c__q_RH_KFE_joint__ + s__q_RH_KFE_joint__*c__q_RH_HFE_joint__)*s__q_RH_HAA_joint__ + (0.38*c__q_RH_KFE_joint__ - 0.3612)*s__q_RH_HAA_joint__ - 0.1185*s__q_RH_HFE_joint__*s__q_RH_KFE_joint__ + 0.1185*c__q_RH_HFE_joint__*c__q_RH_KFE_joint__;
	(*this)(2, 1) = -c__q_RH_HAA_joint__;
	(*this)(2, 2) = s__q_RH_HAA_joint__;
	(*this)(2, 3) = -0.1185*s__q_RH_HAA_joint__ - 0.3612*s__q_RH_HFE_joint__*s__q_RH_KFE_joint__ + 0.3612*c__q_RH_HFE_joint__*c__q_RH_KFE_joint__ - 0.38*c__q_RH_HFE_joint__;
	(*this)(2, 4) = (0.3612*s__q_RH_HFE_joint__*c__q_RH_KFE_joint__ - 0.38*s__q_RH_HFE_joint__ + 0.3612*s__q_RH_KFE_joint__*c__q_RH_HFE_joint__ + 0.37)*s__q_RH_HAA_joint__;
	(*this)(2, 5) = (0.3612*s__q_RH_HFE_joint__*c__q_RH_KFE_joint__ - 0.38*s__q_RH_HFE_joint__ + 0.3612*s__q_RH_KFE_joint__*c__q_RH_HFE_joint__ + 0.37)*c__q_RH_HAA_joint__;
	(*this)(3, 3) = s__q_RH_HFE_joint__*c__q_RH_KFE_joint__ + s__q_RH_KFE_joint__*c__q_RH_HFE_joint__;
	(*this)(3, 4) = -(-s__q_RH_HFE_joint__*s__q_RH_KFE_joint__ + c__q_RH_HFE_joint__*c__q_RH_KFE_joint__)*s__q_RH_HAA_joint__;
	(*this)(3, 5) = -(-s__q_RH_HFE_joint__*s__q_RH_KFE_joint__ + c__q_RH_HFE_joint__*c__q_RH_KFE_joint__)*c__q_RH_HAA_joint__;
	(*this)(4, 3) = -s__q_RH_HFE_joint__*s__q_RH_KFE_joint__ + c__q_RH_HFE_joint__*c__q_RH_KFE_joint__;
	(*this)(4, 4) = (s__q_RH_HFE_joint__*c__q_RH_KFE_joint__ + s__q_RH_KFE_joint__*c__q_RH_HFE_joint__)*s__q_RH_HAA_joint__;
	(*this)(4, 5) = (s__q_RH_HFE_joint__*c__q_RH_KFE_joint__ + s__q_RH_KFE_joint__*c__q_RH_HFE_joint__)*c__q_RH_HAA_joint__;
	(*this)(5, 4) = -c__q_RH_HAA_joint__;
	(*this)(5, 5) = s__q_RH_HAA_joint__;
	return *this;
}

template <typename TRAIT>
iit::pegasus2::tpl::ForceTransforms<TRAIT>::Type_fr_torso_X_fr_LH_foot::Type_fr_torso_X_fr_LH_foot(){ 
	(*this)(0, 2) = 0;
	(*this)(3, 0) = 0;
	(*this)(3, 1) = 0;
	(*this)(3, 2) = 0;
	(*this)(3, 5) = 0;
	(*this)(4, 0) = 0;
	(*this)(4, 1) = 0;
	(*this)(4, 2) = 0;
	(*this)(5, 0) = 0;
	(*this)(5, 1) = 0;
	(*this)(5, 2) = 0;
}

template <typename TRAIT>
const typename iit::pegasus2::tpl::ForceTransforms<TRAIT>::Type_fr_torso_X_fr_LH_foot& 
iit::pegasus2::tpl::ForceTransforms<TRAIT>::Type_fr_torso_X_fr_LH_foot::update(const JState& q){ 
	Scalar c__q_LH_KFE_joint__;
	Scalar c__q_LH_HAA_joint__;
	Scalar c__q_LH_HFE_joint__;
	Scalar s__q_LH_HFE_joint__;
	Scalar s__q_LH_HAA_joint__;
	Scalar s__q_LH_KFE_joint__;

	c__q_LH_KFE_joint__ = TRAIT::cos( q[LH_KFE_JOINT]);
	c__q_LH_HAA_joint__ = TRAIT::cos( q[LH_HAA_JOINT]);
	c__q_LH_HFE_joint__ = TRAIT::cos( q[LH_HFE_JOINT]);
	s__q_LH_HFE_joint__ = TRAIT::sin( q[LH_HFE_JOINT]);
	s__q_LH_HAA_joint__ = TRAIT::sin( q[LH_HAA_JOINT]);
	s__q_LH_KFE_joint__ = TRAIT::sin( q[LH_KFE_JOINT]);

	(*this)(0, 0) = s__q_LH_HFE_joint__*c__q_LH_KFE_joint__ + s__q_LH_KFE_joint__*c__q_LH_HFE_joint__;
	(*this)(0, 1) = -s__q_LH_HFE_joint__*s__q_LH_KFE_joint__ + c__q_LH_HFE_joint__*c__q_LH_KFE_joint__;
	(*this)(0, 3) = -(-s__q_LH_HFE_joint__*s__q_LH_KFE_joint__ + c__q_LH_HFE_joint__*c__q_LH_KFE_joint__)*(0.1185*c__q_LH_HAA_joint__ + 0.05);
	(*this)(0, 4) = (s__q_LH_HFE_joint__*c__q_LH_KFE_joint__ + s__q_LH_KFE_joint__*c__q_LH_HFE_joint__)*(0.1185*c__q_LH_HAA_joint__ + 0.05);
	(*this)(0, 5) = 0.1185*s__q_LH_HAA_joint__ + 0.3612*s__q_LH_HFE_joint__*s__q_LH_KFE_joint__ - 0.3612*c__q_LH_HFE_joint__*c__q_LH_KFE_joint__ - 0.38*c__q_LH_HFE_joint__;
	(*this)(1, 0) = -(-s__q_LH_HFE_joint__*s__q_LH_KFE_joint__ + c__q_LH_HFE_joint__*c__q_LH_KFE_joint__)*s__q_LH_HAA_joint__;
	(*this)(1, 1) = (s__q_LH_HFE_joint__*c__q_LH_KFE_joint__ + s__q_LH_KFE_joint__*c__q_LH_HFE_joint__)*s__q_LH_HAA_joint__;
	(*this)(1, 2) = -c__q_LH_HAA_joint__;
	(*this)(1, 3) = -0.21*(s__q_LH_HAA_joint__*s__q_LH_KFE_joint__ + c__q_LH_HAA_joint__*c__q_LH_KFE_joint__)*c__q_LH_HFE_joint__ + 0.21*(-s__q_LH_HAA_joint__*c__q_LH_KFE_joint__ + s__q_LH_KFE_joint__*c__q_LH_HAA_joint__)*s__q_LH_HFE_joint__ - 0.16*(-s__q_LH_HFE_joint__*s__q_LH_KFE_joint__ + c__q_LH_HFE_joint__*c__q_LH_KFE_joint__)*c__q_LH_HAA_joint__ + 0.16*(s__q_LH_HFE_joint__*c__q_LH_KFE_joint__ + s__q_LH_KFE_joint__*c__q_LH_HFE_joint__)*s__q_LH_HAA_joint__ - 0.38*s__q_LH_KFE_joint__*c__q_LH_HAA_joint__;
	(*this)(1, 4) = 0.21*(s__q_LH_HAA_joint__*s__q_LH_KFE_joint__ + c__q_LH_HAA_joint__*c__q_LH_KFE_joint__)*s__q_LH_HFE_joint__ + 0.21*(-s__q_LH_HAA_joint__*c__q_LH_KFE_joint__ + s__q_LH_KFE_joint__*c__q_LH_HAA_joint__)*c__q_LH_HFE_joint__ + 0.16*(-s__q_LH_HFE_joint__*s__q_LH_KFE_joint__ + c__q_LH_HFE_joint__*c__q_LH_KFE_joint__)*s__q_LH_HAA_joint__ + 0.16*(s__q_LH_HFE_joint__*c__q_LH_KFE_joint__ + s__q_LH_KFE_joint__*c__q_LH_HFE_joint__)*c__q_LH_HAA_joint__ - 0.38*c__q_LH_HAA_joint__*c__q_LH_KFE_joint__ - 0.3612*c__q_LH_HAA_joint__;
	(*this)(1, 5) = (-0.3612*s__q_LH_HFE_joint__*c__q_LH_KFE_joint__ - 0.38*s__q_LH_HFE_joint__ - 0.3612*s__q_LH_KFE_joint__*c__q_LH_HFE_joint__ + 0.37)*s__q_LH_HAA_joint__;
	(*this)(2, 0) = -(-s__q_LH_HFE_joint__*s__q_LH_KFE_joint__ + c__q_LH_HFE_joint__*c__q_LH_KFE_joint__)*c__q_LH_HAA_joint__;
	(*this)(2, 1) = (s__q_LH_HFE_joint__*c__q_LH_KFE_joint__ + s__q_LH_KFE_joint__*c__q_LH_HFE_joint__)*c__q_LH_HAA_joint__;
	(*this)(2, 2) = s__q_LH_HAA_joint__;
	(*this)(2, 3) = 0.37*(-s__q_LH_HFE_joint__*s__q_LH_KFE_joint__ + c__q_LH_HFE_joint__*c__q_LH_KFE_joint__)*s__q_LH_HAA_joint__ - 0.05*(s__q_LH_HFE_joint__*c__q_LH_KFE_joint__ + s__q_LH_KFE_joint__*c__q_LH_HFE_joint__)*c__q_LH_HAA_joint__ + 0.38*s__q_LH_HAA_joint__*s__q_LH_KFE_joint__ - 0.1185*s__q_LH_HFE_joint__*c__q_LH_KFE_joint__ - 0.1185*s__q_LH_KFE_joint__*c__q_LH_HFE_joint__;
	(*this)(2, 4) = -0.05*(-s__q_LH_HFE_joint__*s__q_LH_KFE_joint__ + c__q_LH_HFE_joint__*c__q_LH_KFE_joint__)*c__q_LH_HAA_joint__ - 0.37*(s__q_LH_HFE_joint__*c__q_LH_KFE_joint__ + s__q_LH_KFE_joint__*c__q_LH_HFE_joint__)*s__q_LH_HAA_joint__ + 0.38*s__q_LH_HAA_joint__*c__q_LH_KFE_joint__ + 0.3612*s__q_LH_HAA_joint__ + 0.1185*s__q_LH_HFE_joint__*s__q_LH_KFE_joint__ - 0.1185*c__q_LH_HFE_joint__*c__q_LH_KFE_joint__;
	(*this)(2, 5) = (-0.3612*s__q_LH_HFE_joint__*c__q_LH_KFE_joint__ - 0.38*s__q_LH_HFE_joint__ - 0.3612*s__q_LH_KFE_joint__*c__q_LH_HFE_joint__ + 0.37)*c__q_LH_HAA_joint__;
	(*this)(3, 3) = s__q_LH_HFE_joint__*c__q_LH_KFE_joint__ + s__q_LH_KFE_joint__*c__q_LH_HFE_joint__;
	(*this)(3, 4) = -s__q_LH_HFE_joint__*s__q_LH_KFE_joint__ + c__q_LH_HFE_joint__*c__q_LH_KFE_joint__;
	(*this)(4, 3) = -(-s__q_LH_HFE_joint__*s__q_LH_KFE_joint__ + c__q_LH_HFE_joint__*c__q_LH_KFE_joint__)*s__q_LH_HAA_joint__;
	(*this)(4, 4) = (s__q_LH_HFE_joint__*c__q_LH_KFE_joint__ + s__q_LH_KFE_joint__*c__q_LH_HFE_joint__)*s__q_LH_HAA_joint__;
	(*this)(4, 5) = -c__q_LH_HAA_joint__;
	(*this)(5, 3) = -(-s__q_LH_HFE_joint__*s__q_LH_KFE_joint__ + c__q_LH_HFE_joint__*c__q_LH_KFE_joint__)*c__q_LH_HAA_joint__;
	(*this)(5, 4) = (s__q_LH_HFE_joint__*c__q_LH_KFE_joint__ + s__q_LH_KFE_joint__*c__q_LH_HFE_joint__)*c__q_LH_HAA_joint__;
	(*this)(5, 5) = s__q_LH_HAA_joint__;
	return *this;
}

template <typename TRAIT>
iit::pegasus2::tpl::ForceTransforms<TRAIT>::Type_fr_LH_foot_X_fr_torso::Type_fr_LH_foot_X_fr_torso(){ 
	(*this)(2, 0) = 0;
	(*this)(3, 0) = 0;
	(*this)(3, 1) = 0;
	(*this)(3, 2) = 0;
	(*this)(4, 0) = 0;
	(*this)(4, 1) = 0;
	(*this)(4, 2) = 0;
	(*this)(5, 0) = 0;
	(*this)(5, 1) = 0;
	(*this)(5, 2) = 0;
	(*this)(5, 3) = 0;
}

template <typename TRAIT>
const typename iit::pegasus2::tpl::ForceTransforms<TRAIT>::Type_fr_LH_foot_X_fr_torso& 
iit::pegasus2::tpl::ForceTransforms<TRAIT>::Type_fr_LH_foot_X_fr_torso::update(const JState& q){ 
	Scalar c__q_LH_KFE_joint__;
	Scalar c__q_LH_HAA_joint__;
	Scalar c__q_LH_HFE_joint__;
	Scalar s__q_LH_HFE_joint__;
	Scalar s__q_LH_HAA_joint__;
	Scalar s__q_LH_KFE_joint__;

	c__q_LH_KFE_joint__ = TRAIT::cos( q[LH_KFE_JOINT]);
	c__q_LH_HAA_joint__ = TRAIT::cos( q[LH_HAA_JOINT]);
	c__q_LH_HFE_joint__ = TRAIT::cos( q[LH_HFE_JOINT]);
	s__q_LH_HFE_joint__ = TRAIT::sin( q[LH_HFE_JOINT]);
	s__q_LH_HAA_joint__ = TRAIT::sin( q[LH_HAA_JOINT]);
	s__q_LH_KFE_joint__ = TRAIT::sin( q[LH_KFE_JOINT]);

	(*this)(0, 0) = s__q_LH_HFE_joint__*c__q_LH_KFE_joint__ + s__q_LH_KFE_joint__*c__q_LH_HFE_joint__;
	(*this)(0, 1) = -(-s__q_LH_HFE_joint__*s__q_LH_KFE_joint__ + c__q_LH_HFE_joint__*c__q_LH_KFE_joint__)*s__q_LH_HAA_joint__;
	(*this)(0, 2) = -(-s__q_LH_HFE_joint__*s__q_LH_KFE_joint__ + c__q_LH_HFE_joint__*c__q_LH_KFE_joint__)*c__q_LH_HAA_joint__;
	(*this)(0, 3) = -(-s__q_LH_HFE_joint__*s__q_LH_KFE_joint__ + c__q_LH_HFE_joint__*c__q_LH_KFE_joint__)*(0.1185*c__q_LH_HAA_joint__ + 0.05);
	(*this)(0, 4) = -0.37*(-s__q_LH_HFE_joint__*s__q_LH_KFE_joint__ + c__q_LH_HFE_joint__*c__q_LH_KFE_joint__)*c__q_LH_HAA_joint__ - 0.05*(s__q_LH_HFE_joint__*c__q_LH_KFE_joint__ + s__q_LH_KFE_joint__*c__q_LH_HFE_joint__)*s__q_LH_HAA_joint__ - 0.38*s__q_LH_KFE_joint__*c__q_LH_HAA_joint__;
	(*this)(0, 5) = 0.37*(-s__q_LH_HFE_joint__*s__q_LH_KFE_joint__ + c__q_LH_HFE_joint__*c__q_LH_KFE_joint__)*s__q_LH_HAA_joint__ - 0.05*(s__q_LH_HFE_joint__*c__q_LH_KFE_joint__ + s__q_LH_KFE_joint__*c__q_LH_HFE_joint__)*c__q_LH_HAA_joint__ + 0.38*s__q_LH_HAA_joint__*s__q_LH_KFE_joint__ - 0.1185*s__q_LH_HFE_joint__*c__q_LH_KFE_joint__ - 0.1185*s__q_LH_KFE_joint__*c__q_LH_HFE_joint__;
	(*this)(1, 0) = -s__q_LH_HFE_joint__*s__q_LH_KFE_joint__ + c__q_LH_HFE_joint__*c__q_LH_KFE_joint__;
	(*this)(1, 1) = (s__q_LH_HFE_joint__*c__q_LH_KFE_joint__ + s__q_LH_KFE_joint__*c__q_LH_HFE_joint__)*s__q_LH_HAA_joint__;
	(*this)(1, 2) = (s__q_LH_HFE_joint__*c__q_LH_KFE_joint__ + s__q_LH_KFE_joint__*c__q_LH_HFE_joint__)*c__q_LH_HAA_joint__;
	(*this)(1, 3) = (s__q_LH_HFE_joint__*c__q_LH_KFE_joint__ + s__q_LH_KFE_joint__*c__q_LH_HFE_joint__)*(0.1185*c__q_LH_HAA_joint__ + 0.05);
	(*this)(1, 4) = -0.05*(-s__q_LH_HFE_joint__*s__q_LH_KFE_joint__ + c__q_LH_HFE_joint__*c__q_LH_KFE_joint__)*s__q_LH_HAA_joint__ + 0.37*(s__q_LH_HFE_joint__*c__q_LH_KFE_joint__ + s__q_LH_KFE_joint__*c__q_LH_HFE_joint__)*c__q_LH_HAA_joint__ - (0.38*c__q_LH_KFE_joint__ - 0.3612)*c__q_LH_HAA_joint__;
	(*this)(1, 5) = -0.05*(-s__q_LH_HFE_joint__*s__q_LH_KFE_joint__ + c__q_LH_HFE_joint__*c__q_LH_KFE_joint__)*c__q_LH_HAA_joint__ - 0.37*(s__q_LH_HFE_joint__*c__q_LH_KFE_joint__ + s__q_LH_KFE_joint__*c__q_LH_HFE_joint__)*s__q_LH_HAA_joint__ + (0.38*c__q_LH_KFE_joint__ - 0.3612)*s__q_LH_HAA_joint__ + 0.1185*s__q_LH_HFE_joint__*s__q_LH_KFE_joint__ - 0.1185*c__q_LH_HFE_joint__*c__q_LH_KFE_joint__;
	(*this)(2, 1) = -c__q_LH_HAA_joint__;
	(*this)(2, 2) = s__q_LH_HAA_joint__;
	(*this)(2, 3) = 0.1185*s__q_LH_HAA_joint__ - 0.3612*s__q_LH_HFE_joint__*s__q_LH_KFE_joint__ + 0.3612*c__q_LH_HFE_joint__*c__q_LH_KFE_joint__ - 0.38*c__q_LH_HFE_joint__;
	(*this)(2, 4) = (0.3612*s__q_LH_HFE_joint__*c__q_LH_KFE_joint__ - 0.38*s__q_LH_HFE_joint__ + 0.3612*s__q_LH_KFE_joint__*c__q_LH_HFE_joint__ + 0.37)*s__q_LH_HAA_joint__;
	(*this)(2, 5) = (0.3612*s__q_LH_HFE_joint__*c__q_LH_KFE_joint__ - 0.38*s__q_LH_HFE_joint__ + 0.3612*s__q_LH_KFE_joint__*c__q_LH_HFE_joint__ + 0.37)*c__q_LH_HAA_joint__;
	(*this)(3, 3) = s__q_LH_HFE_joint__*c__q_LH_KFE_joint__ + s__q_LH_KFE_joint__*c__q_LH_HFE_joint__;
	(*this)(3, 4) = -(-s__q_LH_HFE_joint__*s__q_LH_KFE_joint__ + c__q_LH_HFE_joint__*c__q_LH_KFE_joint__)*s__q_LH_HAA_joint__;
	(*this)(3, 5) = -(-s__q_LH_HFE_joint__*s__q_LH_KFE_joint__ + c__q_LH_HFE_joint__*c__q_LH_KFE_joint__)*c__q_LH_HAA_joint__;
	(*this)(4, 3) = -s__q_LH_HFE_joint__*s__q_LH_KFE_joint__ + c__q_LH_HFE_joint__*c__q_LH_KFE_joint__;
	(*this)(4, 4) = (s__q_LH_HFE_joint__*c__q_LH_KFE_joint__ + s__q_LH_KFE_joint__*c__q_LH_HFE_joint__)*s__q_LH_HAA_joint__;
	(*this)(4, 5) = (s__q_LH_HFE_joint__*c__q_LH_KFE_joint__ + s__q_LH_KFE_joint__*c__q_LH_HFE_joint__)*c__q_LH_HAA_joint__;
	(*this)(5, 4) = -c__q_LH_HAA_joint__;
	(*this)(5, 5) = s__q_LH_HAA_joint__;
	return *this;
}

template <typename TRAIT>
iit::pegasus2::tpl::ForceTransforms<TRAIT>::Type_fr_torso_X_fr_LF_foot::Type_fr_torso_X_fr_LF_foot(){ 
	(*this)(0, 2) = 0;
	(*this)(3, 0) = 0;
	(*this)(3, 1) = 0;
	(*this)(3, 2) = 0;
	(*this)(3, 5) = 0;
	(*this)(4, 0) = 0;
	(*this)(4, 1) = 0;
	(*this)(4, 2) = 0;
	(*this)(5, 0) = 0;
	(*this)(5, 1) = 0;
	(*this)(5, 2) = 0;
}

template <typename TRAIT>
const typename iit::pegasus2::tpl::ForceTransforms<TRAIT>::Type_fr_torso_X_fr_LF_foot& 
iit::pegasus2::tpl::ForceTransforms<TRAIT>::Type_fr_torso_X_fr_LF_foot::update(const JState& q){ 
	Scalar c__q_LF_KFE_joint__;
	Scalar c__q_LF_HFE_joint__;
	Scalar c__q_LF_HAA_joint__;
	Scalar s__q_LF_HAA_joint__;
	Scalar s__q_LF_KFE_joint__;
	Scalar s__q_LF_HFE_joint__;

	c__q_LF_KFE_joint__ = TRAIT::cos( q[LF_KFE_JOINT]);
	c__q_LF_HFE_joint__ = TRAIT::cos( q[LF_HFE_JOINT]);
	c__q_LF_HAA_joint__ = TRAIT::cos( q[LF_HAA_JOINT]);
	s__q_LF_HAA_joint__ = TRAIT::sin( q[LF_HAA_JOINT]);
	s__q_LF_KFE_joint__ = TRAIT::sin( q[LF_KFE_JOINT]);
	s__q_LF_HFE_joint__ = TRAIT::sin( q[LF_HFE_JOINT]);

	(*this)(0, 0) = s__q_LF_HFE_joint__*c__q_LF_KFE_joint__ + s__q_LF_KFE_joint__*c__q_LF_HFE_joint__;
	(*this)(0, 1) = -s__q_LF_HFE_joint__*s__q_LF_KFE_joint__ + c__q_LF_HFE_joint__*c__q_LF_KFE_joint__;
	(*this)(0, 3) = -(-s__q_LF_HFE_joint__*s__q_LF_KFE_joint__ + c__q_LF_HFE_joint__*c__q_LF_KFE_joint__)*(0.1185*c__q_LF_HAA_joint__ + 0.05);
	(*this)(0, 4) = (s__q_LF_HFE_joint__*c__q_LF_KFE_joint__ + s__q_LF_KFE_joint__*c__q_LF_HFE_joint__)*(0.1185*c__q_LF_HAA_joint__ + 0.05);
	(*this)(0, 5) = 0.1185*s__q_LF_HAA_joint__ + 0.3612*s__q_LF_HFE_joint__*s__q_LF_KFE_joint__ - 0.3612*c__q_LF_HFE_joint__*c__q_LF_KFE_joint__ - 0.38*c__q_LF_HFE_joint__;
	(*this)(1, 0) = -(-s__q_LF_HFE_joint__*s__q_LF_KFE_joint__ + c__q_LF_HFE_joint__*c__q_LF_KFE_joint__)*s__q_LF_HAA_joint__;
	(*this)(1, 1) = (s__q_LF_HFE_joint__*c__q_LF_KFE_joint__ + s__q_LF_KFE_joint__*c__q_LF_HFE_joint__)*s__q_LF_HAA_joint__;
	(*this)(1, 2) = -c__q_LF_HAA_joint__;
	(*this)(1, 3) = 0.16*(s__q_LF_HAA_joint__*s__q_LF_KFE_joint__ + c__q_LF_HAA_joint__*c__q_LF_KFE_joint__)*c__q_LF_HFE_joint__ - 0.16*(-s__q_LF_HAA_joint__*c__q_LF_KFE_joint__ + s__q_LF_KFE_joint__*c__q_LF_HAA_joint__)*s__q_LF_HFE_joint__ + 0.21*(-s__q_LF_HFE_joint__*s__q_LF_KFE_joint__ + c__q_LF_HFE_joint__*c__q_LF_KFE_joint__)*c__q_LF_HAA_joint__ - 0.21*(s__q_LF_HFE_joint__*c__q_LF_KFE_joint__ + s__q_LF_KFE_joint__*c__q_LF_HFE_joint__)*s__q_LF_HAA_joint__ - 0.38*s__q_LF_KFE_joint__*c__q_LF_HAA_joint__;
	(*this)(1, 4) = -0.16*(s__q_LF_HAA_joint__*s__q_LF_KFE_joint__ + c__q_LF_HAA_joint__*c__q_LF_KFE_joint__)*s__q_LF_HFE_joint__ - 0.16*(-s__q_LF_HAA_joint__*c__q_LF_KFE_joint__ + s__q_LF_KFE_joint__*c__q_LF_HAA_joint__)*c__q_LF_HFE_joint__ - 0.21*(-s__q_LF_HFE_joint__*s__q_LF_KFE_joint__ + c__q_LF_HFE_joint__*c__q_LF_KFE_joint__)*s__q_LF_HAA_joint__ - 0.21*(s__q_LF_HFE_joint__*c__q_LF_KFE_joint__ + s__q_LF_KFE_joint__*c__q_LF_HFE_joint__)*c__q_LF_HAA_joint__ - 0.38*c__q_LF_HAA_joint__*c__q_LF_KFE_joint__ - 0.3612*c__q_LF_HAA_joint__;
	(*this)(1, 5) = -(0.3612*s__q_LF_HFE_joint__*c__q_LF_KFE_joint__ + 0.38*s__q_LF_HFE_joint__ + 0.3612*s__q_LF_KFE_joint__*c__q_LF_HFE_joint__ + 0.37)*s__q_LF_HAA_joint__;
	(*this)(2, 0) = -(-s__q_LF_HFE_joint__*s__q_LF_KFE_joint__ + c__q_LF_HFE_joint__*c__q_LF_KFE_joint__)*c__q_LF_HAA_joint__;
	(*this)(2, 1) = (s__q_LF_HFE_joint__*c__q_LF_KFE_joint__ + s__q_LF_KFE_joint__*c__q_LF_HFE_joint__)*c__q_LF_HAA_joint__;
	(*this)(2, 2) = s__q_LF_HAA_joint__;
	(*this)(2, 3) = -0.37*(-s__q_LF_HFE_joint__*s__q_LF_KFE_joint__ + c__q_LF_HFE_joint__*c__q_LF_KFE_joint__)*s__q_LF_HAA_joint__ - 0.05*(s__q_LF_HFE_joint__*c__q_LF_KFE_joint__ + s__q_LF_KFE_joint__*c__q_LF_HFE_joint__)*c__q_LF_HAA_joint__ + 0.38*s__q_LF_HAA_joint__*s__q_LF_KFE_joint__ - 0.1185*s__q_LF_HFE_joint__*c__q_LF_KFE_joint__ - 0.1185*s__q_LF_KFE_joint__*c__q_LF_HFE_joint__;
	(*this)(2, 4) = -0.05*(-s__q_LF_HFE_joint__*s__q_LF_KFE_joint__ + c__q_LF_HFE_joint__*c__q_LF_KFE_joint__)*c__q_LF_HAA_joint__ + 0.37*(s__q_LF_HFE_joint__*c__q_LF_KFE_joint__ + s__q_LF_KFE_joint__*c__q_LF_HFE_joint__)*s__q_LF_HAA_joint__ + 0.38*s__q_LF_HAA_joint__*c__q_LF_KFE_joint__ + 0.3612*s__q_LF_HAA_joint__ + 0.1185*s__q_LF_HFE_joint__*s__q_LF_KFE_joint__ - 0.1185*c__q_LF_HFE_joint__*c__q_LF_KFE_joint__;
	(*this)(2, 5) = -(0.3612*s__q_LF_HFE_joint__*c__q_LF_KFE_joint__ + 0.38*s__q_LF_HFE_joint__ + 0.3612*s__q_LF_KFE_joint__*c__q_LF_HFE_joint__ + 0.37)*c__q_LF_HAA_joint__;
	(*this)(3, 3) = s__q_LF_HFE_joint__*c__q_LF_KFE_joint__ + s__q_LF_KFE_joint__*c__q_LF_HFE_joint__;
	(*this)(3, 4) = -s__q_LF_HFE_joint__*s__q_LF_KFE_joint__ + c__q_LF_HFE_joint__*c__q_LF_KFE_joint__;
	(*this)(4, 3) = -(-s__q_LF_HFE_joint__*s__q_LF_KFE_joint__ + c__q_LF_HFE_joint__*c__q_LF_KFE_joint__)*s__q_LF_HAA_joint__;
	(*this)(4, 4) = (s__q_LF_HFE_joint__*c__q_LF_KFE_joint__ + s__q_LF_KFE_joint__*c__q_LF_HFE_joint__)*s__q_LF_HAA_joint__;
	(*this)(4, 5) = -c__q_LF_HAA_joint__;
	(*this)(5, 3) = -(-s__q_LF_HFE_joint__*s__q_LF_KFE_joint__ + c__q_LF_HFE_joint__*c__q_LF_KFE_joint__)*c__q_LF_HAA_joint__;
	(*this)(5, 4) = (s__q_LF_HFE_joint__*c__q_LF_KFE_joint__ + s__q_LF_KFE_joint__*c__q_LF_HFE_joint__)*c__q_LF_HAA_joint__;
	(*this)(5, 5) = s__q_LF_HAA_joint__;
	return *this;
}

template <typename TRAIT>
iit::pegasus2::tpl::ForceTransforms<TRAIT>::Type_fr_LF_foot_X_fr_torso::Type_fr_LF_foot_X_fr_torso(){ 
	(*this)(2, 0) = 0;
	(*this)(3, 0) = 0;
	(*this)(3, 1) = 0;
	(*this)(3, 2) = 0;
	(*this)(4, 0) = 0;
	(*this)(4, 1) = 0;
	(*this)(4, 2) = 0;
	(*this)(5, 0) = 0;
	(*this)(5, 1) = 0;
	(*this)(5, 2) = 0;
	(*this)(5, 3) = 0;
}

template <typename TRAIT>
const typename iit::pegasus2::tpl::ForceTransforms<TRAIT>::Type_fr_LF_foot_X_fr_torso& 
iit::pegasus2::tpl::ForceTransforms<TRAIT>::Type_fr_LF_foot_X_fr_torso::update(const JState& q){ 
	Scalar c__q_LF_KFE_joint__;
	Scalar c__q_LF_HFE_joint__;
	Scalar c__q_LF_HAA_joint__;
	Scalar s__q_LF_HAA_joint__;
	Scalar s__q_LF_KFE_joint__;
	Scalar s__q_LF_HFE_joint__;

	c__q_LF_KFE_joint__ = TRAIT::cos( q[LF_KFE_JOINT]);
	c__q_LF_HFE_joint__ = TRAIT::cos( q[LF_HFE_JOINT]);
	c__q_LF_HAA_joint__ = TRAIT::cos( q[LF_HAA_JOINT]);
	s__q_LF_HAA_joint__ = TRAIT::sin( q[LF_HAA_JOINT]);
	s__q_LF_KFE_joint__ = TRAIT::sin( q[LF_KFE_JOINT]);
	s__q_LF_HFE_joint__ = TRAIT::sin( q[LF_HFE_JOINT]);

	(*this)(0, 0) = s__q_LF_HFE_joint__*c__q_LF_KFE_joint__ + s__q_LF_KFE_joint__*c__q_LF_HFE_joint__;
	(*this)(0, 1) = -(-s__q_LF_HFE_joint__*s__q_LF_KFE_joint__ + c__q_LF_HFE_joint__*c__q_LF_KFE_joint__)*s__q_LF_HAA_joint__;
	(*this)(0, 2) = -(-s__q_LF_HFE_joint__*s__q_LF_KFE_joint__ + c__q_LF_HFE_joint__*c__q_LF_KFE_joint__)*c__q_LF_HAA_joint__;
	(*this)(0, 3) = -(-s__q_LF_HFE_joint__*s__q_LF_KFE_joint__ + c__q_LF_HFE_joint__*c__q_LF_KFE_joint__)*(0.1185*c__q_LF_HAA_joint__ + 0.05);
	(*this)(0, 4) = 0.37*(-s__q_LF_HFE_joint__*s__q_LF_KFE_joint__ + c__q_LF_HFE_joint__*c__q_LF_KFE_joint__)*c__q_LF_HAA_joint__ - 0.05*(s__q_LF_HFE_joint__*c__q_LF_KFE_joint__ + s__q_LF_KFE_joint__*c__q_LF_HFE_joint__)*s__q_LF_HAA_joint__ - 0.38*s__q_LF_KFE_joint__*c__q_LF_HAA_joint__;
	(*this)(0, 5) = -0.37*(-s__q_LF_HFE_joint__*s__q_LF_KFE_joint__ + c__q_LF_HFE_joint__*c__q_LF_KFE_joint__)*s__q_LF_HAA_joint__ - 0.05*(s__q_LF_HFE_joint__*c__q_LF_KFE_joint__ + s__q_LF_KFE_joint__*c__q_LF_HFE_joint__)*c__q_LF_HAA_joint__ + 0.38*s__q_LF_HAA_joint__*s__q_LF_KFE_joint__ - 0.1185*s__q_LF_HFE_joint__*c__q_LF_KFE_joint__ - 0.1185*s__q_LF_KFE_joint__*c__q_LF_HFE_joint__;
	(*this)(1, 0) = -s__q_LF_HFE_joint__*s__q_LF_KFE_joint__ + c__q_LF_HFE_joint__*c__q_LF_KFE_joint__;
	(*this)(1, 1) = (s__q_LF_HFE_joint__*c__q_LF_KFE_joint__ + s__q_LF_KFE_joint__*c__q_LF_HFE_joint__)*s__q_LF_HAA_joint__;
	(*this)(1, 2) = (s__q_LF_HFE_joint__*c__q_LF_KFE_joint__ + s__q_LF_KFE_joint__*c__q_LF_HFE_joint__)*c__q_LF_HAA_joint__;
	(*this)(1, 3) = (s__q_LF_HFE_joint__*c__q_LF_KFE_joint__ + s__q_LF_KFE_joint__*c__q_LF_HFE_joint__)*(0.1185*c__q_LF_HAA_joint__ + 0.05);
	(*this)(1, 4) = -0.05*(-s__q_LF_HFE_joint__*s__q_LF_KFE_joint__ + c__q_LF_HFE_joint__*c__q_LF_KFE_joint__)*s__q_LF_HAA_joint__ - 0.37*(s__q_LF_HFE_joint__*c__q_LF_KFE_joint__ + s__q_LF_KFE_joint__*c__q_LF_HFE_joint__)*c__q_LF_HAA_joint__ - (0.38*c__q_LF_KFE_joint__ - 0.3612)*c__q_LF_HAA_joint__;
	(*this)(1, 5) = -0.05*(-s__q_LF_HFE_joint__*s__q_LF_KFE_joint__ + c__q_LF_HFE_joint__*c__q_LF_KFE_joint__)*c__q_LF_HAA_joint__ + 0.37*(s__q_LF_HFE_joint__*c__q_LF_KFE_joint__ + s__q_LF_KFE_joint__*c__q_LF_HFE_joint__)*s__q_LF_HAA_joint__ + (0.38*c__q_LF_KFE_joint__ - 0.3612)*s__q_LF_HAA_joint__ + 0.1185*s__q_LF_HFE_joint__*s__q_LF_KFE_joint__ - 0.1185*c__q_LF_HFE_joint__*c__q_LF_KFE_joint__;
	(*this)(2, 1) = -c__q_LF_HAA_joint__;
	(*this)(2, 2) = s__q_LF_HAA_joint__;
	(*this)(2, 3) = 0.1185*s__q_LF_HAA_joint__ - 0.3612*s__q_LF_HFE_joint__*s__q_LF_KFE_joint__ + 0.3612*c__q_LF_HFE_joint__*c__q_LF_KFE_joint__ - 0.38*c__q_LF_HFE_joint__;
	(*this)(2, 4) = (0.3612*s__q_LF_HFE_joint__*c__q_LF_KFE_joint__ - 0.38*s__q_LF_HFE_joint__ + 0.3612*s__q_LF_KFE_joint__*c__q_LF_HFE_joint__ - 0.37)*s__q_LF_HAA_joint__;
	(*this)(2, 5) = (0.3612*s__q_LF_HFE_joint__*c__q_LF_KFE_joint__ - 0.38*s__q_LF_HFE_joint__ + 0.3612*s__q_LF_KFE_joint__*c__q_LF_HFE_joint__ - 0.37)*c__q_LF_HAA_joint__;
	(*this)(3, 3) = s__q_LF_HFE_joint__*c__q_LF_KFE_joint__ + s__q_LF_KFE_joint__*c__q_LF_HFE_joint__;
	(*this)(3, 4) = -(-s__q_LF_HFE_joint__*s__q_LF_KFE_joint__ + c__q_LF_HFE_joint__*c__q_LF_KFE_joint__)*s__q_LF_HAA_joint__;
	(*this)(3, 5) = -(-s__q_LF_HFE_joint__*s__q_LF_KFE_joint__ + c__q_LF_HFE_joint__*c__q_LF_KFE_joint__)*c__q_LF_HAA_joint__;
	(*this)(4, 3) = -s__q_LF_HFE_joint__*s__q_LF_KFE_joint__ + c__q_LF_HFE_joint__*c__q_LF_KFE_joint__;
	(*this)(4, 4) = (s__q_LF_HFE_joint__*c__q_LF_KFE_joint__ + s__q_LF_KFE_joint__*c__q_LF_HFE_joint__)*s__q_LF_HAA_joint__;
	(*this)(4, 5) = (s__q_LF_HFE_joint__*c__q_LF_KFE_joint__ + s__q_LF_KFE_joint__*c__q_LF_HFE_joint__)*c__q_LF_HAA_joint__;
	(*this)(5, 4) = -c__q_LF_HAA_joint__;
	(*this)(5, 5) = s__q_LF_HAA_joint__;
	return *this;
}

