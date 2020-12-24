
template <typename TRAIT>
iit::pegasus2::tpl::Jacobians<TRAIT>::Jacobians()
	:
	fr_torso_J_fr_RF_foot(),
	fr_torso_J_fr_RH_foot(),
	fr_torso_J_fr_LH_foot(),
	fr_torso_J_fr_LF_foot()
{
	updateParameters();
}

template <typename TRAIT>
void iit::pegasus2::tpl::Jacobians<TRAIT>::updateParameters(){
}

template <typename TRAIT>
iit::pegasus2::tpl::Jacobians<TRAIT>::Type_fr_torso_J_fr_RF_foot::Type_fr_torso_J_fr_RF_foot()
{
	(*this)(0, 0) = -1;
	(*this)(0, 1) = 0;
	(*this)(0, 2) = 0;
	(*this)(1, 0) = 0;
	(*this)(2, 0) = 0;
	(*this)(3, 0) = 0;
}



template <typename TRAIT>
const typename iit::pegasus2::tpl::Jacobians<TRAIT>::Type_fr_torso_J_fr_RF_foot& iit::pegasus2::tpl::Jacobians<TRAIT>::Type_fr_torso_J_fr_RF_foot::update(const JState& q){ 
	Scalar c__q_RF_KFE_joint__;
	Scalar c__q_RF_HAA_joint__;
	Scalar c__q_RF_HFE_joint__;
	Scalar s__q_RF_HAA_joint__;
	Scalar s__q_RF_KFE_joint__;
	Scalar s__q_RF_HFE_joint__;

	c__q_RF_KFE_joint__ = TRAIT::cos( q[RF_KFE_JOINT]);
	c__q_RF_HAA_joint__ = TRAIT::cos( q[RF_HAA_JOINT]);
	c__q_RF_HFE_joint__ = TRAIT::cos( q[RF_HFE_JOINT]);
	s__q_RF_HAA_joint__ = TRAIT::sin( q[RF_HAA_JOINT]);
	s__q_RF_KFE_joint__ = TRAIT::sin( q[RF_KFE_JOINT]);
	s__q_RF_HFE_joint__ = TRAIT::sin( q[RF_HFE_JOINT]);

	(*this)(1, 1) = -c__q_RF_HAA_joint__;
	(*this)(1, 2) = -c__q_RF_HAA_joint__;
	(*this)(2, 1) = s__q_RF_HAA_joint__;
	(*this)(2, 2) = s__q_RF_HAA_joint__;
	(*this)(3, 1) = -0.3612*s__q_RF_HFE_joint__*s__q_RF_KFE_joint__ + 0.3612*c__q_RF_HFE_joint__*c__q_RF_KFE_joint__ + 0.38*c__q_RF_HFE_joint__;
	(*this)(3, 2) = -0.3612*s__q_RF_HFE_joint__*s__q_RF_KFE_joint__ + 0.3612*c__q_RF_HFE_joint__*c__q_RF_KFE_joint__;
	(*this)(4, 0) = -0.3612*(-s__q_RF_HFE_joint__*s__q_RF_KFE_joint__ + c__q_RF_HFE_joint__*c__q_RF_KFE_joint__)*c__q_RF_HAA_joint__ + 0.05*s__q_RF_HAA_joint__ - 0.38*c__q_RF_HAA_joint__*c__q_RF_HFE_joint__;
	(*this)(4, 1) = (0.3612*s__q_RF_HFE_joint__*c__q_RF_KFE_joint__ + 0.38*s__q_RF_HFE_joint__ + 0.3612*s__q_RF_KFE_joint__*c__q_RF_HFE_joint__)*s__q_RF_HAA_joint__;
	(*this)(4, 2) = 0.3612*(s__q_RF_HFE_joint__*c__q_RF_KFE_joint__ + s__q_RF_KFE_joint__*c__q_RF_HFE_joint__)*s__q_RF_HAA_joint__;
	(*this)(5, 0) = 0.3612*(-s__q_RF_HFE_joint__*s__q_RF_KFE_joint__ + c__q_RF_HFE_joint__*c__q_RF_KFE_joint__)*s__q_RF_HAA_joint__ + 0.38*s__q_RF_HAA_joint__*c__q_RF_HFE_joint__ + 0.05*c__q_RF_HAA_joint__;
	(*this)(5, 1) = (0.3612*s__q_RF_HFE_joint__*c__q_RF_KFE_joint__ + 0.38*s__q_RF_HFE_joint__ + 0.3612*s__q_RF_KFE_joint__*c__q_RF_HFE_joint__)*c__q_RF_HAA_joint__;
	(*this)(5, 2) = 0.3612*(s__q_RF_HFE_joint__*c__q_RF_KFE_joint__ + s__q_RF_KFE_joint__*c__q_RF_HFE_joint__)*c__q_RF_HAA_joint__;
	return *this;
}

template <typename TRAIT>
iit::pegasus2::tpl::Jacobians<TRAIT>::Type_fr_torso_J_fr_RH_foot::Type_fr_torso_J_fr_RH_foot()
{
	(*this)(0, 0) = -1;
	(*this)(0, 1) = 0;
	(*this)(0, 2) = 0;
	(*this)(1, 0) = 0;
	(*this)(2, 0) = 0;
	(*this)(3, 0) = 0;
}



template <typename TRAIT>
const typename iit::pegasus2::tpl::Jacobians<TRAIT>::Type_fr_torso_J_fr_RH_foot& iit::pegasus2::tpl::Jacobians<TRAIT>::Type_fr_torso_J_fr_RH_foot::update(const JState& q){ 
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

	(*this)(1, 1) = -c__q_RH_HAA_joint__;
	(*this)(1, 2) = -c__q_RH_HAA_joint__;
	(*this)(2, 1) = s__q_RH_HAA_joint__;
	(*this)(2, 2) = s__q_RH_HAA_joint__;
	(*this)(3, 1) = -0.3612*s__q_RH_HFE_joint__*s__q_RH_KFE_joint__ + 0.3612*c__q_RH_HFE_joint__*c__q_RH_KFE_joint__ + 0.38*c__q_RH_HFE_joint__;
	(*this)(3, 2) = -0.3612*s__q_RH_HFE_joint__*s__q_RH_KFE_joint__ + 0.3612*c__q_RH_HFE_joint__*c__q_RH_KFE_joint__;
	(*this)(4, 0) = -0.3612*(-s__q_RH_HFE_joint__*s__q_RH_KFE_joint__ + c__q_RH_HFE_joint__*c__q_RH_KFE_joint__)*c__q_RH_HAA_joint__ + 0.05*s__q_RH_HAA_joint__ - 0.38*c__q_RH_HAA_joint__*c__q_RH_HFE_joint__;
	(*this)(4, 1) = (0.3612*s__q_RH_HFE_joint__*c__q_RH_KFE_joint__ + 0.38*s__q_RH_HFE_joint__ + 0.3612*s__q_RH_KFE_joint__*c__q_RH_HFE_joint__)*s__q_RH_HAA_joint__;
	(*this)(4, 2) = 0.3612*(s__q_RH_HFE_joint__*c__q_RH_KFE_joint__ + s__q_RH_KFE_joint__*c__q_RH_HFE_joint__)*s__q_RH_HAA_joint__;
	(*this)(5, 0) = 0.3612*(-s__q_RH_HFE_joint__*s__q_RH_KFE_joint__ + c__q_RH_HFE_joint__*c__q_RH_KFE_joint__)*s__q_RH_HAA_joint__ + 0.38*s__q_RH_HAA_joint__*c__q_RH_HFE_joint__ + 0.05*c__q_RH_HAA_joint__;
	(*this)(5, 1) = (0.3612*s__q_RH_HFE_joint__*c__q_RH_KFE_joint__ + 0.38*s__q_RH_HFE_joint__ + 0.3612*s__q_RH_KFE_joint__*c__q_RH_HFE_joint__)*c__q_RH_HAA_joint__;
	(*this)(5, 2) = 0.3612*(s__q_RH_HFE_joint__*c__q_RH_KFE_joint__ + s__q_RH_KFE_joint__*c__q_RH_HFE_joint__)*c__q_RH_HAA_joint__;
	return *this;
}

template <typename TRAIT>
iit::pegasus2::tpl::Jacobians<TRAIT>::Type_fr_torso_J_fr_LH_foot::Type_fr_torso_J_fr_LH_foot()
{
	(*this)(0, 0) = -1;
	(*this)(0, 1) = 0;
	(*this)(0, 2) = 0;
	(*this)(1, 0) = 0;
	(*this)(2, 0) = 0;
	(*this)(3, 0) = 0;
}



template <typename TRAIT>
const typename iit::pegasus2::tpl::Jacobians<TRAIT>::Type_fr_torso_J_fr_LH_foot& iit::pegasus2::tpl::Jacobians<TRAIT>::Type_fr_torso_J_fr_LH_foot::update(const JState& q){ 
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

	(*this)(1, 1) = -c__q_LH_HAA_joint__;
	(*this)(1, 2) = -c__q_LH_HAA_joint__;
	(*this)(2, 1) = s__q_LH_HAA_joint__;
	(*this)(2, 2) = s__q_LH_HAA_joint__;
	(*this)(3, 1) = -0.3612*s__q_LH_HFE_joint__*s__q_LH_KFE_joint__ + 0.3612*c__q_LH_HFE_joint__*c__q_LH_KFE_joint__ + 0.38*c__q_LH_HFE_joint__;
	(*this)(3, 2) = -0.3612*s__q_LH_HFE_joint__*s__q_LH_KFE_joint__ + 0.3612*c__q_LH_HFE_joint__*c__q_LH_KFE_joint__;
	(*this)(4, 0) = -0.3612*(-s__q_LH_HFE_joint__*s__q_LH_KFE_joint__ + c__q_LH_HFE_joint__*c__q_LH_KFE_joint__)*c__q_LH_HAA_joint__ - 0.05*s__q_LH_HAA_joint__ - 0.38*c__q_LH_HAA_joint__*c__q_LH_HFE_joint__;
	(*this)(4, 1) = (0.3612*s__q_LH_HFE_joint__*c__q_LH_KFE_joint__ + 0.38*s__q_LH_HFE_joint__ + 0.3612*s__q_LH_KFE_joint__*c__q_LH_HFE_joint__)*s__q_LH_HAA_joint__;
	(*this)(4, 2) = 0.3612*(s__q_LH_HFE_joint__*c__q_LH_KFE_joint__ + s__q_LH_KFE_joint__*c__q_LH_HFE_joint__)*s__q_LH_HAA_joint__;
	(*this)(5, 0) = 0.3612*(-s__q_LH_HFE_joint__*s__q_LH_KFE_joint__ + c__q_LH_HFE_joint__*c__q_LH_KFE_joint__)*s__q_LH_HAA_joint__ + 0.38*s__q_LH_HAA_joint__*c__q_LH_HFE_joint__ - 0.05*c__q_LH_HAA_joint__;
	(*this)(5, 1) = (0.3612*s__q_LH_HFE_joint__*c__q_LH_KFE_joint__ + 0.38*s__q_LH_HFE_joint__ + 0.3612*s__q_LH_KFE_joint__*c__q_LH_HFE_joint__)*c__q_LH_HAA_joint__;
	(*this)(5, 2) = 0.3612*(s__q_LH_HFE_joint__*c__q_LH_KFE_joint__ + s__q_LH_KFE_joint__*c__q_LH_HFE_joint__)*c__q_LH_HAA_joint__;
	return *this;
}

template <typename TRAIT>
iit::pegasus2::tpl::Jacobians<TRAIT>::Type_fr_torso_J_fr_LF_foot::Type_fr_torso_J_fr_LF_foot()
{
	(*this)(0, 0) = -1;
	(*this)(0, 1) = 0;
	(*this)(0, 2) = 0;
	(*this)(1, 0) = 0;
	(*this)(2, 0) = 0;
	(*this)(3, 0) = 0;
}



template <typename TRAIT>
const typename iit::pegasus2::tpl::Jacobians<TRAIT>::Type_fr_torso_J_fr_LF_foot& iit::pegasus2::tpl::Jacobians<TRAIT>::Type_fr_torso_J_fr_LF_foot::update(const JState& q){ 
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

	(*this)(1, 1) = -c__q_LF_HAA_joint__;
	(*this)(1, 2) = -c__q_LF_HAA_joint__;
	(*this)(2, 1) = s__q_LF_HAA_joint__;
	(*this)(2, 2) = s__q_LF_HAA_joint__;
	(*this)(3, 1) = -0.3612*s__q_LF_HFE_joint__*s__q_LF_KFE_joint__ + 0.3612*c__q_LF_HFE_joint__*c__q_LF_KFE_joint__ + 0.38*c__q_LF_HFE_joint__;
	(*this)(3, 2) = -0.3612*s__q_LF_HFE_joint__*s__q_LF_KFE_joint__ + 0.3612*c__q_LF_HFE_joint__*c__q_LF_KFE_joint__;
	(*this)(4, 0) = -0.3612*(-s__q_LF_HFE_joint__*s__q_LF_KFE_joint__ + c__q_LF_HFE_joint__*c__q_LF_KFE_joint__)*c__q_LF_HAA_joint__ - 0.05*s__q_LF_HAA_joint__ - 0.38*c__q_LF_HAA_joint__*c__q_LF_HFE_joint__;
	(*this)(4, 1) = (0.3612*s__q_LF_HFE_joint__*c__q_LF_KFE_joint__ + 0.38*s__q_LF_HFE_joint__ + 0.3612*s__q_LF_KFE_joint__*c__q_LF_HFE_joint__)*s__q_LF_HAA_joint__;
	(*this)(4, 2) = 0.3612*(s__q_LF_HFE_joint__*c__q_LF_KFE_joint__ + s__q_LF_KFE_joint__*c__q_LF_HFE_joint__)*s__q_LF_HAA_joint__;
	(*this)(5, 0) = 0.3612*(-s__q_LF_HFE_joint__*s__q_LF_KFE_joint__ + c__q_LF_HFE_joint__*c__q_LF_KFE_joint__)*s__q_LF_HAA_joint__ + 0.38*s__q_LF_HAA_joint__*c__q_LF_HFE_joint__ - 0.05*c__q_LF_HAA_joint__;
	(*this)(5, 1) = (0.3612*s__q_LF_HFE_joint__*c__q_LF_KFE_joint__ + 0.38*s__q_LF_HFE_joint__ + 0.3612*s__q_LF_KFE_joint__*c__q_LF_HFE_joint__)*c__q_LF_HAA_joint__;
	(*this)(5, 2) = 0.3612*(s__q_LF_HFE_joint__*c__q_LF_KFE_joint__ + s__q_LF_KFE_joint__*c__q_LF_HFE_joint__)*c__q_LF_HAA_joint__;
	return *this;
}

