// Initialization of static-const data
template <typename TRAIT>
const typename iit::pegasus2::dyn::tpl::InverseDynamics<TRAIT>::ExtForces
iit::pegasus2::dyn::tpl::InverseDynamics<TRAIT>::zeroExtForces(Force::Zero());

template <typename TRAIT>
iit::pegasus2::dyn::tpl::InverseDynamics<TRAIT>::InverseDynamics(IProperties& inertia, MTransforms& transforms) :
    inertiaProps( & inertia ),
    xm( & transforms ),
    RF_hip_I(inertiaProps->getTensor_RF_hip() ),
    RF_upperleg_I(inertiaProps->getTensor_RF_upperleg() ),
    RF_lowerleg_I(inertiaProps->getTensor_RF_lowerleg() ),
    RH_hip_I(inertiaProps->getTensor_RH_hip() ),
    RH_upperleg_I(inertiaProps->getTensor_RH_upperleg() ),
    RH_lowerleg_I(inertiaProps->getTensor_RH_lowerleg() ),
    LH_hip_I(inertiaProps->getTensor_LH_hip() ),
    LH_upperleg_I(inertiaProps->getTensor_LH_upperleg() ),
    LH_lowerleg_I(inertiaProps->getTensor_LH_lowerleg() ),
    LF_hip_I(inertiaProps->getTensor_LF_hip() ),
    LF_upperleg_I(inertiaProps->getTensor_LF_upperleg() ),
    LF_lowerleg_I(inertiaProps->getTensor_LF_lowerleg() )
    ,
        torso_I( inertiaProps->getTensor_torso() ),
        RF_lowerleg_Ic(RF_lowerleg_I),
        RH_lowerleg_Ic(RH_lowerleg_I),
        LH_lowerleg_Ic(LH_lowerleg_I),
        LF_lowerleg_Ic(LF_lowerleg_I)
{
#ifndef EIGEN_NO_DEBUG
    std::cout << "Robot pegasus2, InverseDynamics<TRAIT>::InverseDynamics()" << std::endl;
    std::cout << "Compiled with Eigen debug active" << std::endl;
#endif
    RF_hip_v.setZero();
    RF_upperleg_v.setZero();
    RF_lowerleg_v.setZero();
    RH_hip_v.setZero();
    RH_upperleg_v.setZero();
    RH_lowerleg_v.setZero();
    LH_hip_v.setZero();
    LH_upperleg_v.setZero();
    LH_lowerleg_v.setZero();
    LF_hip_v.setZero();
    LF_upperleg_v.setZero();
    LF_lowerleg_v.setZero();

    vcross.setZero();
}

template <typename TRAIT>
void iit::pegasus2::dyn::tpl::InverseDynamics<TRAIT>::id(
    JointState& jForces, Acceleration& torso_a,
    const Acceleration& g, const Velocity& torso_v,
    const JointState& qd, const JointState& qdd,
    const ExtForces& fext)
{
    torso_Ic = torso_I;
    RF_hip_Ic = RF_hip_I;
    RF_upperleg_Ic = RF_upperleg_I;
    RH_hip_Ic = RH_hip_I;
    RH_upperleg_Ic = RH_upperleg_I;
    LH_hip_Ic = LH_hip_I;
    LH_upperleg_Ic = LH_upperleg_I;
    LF_hip_Ic = LF_hip_I;
    LF_upperleg_Ic = LF_upperleg_I;

    // First pass, link 'RF_hip'
    RF_hip_v = ((xm->fr_RF_hip_X_fr_torso) * torso_v);
    RF_hip_v(iit::rbd::AZ) += qd(RF_HAA_JOINT);
    
    iit::rbd::motionCrossProductMx<Scalar>(RF_hip_v, vcross);
    
    RF_hip_a = (vcross.col(iit::rbd::AZ) * qd(RF_HAA_JOINT));
    RF_hip_a(iit::rbd::AZ) += qdd(RF_HAA_JOINT);
    
    RF_hip_f = RF_hip_I * RF_hip_a + iit::rbd::vxIv(RF_hip_v, RF_hip_I);
    
    // First pass, link 'RF_upperleg'
    RF_upperleg_v = ((xm->fr_RF_upperleg_X_fr_RF_hip) * RF_hip_v);
    RF_upperleg_v(iit::rbd::AZ) += qd(RF_HFE_JOINT);
    
    iit::rbd::motionCrossProductMx<Scalar>(RF_upperleg_v, vcross);
    
    RF_upperleg_a = (xm->fr_RF_upperleg_X_fr_RF_hip) * RF_hip_a + vcross.col(iit::rbd::AZ) * qd(RF_HFE_JOINT);
    RF_upperleg_a(iit::rbd::AZ) += qdd(RF_HFE_JOINT);
    
    RF_upperleg_f = RF_upperleg_I * RF_upperleg_a + iit::rbd::vxIv(RF_upperleg_v, RF_upperleg_I);
    
    // First pass, link 'RF_lowerleg'
    RF_lowerleg_v = ((xm->fr_RF_lowerleg_X_fr_RF_upperleg) * RF_upperleg_v);
    RF_lowerleg_v(iit::rbd::AZ) += qd(RF_KFE_JOINT);
    
    iit::rbd::motionCrossProductMx<Scalar>(RF_lowerleg_v, vcross);
    
    RF_lowerleg_a = (xm->fr_RF_lowerleg_X_fr_RF_upperleg) * RF_upperleg_a + vcross.col(iit::rbd::AZ) * qd(RF_KFE_JOINT);
    RF_lowerleg_a(iit::rbd::AZ) += qdd(RF_KFE_JOINT);
    
    RF_lowerleg_f = RF_lowerleg_I * RF_lowerleg_a + iit::rbd::vxIv(RF_lowerleg_v, RF_lowerleg_I);
    
    // First pass, link 'RH_hip'
    RH_hip_v = ((xm->fr_RH_hip_X_fr_torso) * torso_v);
    RH_hip_v(iit::rbd::AZ) += qd(RH_HAA_JOINT);
    
    iit::rbd::motionCrossProductMx<Scalar>(RH_hip_v, vcross);
    
    RH_hip_a = (vcross.col(iit::rbd::AZ) * qd(RH_HAA_JOINT));
    RH_hip_a(iit::rbd::AZ) += qdd(RH_HAA_JOINT);
    
    RH_hip_f = RH_hip_I * RH_hip_a + iit::rbd::vxIv(RH_hip_v, RH_hip_I);
    
    // First pass, link 'RH_upperleg'
    RH_upperleg_v = ((xm->fr_RH_upperleg_X_fr_RH_hip) * RH_hip_v);
    RH_upperleg_v(iit::rbd::AZ) += qd(RH_HFE_JOINT);
    
    iit::rbd::motionCrossProductMx<Scalar>(RH_upperleg_v, vcross);
    
    RH_upperleg_a = (xm->fr_RH_upperleg_X_fr_RH_hip) * RH_hip_a + vcross.col(iit::rbd::AZ) * qd(RH_HFE_JOINT);
    RH_upperleg_a(iit::rbd::AZ) += qdd(RH_HFE_JOINT);
    
    RH_upperleg_f = RH_upperleg_I * RH_upperleg_a + iit::rbd::vxIv(RH_upperleg_v, RH_upperleg_I);
    
    // First pass, link 'RH_lowerleg'
    RH_lowerleg_v = ((xm->fr_RH_lowerleg_X_fr_RH_upperleg) * RH_upperleg_v);
    RH_lowerleg_v(iit::rbd::AZ) += qd(RH_KFE_JOINT);
    
    iit::rbd::motionCrossProductMx<Scalar>(RH_lowerleg_v, vcross);
    
    RH_lowerleg_a = (xm->fr_RH_lowerleg_X_fr_RH_upperleg) * RH_upperleg_a + vcross.col(iit::rbd::AZ) * qd(RH_KFE_JOINT);
    RH_lowerleg_a(iit::rbd::AZ) += qdd(RH_KFE_JOINT);
    
    RH_lowerleg_f = RH_lowerleg_I * RH_lowerleg_a + iit::rbd::vxIv(RH_lowerleg_v, RH_lowerleg_I);
    
    // First pass, link 'LH_hip'
    LH_hip_v = ((xm->fr_LH_hip_X_fr_torso) * torso_v);
    LH_hip_v(iit::rbd::AZ) += qd(LH_HAA_JOINT);
    
    iit::rbd::motionCrossProductMx<Scalar>(LH_hip_v, vcross);
    
    LH_hip_a = (vcross.col(iit::rbd::AZ) * qd(LH_HAA_JOINT));
    LH_hip_a(iit::rbd::AZ) += qdd(LH_HAA_JOINT);
    
    LH_hip_f = LH_hip_I * LH_hip_a + iit::rbd::vxIv(LH_hip_v, LH_hip_I);
    
    // First pass, link 'LH_upperleg'
    LH_upperleg_v = ((xm->fr_LH_upperleg_X_fr_LH_hip) * LH_hip_v);
    LH_upperleg_v(iit::rbd::AZ) += qd(LH_HFE_JOINT);
    
    iit::rbd::motionCrossProductMx<Scalar>(LH_upperleg_v, vcross);
    
    LH_upperleg_a = (xm->fr_LH_upperleg_X_fr_LH_hip) * LH_hip_a + vcross.col(iit::rbd::AZ) * qd(LH_HFE_JOINT);
    LH_upperleg_a(iit::rbd::AZ) += qdd(LH_HFE_JOINT);
    
    LH_upperleg_f = LH_upperleg_I * LH_upperleg_a + iit::rbd::vxIv(LH_upperleg_v, LH_upperleg_I);
    
    // First pass, link 'LH_lowerleg'
    LH_lowerleg_v = ((xm->fr_LH_lowerleg_X_fr_LH_upperleg) * LH_upperleg_v);
    LH_lowerleg_v(iit::rbd::AZ) += qd(LH_KFE_JOINT);
    
    iit::rbd::motionCrossProductMx<Scalar>(LH_lowerleg_v, vcross);
    
    LH_lowerleg_a = (xm->fr_LH_lowerleg_X_fr_LH_upperleg) * LH_upperleg_a + vcross.col(iit::rbd::AZ) * qd(LH_KFE_JOINT);
    LH_lowerleg_a(iit::rbd::AZ) += qdd(LH_KFE_JOINT);
    
    LH_lowerleg_f = LH_lowerleg_I * LH_lowerleg_a + iit::rbd::vxIv(LH_lowerleg_v, LH_lowerleg_I);
    
    // First pass, link 'LF_hip'
    LF_hip_v = ((xm->fr_LF_hip_X_fr_torso) * torso_v);
    LF_hip_v(iit::rbd::AZ) += qd(LF_HAA_JOINT);
    
    iit::rbd::motionCrossProductMx<Scalar>(LF_hip_v, vcross);
    
    LF_hip_a = (vcross.col(iit::rbd::AZ) * qd(LF_HAA_JOINT));
    LF_hip_a(iit::rbd::AZ) += qdd(LF_HAA_JOINT);
    
    LF_hip_f = LF_hip_I * LF_hip_a + iit::rbd::vxIv(LF_hip_v, LF_hip_I);
    
    // First pass, link 'LF_upperleg'
    LF_upperleg_v = ((xm->fr_LF_upperleg_X_fr_LF_hip) * LF_hip_v);
    LF_upperleg_v(iit::rbd::AZ) += qd(LF_HFE_JOINT);
    
    iit::rbd::motionCrossProductMx<Scalar>(LF_upperleg_v, vcross);
    
    LF_upperleg_a = (xm->fr_LF_upperleg_X_fr_LF_hip) * LF_hip_a + vcross.col(iit::rbd::AZ) * qd(LF_HFE_JOINT);
    LF_upperleg_a(iit::rbd::AZ) += qdd(LF_HFE_JOINT);
    
    LF_upperleg_f = LF_upperleg_I * LF_upperleg_a + iit::rbd::vxIv(LF_upperleg_v, LF_upperleg_I);
    
    // First pass, link 'LF_lowerleg'
    LF_lowerleg_v = ((xm->fr_LF_lowerleg_X_fr_LF_upperleg) * LF_upperleg_v);
    LF_lowerleg_v(iit::rbd::AZ) += qd(LF_KFE_JOINT);
    
    iit::rbd::motionCrossProductMx<Scalar>(LF_lowerleg_v, vcross);
    
    LF_lowerleg_a = (xm->fr_LF_lowerleg_X_fr_LF_upperleg) * LF_upperleg_a + vcross.col(iit::rbd::AZ) * qd(LF_KFE_JOINT);
    LF_lowerleg_a(iit::rbd::AZ) += qdd(LF_KFE_JOINT);
    
    LF_lowerleg_f = LF_lowerleg_I * LF_lowerleg_a + iit::rbd::vxIv(LF_lowerleg_v, LF_lowerleg_I);
    
    // The force exerted on the floating base by the links
    torso_f = iit::rbd::vxIv(torso_v, torso_I);
    

    // Add the external forces:
    torso_f -= fext[TORSO];
    RF_hip_f -= fext[RF_HIP];
    RF_upperleg_f -= fext[RF_UPPERLEG];
    RF_lowerleg_f -= fext[RF_LOWERLEG];
    RH_hip_f -= fext[RH_HIP];
    RH_upperleg_f -= fext[RH_UPPERLEG];
    RH_lowerleg_f -= fext[RH_LOWERLEG];
    LH_hip_f -= fext[LH_HIP];
    LH_upperleg_f -= fext[LH_UPPERLEG];
    LH_lowerleg_f -= fext[LH_LOWERLEG];
    LF_hip_f -= fext[LF_HIP];
    LF_upperleg_f -= fext[LF_UPPERLEG];
    LF_lowerleg_f -= fext[LF_LOWERLEG];

    LF_upperleg_Ic = LF_upperleg_Ic + (xm->fr_LF_lowerleg_X_fr_LF_upperleg).transpose() * LF_lowerleg_Ic * (xm->fr_LF_lowerleg_X_fr_LF_upperleg);
    LF_upperleg_f = LF_upperleg_f + (xm->fr_LF_lowerleg_X_fr_LF_upperleg).transpose() * LF_lowerleg_f;
    
    LF_hip_Ic = LF_hip_Ic + (xm->fr_LF_upperleg_X_fr_LF_hip).transpose() * LF_upperleg_Ic * (xm->fr_LF_upperleg_X_fr_LF_hip);
    LF_hip_f = LF_hip_f + (xm->fr_LF_upperleg_X_fr_LF_hip).transpose() * LF_upperleg_f;
    
    torso_Ic = torso_Ic + (xm->fr_LF_hip_X_fr_torso).transpose() * LF_hip_Ic * (xm->fr_LF_hip_X_fr_torso);
    torso_f = torso_f + (xm->fr_LF_hip_X_fr_torso).transpose() * LF_hip_f;
    
    LH_upperleg_Ic = LH_upperleg_Ic + (xm->fr_LH_lowerleg_X_fr_LH_upperleg).transpose() * LH_lowerleg_Ic * (xm->fr_LH_lowerleg_X_fr_LH_upperleg);
    LH_upperleg_f = LH_upperleg_f + (xm->fr_LH_lowerleg_X_fr_LH_upperleg).transpose() * LH_lowerleg_f;
    
    LH_hip_Ic = LH_hip_Ic + (xm->fr_LH_upperleg_X_fr_LH_hip).transpose() * LH_upperleg_Ic * (xm->fr_LH_upperleg_X_fr_LH_hip);
    LH_hip_f = LH_hip_f + (xm->fr_LH_upperleg_X_fr_LH_hip).transpose() * LH_upperleg_f;
    
    torso_Ic = torso_Ic + (xm->fr_LH_hip_X_fr_torso).transpose() * LH_hip_Ic * (xm->fr_LH_hip_X_fr_torso);
    torso_f = torso_f + (xm->fr_LH_hip_X_fr_torso).transpose() * LH_hip_f;
    
    RH_upperleg_Ic = RH_upperleg_Ic + (xm->fr_RH_lowerleg_X_fr_RH_upperleg).transpose() * RH_lowerleg_Ic * (xm->fr_RH_lowerleg_X_fr_RH_upperleg);
    RH_upperleg_f = RH_upperleg_f + (xm->fr_RH_lowerleg_X_fr_RH_upperleg).transpose() * RH_lowerleg_f;
    
    RH_hip_Ic = RH_hip_Ic + (xm->fr_RH_upperleg_X_fr_RH_hip).transpose() * RH_upperleg_Ic * (xm->fr_RH_upperleg_X_fr_RH_hip);
    RH_hip_f = RH_hip_f + (xm->fr_RH_upperleg_X_fr_RH_hip).transpose() * RH_upperleg_f;
    
    torso_Ic = torso_Ic + (xm->fr_RH_hip_X_fr_torso).transpose() * RH_hip_Ic * (xm->fr_RH_hip_X_fr_torso);
    torso_f = torso_f + (xm->fr_RH_hip_X_fr_torso).transpose() * RH_hip_f;
    
    RF_upperleg_Ic = RF_upperleg_Ic + (xm->fr_RF_lowerleg_X_fr_RF_upperleg).transpose() * RF_lowerleg_Ic * (xm->fr_RF_lowerleg_X_fr_RF_upperleg);
    RF_upperleg_f = RF_upperleg_f + (xm->fr_RF_lowerleg_X_fr_RF_upperleg).transpose() * RF_lowerleg_f;
    
    RF_hip_Ic = RF_hip_Ic + (xm->fr_RF_upperleg_X_fr_RF_hip).transpose() * RF_upperleg_Ic * (xm->fr_RF_upperleg_X_fr_RF_hip);
    RF_hip_f = RF_hip_f + (xm->fr_RF_upperleg_X_fr_RF_hip).transpose() * RF_upperleg_f;
    
    torso_Ic = torso_Ic + (xm->fr_RF_hip_X_fr_torso).transpose() * RF_hip_Ic * (xm->fr_RF_hip_X_fr_torso);
    torso_f = torso_f + (xm->fr_RF_hip_X_fr_torso).transpose() * RF_hip_f;
    

    // The base acceleration due to the force due to the movement of the links
    torso_a = - torso_Ic.inverse() * torso_f;
    
    RF_hip_a = xm->fr_RF_hip_X_fr_torso * torso_a;
    jForces(RF_HAA_JOINT) = (RF_hip_Ic.row(iit::rbd::AZ) * RF_hip_a + RF_hip_f(iit::rbd::AZ));
    
    RF_upperleg_a = xm->fr_RF_upperleg_X_fr_RF_hip * RF_hip_a;
    jForces(RF_HFE_JOINT) = (RF_upperleg_Ic.row(iit::rbd::AZ) * RF_upperleg_a + RF_upperleg_f(iit::rbd::AZ));
    
    RF_lowerleg_a = xm->fr_RF_lowerleg_X_fr_RF_upperleg * RF_upperleg_a;
    jForces(RF_KFE_JOINT) = (RF_lowerleg_Ic.row(iit::rbd::AZ) * RF_lowerleg_a + RF_lowerleg_f(iit::rbd::AZ));
    
    RH_hip_a = xm->fr_RH_hip_X_fr_torso * torso_a;
    jForces(RH_HAA_JOINT) = (RH_hip_Ic.row(iit::rbd::AZ) * RH_hip_a + RH_hip_f(iit::rbd::AZ));
    
    RH_upperleg_a = xm->fr_RH_upperleg_X_fr_RH_hip * RH_hip_a;
    jForces(RH_HFE_JOINT) = (RH_upperleg_Ic.row(iit::rbd::AZ) * RH_upperleg_a + RH_upperleg_f(iit::rbd::AZ));
    
    RH_lowerleg_a = xm->fr_RH_lowerleg_X_fr_RH_upperleg * RH_upperleg_a;
    jForces(RH_KFE_JOINT) = (RH_lowerleg_Ic.row(iit::rbd::AZ) * RH_lowerleg_a + RH_lowerleg_f(iit::rbd::AZ));
    
    LH_hip_a = xm->fr_LH_hip_X_fr_torso * torso_a;
    jForces(LH_HAA_JOINT) = (LH_hip_Ic.row(iit::rbd::AZ) * LH_hip_a + LH_hip_f(iit::rbd::AZ));
    
    LH_upperleg_a = xm->fr_LH_upperleg_X_fr_LH_hip * LH_hip_a;
    jForces(LH_HFE_JOINT) = (LH_upperleg_Ic.row(iit::rbd::AZ) * LH_upperleg_a + LH_upperleg_f(iit::rbd::AZ));
    
    LH_lowerleg_a = xm->fr_LH_lowerleg_X_fr_LH_upperleg * LH_upperleg_a;
    jForces(LH_KFE_JOINT) = (LH_lowerleg_Ic.row(iit::rbd::AZ) * LH_lowerleg_a + LH_lowerleg_f(iit::rbd::AZ));
    
    LF_hip_a = xm->fr_LF_hip_X_fr_torso * torso_a;
    jForces(LF_HAA_JOINT) = (LF_hip_Ic.row(iit::rbd::AZ) * LF_hip_a + LF_hip_f(iit::rbd::AZ));
    
    LF_upperleg_a = xm->fr_LF_upperleg_X_fr_LF_hip * LF_hip_a;
    jForces(LF_HFE_JOINT) = (LF_upperleg_Ic.row(iit::rbd::AZ) * LF_upperleg_a + LF_upperleg_f(iit::rbd::AZ));
    
    LF_lowerleg_a = xm->fr_LF_lowerleg_X_fr_LF_upperleg * LF_upperleg_a;
    jForces(LF_KFE_JOINT) = (LF_lowerleg_Ic.row(iit::rbd::AZ) * LF_lowerleg_a + LF_lowerleg_f(iit::rbd::AZ));
    

    torso_a += g;
}

template <typename TRAIT>
void iit::pegasus2::dyn::tpl::InverseDynamics<TRAIT>::G_terms_fully_actuated(
    Force& baseWrench, JointState& jForces,
    const Acceleration& g)
{
    const Acceleration& torso_a = -g;

    // Link 'RF_hip'
    RF_hip_a = (xm->fr_RF_hip_X_fr_torso) * torso_a;
    RF_hip_f = RF_hip_I * RF_hip_a;
    // Link 'RF_upperleg'
    RF_upperleg_a = (xm->fr_RF_upperleg_X_fr_RF_hip) * RF_hip_a;
    RF_upperleg_f = RF_upperleg_I * RF_upperleg_a;
    // Link 'RF_lowerleg'
    RF_lowerleg_a = (xm->fr_RF_lowerleg_X_fr_RF_upperleg) * RF_upperleg_a;
    RF_lowerleg_f = RF_lowerleg_I * RF_lowerleg_a;
    // Link 'RH_hip'
    RH_hip_a = (xm->fr_RH_hip_X_fr_torso) * torso_a;
    RH_hip_f = RH_hip_I * RH_hip_a;
    // Link 'RH_upperleg'
    RH_upperleg_a = (xm->fr_RH_upperleg_X_fr_RH_hip) * RH_hip_a;
    RH_upperleg_f = RH_upperleg_I * RH_upperleg_a;
    // Link 'RH_lowerleg'
    RH_lowerleg_a = (xm->fr_RH_lowerleg_X_fr_RH_upperleg) * RH_upperleg_a;
    RH_lowerleg_f = RH_lowerleg_I * RH_lowerleg_a;
    // Link 'LH_hip'
    LH_hip_a = (xm->fr_LH_hip_X_fr_torso) * torso_a;
    LH_hip_f = LH_hip_I * LH_hip_a;
    // Link 'LH_upperleg'
    LH_upperleg_a = (xm->fr_LH_upperleg_X_fr_LH_hip) * LH_hip_a;
    LH_upperleg_f = LH_upperleg_I * LH_upperleg_a;
    // Link 'LH_lowerleg'
    LH_lowerleg_a = (xm->fr_LH_lowerleg_X_fr_LH_upperleg) * LH_upperleg_a;
    LH_lowerleg_f = LH_lowerleg_I * LH_lowerleg_a;
    // Link 'LF_hip'
    LF_hip_a = (xm->fr_LF_hip_X_fr_torso) * torso_a;
    LF_hip_f = LF_hip_I * LF_hip_a;
    // Link 'LF_upperleg'
    LF_upperleg_a = (xm->fr_LF_upperleg_X_fr_LF_hip) * LF_hip_a;
    LF_upperleg_f = LF_upperleg_I * LF_upperleg_a;
    // Link 'LF_lowerleg'
    LF_lowerleg_a = (xm->fr_LF_lowerleg_X_fr_LF_upperleg) * LF_upperleg_a;
    LF_lowerleg_f = LF_lowerleg_I * LF_lowerleg_a;

    torso_f = torso_I * torso_a;

    secondPass_fullyActuated(jForces);

    baseWrench = torso_f;
}

template <typename TRAIT>
void iit::pegasus2::dyn::tpl::InverseDynamics<TRAIT>::C_terms_fully_actuated(
    Force& baseWrench, JointState& jForces,
    const Velocity& torso_v, const JointState& qd)
{
    // Link 'RF_hip'
    RF_hip_v = ((xm->fr_RF_hip_X_fr_torso) * torso_v);
    RF_hip_v(iit::rbd::AZ) += qd(RF_HAA_JOINT);
    iit::rbd::motionCrossProductMx<Scalar>(RF_hip_v, vcross);
    RF_hip_a = (vcross.col(iit::rbd::AZ) * qd(RF_HAA_JOINT));
    RF_hip_f = RF_hip_I * RF_hip_a + iit::rbd::vxIv(RF_hip_v, RF_hip_I);
    
    // Link 'RF_upperleg'
    RF_upperleg_v = ((xm->fr_RF_upperleg_X_fr_RF_hip) * RF_hip_v);
    RF_upperleg_v(iit::rbd::AZ) += qd(RF_HFE_JOINT);
    iit::rbd::motionCrossProductMx<Scalar>(RF_upperleg_v, vcross);
    RF_upperleg_a = (xm->fr_RF_upperleg_X_fr_RF_hip) * RF_hip_a + vcross.col(iit::rbd::AZ) * qd(RF_HFE_JOINT);
    RF_upperleg_f = RF_upperleg_I * RF_upperleg_a + iit::rbd::vxIv(RF_upperleg_v, RF_upperleg_I);
    
    // Link 'RF_lowerleg'
    RF_lowerleg_v = ((xm->fr_RF_lowerleg_X_fr_RF_upperleg) * RF_upperleg_v);
    RF_lowerleg_v(iit::rbd::AZ) += qd(RF_KFE_JOINT);
    iit::rbd::motionCrossProductMx<Scalar>(RF_lowerleg_v, vcross);
    RF_lowerleg_a = (xm->fr_RF_lowerleg_X_fr_RF_upperleg) * RF_upperleg_a + vcross.col(iit::rbd::AZ) * qd(RF_KFE_JOINT);
    RF_lowerleg_f = RF_lowerleg_I * RF_lowerleg_a + iit::rbd::vxIv(RF_lowerleg_v, RF_lowerleg_I);
    
    // Link 'RH_hip'
    RH_hip_v = ((xm->fr_RH_hip_X_fr_torso) * torso_v);
    RH_hip_v(iit::rbd::AZ) += qd(RH_HAA_JOINT);
    iit::rbd::motionCrossProductMx<Scalar>(RH_hip_v, vcross);
    RH_hip_a = (vcross.col(iit::rbd::AZ) * qd(RH_HAA_JOINT));
    RH_hip_f = RH_hip_I * RH_hip_a + iit::rbd::vxIv(RH_hip_v, RH_hip_I);
    
    // Link 'RH_upperleg'
    RH_upperleg_v = ((xm->fr_RH_upperleg_X_fr_RH_hip) * RH_hip_v);
    RH_upperleg_v(iit::rbd::AZ) += qd(RH_HFE_JOINT);
    iit::rbd::motionCrossProductMx<Scalar>(RH_upperleg_v, vcross);
    RH_upperleg_a = (xm->fr_RH_upperleg_X_fr_RH_hip) * RH_hip_a + vcross.col(iit::rbd::AZ) * qd(RH_HFE_JOINT);
    RH_upperleg_f = RH_upperleg_I * RH_upperleg_a + iit::rbd::vxIv(RH_upperleg_v, RH_upperleg_I);
    
    // Link 'RH_lowerleg'
    RH_lowerleg_v = ((xm->fr_RH_lowerleg_X_fr_RH_upperleg) * RH_upperleg_v);
    RH_lowerleg_v(iit::rbd::AZ) += qd(RH_KFE_JOINT);
    iit::rbd::motionCrossProductMx<Scalar>(RH_lowerleg_v, vcross);
    RH_lowerleg_a = (xm->fr_RH_lowerleg_X_fr_RH_upperleg) * RH_upperleg_a + vcross.col(iit::rbd::AZ) * qd(RH_KFE_JOINT);
    RH_lowerleg_f = RH_lowerleg_I * RH_lowerleg_a + iit::rbd::vxIv(RH_lowerleg_v, RH_lowerleg_I);
    
    // Link 'LH_hip'
    LH_hip_v = ((xm->fr_LH_hip_X_fr_torso) * torso_v);
    LH_hip_v(iit::rbd::AZ) += qd(LH_HAA_JOINT);
    iit::rbd::motionCrossProductMx<Scalar>(LH_hip_v, vcross);
    LH_hip_a = (vcross.col(iit::rbd::AZ) * qd(LH_HAA_JOINT));
    LH_hip_f = LH_hip_I * LH_hip_a + iit::rbd::vxIv(LH_hip_v, LH_hip_I);
    
    // Link 'LH_upperleg'
    LH_upperleg_v = ((xm->fr_LH_upperleg_X_fr_LH_hip) * LH_hip_v);
    LH_upperleg_v(iit::rbd::AZ) += qd(LH_HFE_JOINT);
    iit::rbd::motionCrossProductMx<Scalar>(LH_upperleg_v, vcross);
    LH_upperleg_a = (xm->fr_LH_upperleg_X_fr_LH_hip) * LH_hip_a + vcross.col(iit::rbd::AZ) * qd(LH_HFE_JOINT);
    LH_upperleg_f = LH_upperleg_I * LH_upperleg_a + iit::rbd::vxIv(LH_upperleg_v, LH_upperleg_I);
    
    // Link 'LH_lowerleg'
    LH_lowerleg_v = ((xm->fr_LH_lowerleg_X_fr_LH_upperleg) * LH_upperleg_v);
    LH_lowerleg_v(iit::rbd::AZ) += qd(LH_KFE_JOINT);
    iit::rbd::motionCrossProductMx<Scalar>(LH_lowerleg_v, vcross);
    LH_lowerleg_a = (xm->fr_LH_lowerleg_X_fr_LH_upperleg) * LH_upperleg_a + vcross.col(iit::rbd::AZ) * qd(LH_KFE_JOINT);
    LH_lowerleg_f = LH_lowerleg_I * LH_lowerleg_a + iit::rbd::vxIv(LH_lowerleg_v, LH_lowerleg_I);
    
    // Link 'LF_hip'
    LF_hip_v = ((xm->fr_LF_hip_X_fr_torso) * torso_v);
    LF_hip_v(iit::rbd::AZ) += qd(LF_HAA_JOINT);
    iit::rbd::motionCrossProductMx<Scalar>(LF_hip_v, vcross);
    LF_hip_a = (vcross.col(iit::rbd::AZ) * qd(LF_HAA_JOINT));
    LF_hip_f = LF_hip_I * LF_hip_a + iit::rbd::vxIv(LF_hip_v, LF_hip_I);
    
    // Link 'LF_upperleg'
    LF_upperleg_v = ((xm->fr_LF_upperleg_X_fr_LF_hip) * LF_hip_v);
    LF_upperleg_v(iit::rbd::AZ) += qd(LF_HFE_JOINT);
    iit::rbd::motionCrossProductMx<Scalar>(LF_upperleg_v, vcross);
    LF_upperleg_a = (xm->fr_LF_upperleg_X_fr_LF_hip) * LF_hip_a + vcross.col(iit::rbd::AZ) * qd(LF_HFE_JOINT);
    LF_upperleg_f = LF_upperleg_I * LF_upperleg_a + iit::rbd::vxIv(LF_upperleg_v, LF_upperleg_I);
    
    // Link 'LF_lowerleg'
    LF_lowerleg_v = ((xm->fr_LF_lowerleg_X_fr_LF_upperleg) * LF_upperleg_v);
    LF_lowerleg_v(iit::rbd::AZ) += qd(LF_KFE_JOINT);
    iit::rbd::motionCrossProductMx<Scalar>(LF_lowerleg_v, vcross);
    LF_lowerleg_a = (xm->fr_LF_lowerleg_X_fr_LF_upperleg) * LF_upperleg_a + vcross.col(iit::rbd::AZ) * qd(LF_KFE_JOINT);
    LF_lowerleg_f = LF_lowerleg_I * LF_lowerleg_a + iit::rbd::vxIv(LF_lowerleg_v, LF_lowerleg_I);
    

    torso_f = iit::rbd::vxIv(torso_v, torso_I);

    secondPass_fullyActuated(jForces);

    baseWrench = torso_f;
}

template <typename TRAIT>
void iit::pegasus2::dyn::tpl::InverseDynamics<TRAIT>::id_fully_actuated(
        Force& baseWrench, JointState& jForces,
        const Acceleration& g, const Velocity& torso_v, const Acceleration& baseAccel,
        const JointState& qd, const JointState& qdd, const ExtForces& fext)
{
    Acceleration torso_a = baseAccel -g;

    // First pass, link 'RF_hip'
    RF_hip_v = ((xm->fr_RF_hip_X_fr_torso) * torso_v);
    RF_hip_v(iit::rbd::AZ) += qd(RF_HAA_JOINT);
    
    iit::rbd::motionCrossProductMx<Scalar>(RF_hip_v, vcross);
    
    RF_hip_a = (xm->fr_RF_hip_X_fr_torso) * torso_a + vcross.col(iit::rbd::AZ) * qd(RF_HAA_JOINT);
    RF_hip_a(iit::rbd::AZ) += qdd(RF_HAA_JOINT);
    
    RF_hip_f = RF_hip_I * RF_hip_a + iit::rbd::vxIv(RF_hip_v, RF_hip_I) - fext[RF_HIP];
    
    // First pass, link 'RF_upperleg'
    RF_upperleg_v = ((xm->fr_RF_upperleg_X_fr_RF_hip) * RF_hip_v);
    RF_upperleg_v(iit::rbd::AZ) += qd(RF_HFE_JOINT);
    
    iit::rbd::motionCrossProductMx<Scalar>(RF_upperleg_v, vcross);
    
    RF_upperleg_a = (xm->fr_RF_upperleg_X_fr_RF_hip) * RF_hip_a + vcross.col(iit::rbd::AZ) * qd(RF_HFE_JOINT);
    RF_upperleg_a(iit::rbd::AZ) += qdd(RF_HFE_JOINT);
    
    RF_upperleg_f = RF_upperleg_I * RF_upperleg_a + iit::rbd::vxIv(RF_upperleg_v, RF_upperleg_I) - fext[RF_UPPERLEG];
    
    // First pass, link 'RF_lowerleg'
    RF_lowerleg_v = ((xm->fr_RF_lowerleg_X_fr_RF_upperleg) * RF_upperleg_v);
    RF_lowerleg_v(iit::rbd::AZ) += qd(RF_KFE_JOINT);
    
    iit::rbd::motionCrossProductMx<Scalar>(RF_lowerleg_v, vcross);
    
    RF_lowerleg_a = (xm->fr_RF_lowerleg_X_fr_RF_upperleg) * RF_upperleg_a + vcross.col(iit::rbd::AZ) * qd(RF_KFE_JOINT);
    RF_lowerleg_a(iit::rbd::AZ) += qdd(RF_KFE_JOINT);
    
    RF_lowerleg_f = RF_lowerleg_I * RF_lowerleg_a + iit::rbd::vxIv(RF_lowerleg_v, RF_lowerleg_I) - fext[RF_LOWERLEG];
    
    // First pass, link 'RH_hip'
    RH_hip_v = ((xm->fr_RH_hip_X_fr_torso) * torso_v);
    RH_hip_v(iit::rbd::AZ) += qd(RH_HAA_JOINT);
    
    iit::rbd::motionCrossProductMx<Scalar>(RH_hip_v, vcross);
    
    RH_hip_a = (xm->fr_RH_hip_X_fr_torso) * torso_a + vcross.col(iit::rbd::AZ) * qd(RH_HAA_JOINT);
    RH_hip_a(iit::rbd::AZ) += qdd(RH_HAA_JOINT);
    
    RH_hip_f = RH_hip_I * RH_hip_a + iit::rbd::vxIv(RH_hip_v, RH_hip_I) - fext[RH_HIP];
    
    // First pass, link 'RH_upperleg'
    RH_upperleg_v = ((xm->fr_RH_upperleg_X_fr_RH_hip) * RH_hip_v);
    RH_upperleg_v(iit::rbd::AZ) += qd(RH_HFE_JOINT);
    
    iit::rbd::motionCrossProductMx<Scalar>(RH_upperleg_v, vcross);
    
    RH_upperleg_a = (xm->fr_RH_upperleg_X_fr_RH_hip) * RH_hip_a + vcross.col(iit::rbd::AZ) * qd(RH_HFE_JOINT);
    RH_upperleg_a(iit::rbd::AZ) += qdd(RH_HFE_JOINT);
    
    RH_upperleg_f = RH_upperleg_I * RH_upperleg_a + iit::rbd::vxIv(RH_upperleg_v, RH_upperleg_I) - fext[RH_UPPERLEG];
    
    // First pass, link 'RH_lowerleg'
    RH_lowerleg_v = ((xm->fr_RH_lowerleg_X_fr_RH_upperleg) * RH_upperleg_v);
    RH_lowerleg_v(iit::rbd::AZ) += qd(RH_KFE_JOINT);
    
    iit::rbd::motionCrossProductMx<Scalar>(RH_lowerleg_v, vcross);
    
    RH_lowerleg_a = (xm->fr_RH_lowerleg_X_fr_RH_upperleg) * RH_upperleg_a + vcross.col(iit::rbd::AZ) * qd(RH_KFE_JOINT);
    RH_lowerleg_a(iit::rbd::AZ) += qdd(RH_KFE_JOINT);
    
    RH_lowerleg_f = RH_lowerleg_I * RH_lowerleg_a + iit::rbd::vxIv(RH_lowerleg_v, RH_lowerleg_I) - fext[RH_LOWERLEG];
    
    // First pass, link 'LH_hip'
    LH_hip_v = ((xm->fr_LH_hip_X_fr_torso) * torso_v);
    LH_hip_v(iit::rbd::AZ) += qd(LH_HAA_JOINT);
    
    iit::rbd::motionCrossProductMx<Scalar>(LH_hip_v, vcross);
    
    LH_hip_a = (xm->fr_LH_hip_X_fr_torso) * torso_a + vcross.col(iit::rbd::AZ) * qd(LH_HAA_JOINT);
    LH_hip_a(iit::rbd::AZ) += qdd(LH_HAA_JOINT);
    
    LH_hip_f = LH_hip_I * LH_hip_a + iit::rbd::vxIv(LH_hip_v, LH_hip_I) - fext[LH_HIP];
    
    // First pass, link 'LH_upperleg'
    LH_upperleg_v = ((xm->fr_LH_upperleg_X_fr_LH_hip) * LH_hip_v);
    LH_upperleg_v(iit::rbd::AZ) += qd(LH_HFE_JOINT);
    
    iit::rbd::motionCrossProductMx<Scalar>(LH_upperleg_v, vcross);
    
    LH_upperleg_a = (xm->fr_LH_upperleg_X_fr_LH_hip) * LH_hip_a + vcross.col(iit::rbd::AZ) * qd(LH_HFE_JOINT);
    LH_upperleg_a(iit::rbd::AZ) += qdd(LH_HFE_JOINT);
    
    LH_upperleg_f = LH_upperleg_I * LH_upperleg_a + iit::rbd::vxIv(LH_upperleg_v, LH_upperleg_I) - fext[LH_UPPERLEG];
    
    // First pass, link 'LH_lowerleg'
    LH_lowerleg_v = ((xm->fr_LH_lowerleg_X_fr_LH_upperleg) * LH_upperleg_v);
    LH_lowerleg_v(iit::rbd::AZ) += qd(LH_KFE_JOINT);
    
    iit::rbd::motionCrossProductMx<Scalar>(LH_lowerleg_v, vcross);
    
    LH_lowerleg_a = (xm->fr_LH_lowerleg_X_fr_LH_upperleg) * LH_upperleg_a + vcross.col(iit::rbd::AZ) * qd(LH_KFE_JOINT);
    LH_lowerleg_a(iit::rbd::AZ) += qdd(LH_KFE_JOINT);
    
    LH_lowerleg_f = LH_lowerleg_I * LH_lowerleg_a + iit::rbd::vxIv(LH_lowerleg_v, LH_lowerleg_I) - fext[LH_LOWERLEG];
    
    // First pass, link 'LF_hip'
    LF_hip_v = ((xm->fr_LF_hip_X_fr_torso) * torso_v);
    LF_hip_v(iit::rbd::AZ) += qd(LF_HAA_JOINT);
    
    iit::rbd::motionCrossProductMx<Scalar>(LF_hip_v, vcross);
    
    LF_hip_a = (xm->fr_LF_hip_X_fr_torso) * torso_a + vcross.col(iit::rbd::AZ) * qd(LF_HAA_JOINT);
    LF_hip_a(iit::rbd::AZ) += qdd(LF_HAA_JOINT);
    
    LF_hip_f = LF_hip_I * LF_hip_a + iit::rbd::vxIv(LF_hip_v, LF_hip_I) - fext[LF_HIP];
    
    // First pass, link 'LF_upperleg'
    LF_upperleg_v = ((xm->fr_LF_upperleg_X_fr_LF_hip) * LF_hip_v);
    LF_upperleg_v(iit::rbd::AZ) += qd(LF_HFE_JOINT);
    
    iit::rbd::motionCrossProductMx<Scalar>(LF_upperleg_v, vcross);
    
    LF_upperleg_a = (xm->fr_LF_upperleg_X_fr_LF_hip) * LF_hip_a + vcross.col(iit::rbd::AZ) * qd(LF_HFE_JOINT);
    LF_upperleg_a(iit::rbd::AZ) += qdd(LF_HFE_JOINT);
    
    LF_upperleg_f = LF_upperleg_I * LF_upperleg_a + iit::rbd::vxIv(LF_upperleg_v, LF_upperleg_I) - fext[LF_UPPERLEG];
    
    // First pass, link 'LF_lowerleg'
    LF_lowerleg_v = ((xm->fr_LF_lowerleg_X_fr_LF_upperleg) * LF_upperleg_v);
    LF_lowerleg_v(iit::rbd::AZ) += qd(LF_KFE_JOINT);
    
    iit::rbd::motionCrossProductMx<Scalar>(LF_lowerleg_v, vcross);
    
    LF_lowerleg_a = (xm->fr_LF_lowerleg_X_fr_LF_upperleg) * LF_upperleg_a + vcross.col(iit::rbd::AZ) * qd(LF_KFE_JOINT);
    LF_lowerleg_a(iit::rbd::AZ) += qdd(LF_KFE_JOINT);
    
    LF_lowerleg_f = LF_lowerleg_I * LF_lowerleg_a + iit::rbd::vxIv(LF_lowerleg_v, LF_lowerleg_I) - fext[LF_LOWERLEG];
    

    // The base
    torso_f = torso_I * torso_a + iit::rbd::vxIv(torso_v, torso_I) - fext[TORSO];

    secondPass_fullyActuated(jForces);

    baseWrench = torso_f;
}

template <typename TRAIT>
void iit::pegasus2::dyn::tpl::InverseDynamics<TRAIT>::secondPass_fullyActuated(JointState& jForces)
{
    // Link 'LF_lowerleg'
    jForces(LF_KFE_JOINT) = LF_lowerleg_f(iit::rbd::AZ);
    LF_upperleg_f += xm->fr_LF_lowerleg_X_fr_LF_upperleg.transpose() * LF_lowerleg_f;
    // Link 'LF_upperleg'
    jForces(LF_HFE_JOINT) = LF_upperleg_f(iit::rbd::AZ);
    LF_hip_f += xm->fr_LF_upperleg_X_fr_LF_hip.transpose() * LF_upperleg_f;
    // Link 'LF_hip'
    jForces(LF_HAA_JOINT) = LF_hip_f(iit::rbd::AZ);
    torso_f += xm->fr_LF_hip_X_fr_torso.transpose() * LF_hip_f;
    // Link 'LH_lowerleg'
    jForces(LH_KFE_JOINT) = LH_lowerleg_f(iit::rbd::AZ);
    LH_upperleg_f += xm->fr_LH_lowerleg_X_fr_LH_upperleg.transpose() * LH_lowerleg_f;
    // Link 'LH_upperleg'
    jForces(LH_HFE_JOINT) = LH_upperleg_f(iit::rbd::AZ);
    LH_hip_f += xm->fr_LH_upperleg_X_fr_LH_hip.transpose() * LH_upperleg_f;
    // Link 'LH_hip'
    jForces(LH_HAA_JOINT) = LH_hip_f(iit::rbd::AZ);
    torso_f += xm->fr_LH_hip_X_fr_torso.transpose() * LH_hip_f;
    // Link 'RH_lowerleg'
    jForces(RH_KFE_JOINT) = RH_lowerleg_f(iit::rbd::AZ);
    RH_upperleg_f += xm->fr_RH_lowerleg_X_fr_RH_upperleg.transpose() * RH_lowerleg_f;
    // Link 'RH_upperleg'
    jForces(RH_HFE_JOINT) = RH_upperleg_f(iit::rbd::AZ);
    RH_hip_f += xm->fr_RH_upperleg_X_fr_RH_hip.transpose() * RH_upperleg_f;
    // Link 'RH_hip'
    jForces(RH_HAA_JOINT) = RH_hip_f(iit::rbd::AZ);
    torso_f += xm->fr_RH_hip_X_fr_torso.transpose() * RH_hip_f;
    // Link 'RF_lowerleg'
    jForces(RF_KFE_JOINT) = RF_lowerleg_f(iit::rbd::AZ);
    RF_upperleg_f += xm->fr_RF_lowerleg_X_fr_RF_upperleg.transpose() * RF_lowerleg_f;
    // Link 'RF_upperleg'
    jForces(RF_HFE_JOINT) = RF_upperleg_f(iit::rbd::AZ);
    RF_hip_f += xm->fr_RF_upperleg_X_fr_RF_hip.transpose() * RF_upperleg_f;
    // Link 'RF_hip'
    jForces(RF_HAA_JOINT) = RF_hip_f(iit::rbd::AZ);
    torso_f += xm->fr_RF_hip_X_fr_torso.transpose() * RF_hip_f;
}

