
// Initialization of static-const data
template <typename TRAIT>
const typename iit::pegasus2::dyn::tpl::ForwardDynamics<TRAIT>::ExtForces
    iit::pegasus2::dyn::tpl::ForwardDynamics<TRAIT>::zeroExtForces(Force::Zero());

template <typename TRAIT>
iit::pegasus2::dyn::tpl::ForwardDynamics<TRAIT>::ForwardDynamics(iit::pegasus2::dyn::tpl::InertiaProperties<TRAIT>& inertia, MTransforms& transforms) :
    inertiaProps( & inertia ),
    motionTransforms( & transforms )
{
    RF_hip_v.setZero();
    RF_hip_c.setZero();
    RF_upperleg_v.setZero();
    RF_upperleg_c.setZero();
    RF_lowerleg_v.setZero();
    RF_lowerleg_c.setZero();
    RH_hip_v.setZero();
    RH_hip_c.setZero();
    RH_upperleg_v.setZero();
    RH_upperleg_c.setZero();
    RH_lowerleg_v.setZero();
    RH_lowerleg_c.setZero();
    LH_hip_v.setZero();
    LH_hip_c.setZero();
    LH_upperleg_v.setZero();
    LH_upperleg_c.setZero();
    LH_lowerleg_v.setZero();
    LH_lowerleg_c.setZero();
    LF_hip_v.setZero();
    LF_hip_c.setZero();
    LF_upperleg_v.setZero();
    LF_upperleg_c.setZero();
    LF_lowerleg_v.setZero();
    LF_lowerleg_c.setZero();

    vcross.setZero();
    Ia_r.setZero();

}

template <typename TRAIT>
void iit::pegasus2::dyn::tpl::ForwardDynamics<TRAIT>::fd(
    JointState& qdd,
    Acceleration& torso_a,
    const Velocity& torso_v,
    const Acceleration& g,
    const JointState& qd,
    const JointState& tau,
    const ExtForces& fext/* = zeroExtForces */)
{
    
    torso_AI = inertiaProps->getTensor_torso();
    torso_p = - fext[TORSO];
    RF_hip_AI = inertiaProps->getTensor_RF_hip();
    RF_hip_p = - fext[RF_HIP];
    RF_upperleg_AI = inertiaProps->getTensor_RF_upperleg();
    RF_upperleg_p = - fext[RF_UPPERLEG];
    RF_lowerleg_AI = inertiaProps->getTensor_RF_lowerleg();
    RF_lowerleg_p = - fext[RF_LOWERLEG];
    RH_hip_AI = inertiaProps->getTensor_RH_hip();
    RH_hip_p = - fext[RH_HIP];
    RH_upperleg_AI = inertiaProps->getTensor_RH_upperleg();
    RH_upperleg_p = - fext[RH_UPPERLEG];
    RH_lowerleg_AI = inertiaProps->getTensor_RH_lowerleg();
    RH_lowerleg_p = - fext[RH_LOWERLEG];
    LH_hip_AI = inertiaProps->getTensor_LH_hip();
    LH_hip_p = - fext[LH_HIP];
    LH_upperleg_AI = inertiaProps->getTensor_LH_upperleg();
    LH_upperleg_p = - fext[LH_UPPERLEG];
    LH_lowerleg_AI = inertiaProps->getTensor_LH_lowerleg();
    LH_lowerleg_p = - fext[LH_LOWERLEG];
    LF_hip_AI = inertiaProps->getTensor_LF_hip();
    LF_hip_p = - fext[LF_HIP];
    LF_upperleg_AI = inertiaProps->getTensor_LF_upperleg();
    LF_upperleg_p = - fext[LF_UPPERLEG];
    LF_lowerleg_AI = inertiaProps->getTensor_LF_lowerleg();
    LF_lowerleg_p = - fext[LF_LOWERLEG];
    // ---------------------- FIRST PASS ---------------------- //
    // Note that, during the first pass, the articulated inertias are really
    //  just the spatial inertia of the links (see assignments above).
    //  Afterwards things change, and articulated inertias shall not be used
    //  in functions which work specifically with spatial inertias.
    
    // + Link RF_hip
    //  - The spatial velocity:
    RF_hip_v = (motionTransforms-> fr_RF_hip_X_fr_torso) * torso_v;
    RF_hip_v(iit::rbd::AZ) += qd(RF_HAA_JOINT);
    
    //  - The velocity-product acceleration term:
    iit::rbd::motionCrossProductMx<Scalar>(RF_hip_v, vcross);
    RF_hip_c = vcross.col(iit::rbd::AZ) * qd(RF_HAA_JOINT);
    
    //  - The bias force term:
    RF_hip_p += iit::rbd::vxIv(RF_hip_v, RF_hip_AI);
    
    // + Link RF_upperleg
    //  - The spatial velocity:
    RF_upperleg_v = (motionTransforms-> fr_RF_upperleg_X_fr_RF_hip) * RF_hip_v;
    RF_upperleg_v(iit::rbd::AZ) += qd(RF_HFE_JOINT);
    
    //  - The velocity-product acceleration term:
    iit::rbd::motionCrossProductMx<Scalar>(RF_upperleg_v, vcross);
    RF_upperleg_c = vcross.col(iit::rbd::AZ) * qd(RF_HFE_JOINT);
    
    //  - The bias force term:
    RF_upperleg_p += iit::rbd::vxIv(RF_upperleg_v, RF_upperleg_AI);
    
    // + Link RF_lowerleg
    //  - The spatial velocity:
    RF_lowerleg_v = (motionTransforms-> fr_RF_lowerleg_X_fr_RF_upperleg) * RF_upperleg_v;
    RF_lowerleg_v(iit::rbd::AZ) += qd(RF_KFE_JOINT);
    
    //  - The velocity-product acceleration term:
    iit::rbd::motionCrossProductMx<Scalar>(RF_lowerleg_v, vcross);
    RF_lowerleg_c = vcross.col(iit::rbd::AZ) * qd(RF_KFE_JOINT);
    
    //  - The bias force term:
    RF_lowerleg_p += iit::rbd::vxIv(RF_lowerleg_v, RF_lowerleg_AI);
    
    // + Link RH_hip
    //  - The spatial velocity:
    RH_hip_v = (motionTransforms-> fr_RH_hip_X_fr_torso) * torso_v;
    RH_hip_v(iit::rbd::AZ) += qd(RH_HAA_JOINT);
    
    //  - The velocity-product acceleration term:
    iit::rbd::motionCrossProductMx<Scalar>(RH_hip_v, vcross);
    RH_hip_c = vcross.col(iit::rbd::AZ) * qd(RH_HAA_JOINT);
    
    //  - The bias force term:
    RH_hip_p += iit::rbd::vxIv(RH_hip_v, RH_hip_AI);
    
    // + Link RH_upperleg
    //  - The spatial velocity:
    RH_upperleg_v = (motionTransforms-> fr_RH_upperleg_X_fr_RH_hip) * RH_hip_v;
    RH_upperleg_v(iit::rbd::AZ) += qd(RH_HFE_JOINT);
    
    //  - The velocity-product acceleration term:
    iit::rbd::motionCrossProductMx<Scalar>(RH_upperleg_v, vcross);
    RH_upperleg_c = vcross.col(iit::rbd::AZ) * qd(RH_HFE_JOINT);
    
    //  - The bias force term:
    RH_upperleg_p += iit::rbd::vxIv(RH_upperleg_v, RH_upperleg_AI);
    
    // + Link RH_lowerleg
    //  - The spatial velocity:
    RH_lowerleg_v = (motionTransforms-> fr_RH_lowerleg_X_fr_RH_upperleg) * RH_upperleg_v;
    RH_lowerleg_v(iit::rbd::AZ) += qd(RH_KFE_JOINT);
    
    //  - The velocity-product acceleration term:
    iit::rbd::motionCrossProductMx<Scalar>(RH_lowerleg_v, vcross);
    RH_lowerleg_c = vcross.col(iit::rbd::AZ) * qd(RH_KFE_JOINT);
    
    //  - The bias force term:
    RH_lowerleg_p += iit::rbd::vxIv(RH_lowerleg_v, RH_lowerleg_AI);
    
    // + Link LH_hip
    //  - The spatial velocity:
    LH_hip_v = (motionTransforms-> fr_LH_hip_X_fr_torso) * torso_v;
    LH_hip_v(iit::rbd::AZ) += qd(LH_HAA_JOINT);
    
    //  - The velocity-product acceleration term:
    iit::rbd::motionCrossProductMx<Scalar>(LH_hip_v, vcross);
    LH_hip_c = vcross.col(iit::rbd::AZ) * qd(LH_HAA_JOINT);
    
    //  - The bias force term:
    LH_hip_p += iit::rbd::vxIv(LH_hip_v, LH_hip_AI);
    
    // + Link LH_upperleg
    //  - The spatial velocity:
    LH_upperleg_v = (motionTransforms-> fr_LH_upperleg_X_fr_LH_hip) * LH_hip_v;
    LH_upperleg_v(iit::rbd::AZ) += qd(LH_HFE_JOINT);
    
    //  - The velocity-product acceleration term:
    iit::rbd::motionCrossProductMx<Scalar>(LH_upperleg_v, vcross);
    LH_upperleg_c = vcross.col(iit::rbd::AZ) * qd(LH_HFE_JOINT);
    
    //  - The bias force term:
    LH_upperleg_p += iit::rbd::vxIv(LH_upperleg_v, LH_upperleg_AI);
    
    // + Link LH_lowerleg
    //  - The spatial velocity:
    LH_lowerleg_v = (motionTransforms-> fr_LH_lowerleg_X_fr_LH_upperleg) * LH_upperleg_v;
    LH_lowerleg_v(iit::rbd::AZ) += qd(LH_KFE_JOINT);
    
    //  - The velocity-product acceleration term:
    iit::rbd::motionCrossProductMx<Scalar>(LH_lowerleg_v, vcross);
    LH_lowerleg_c = vcross.col(iit::rbd::AZ) * qd(LH_KFE_JOINT);
    
    //  - The bias force term:
    LH_lowerleg_p += iit::rbd::vxIv(LH_lowerleg_v, LH_lowerleg_AI);
    
    // + Link LF_hip
    //  - The spatial velocity:
    LF_hip_v = (motionTransforms-> fr_LF_hip_X_fr_torso) * torso_v;
    LF_hip_v(iit::rbd::AZ) += qd(LF_HAA_JOINT);
    
    //  - The velocity-product acceleration term:
    iit::rbd::motionCrossProductMx<Scalar>(LF_hip_v, vcross);
    LF_hip_c = vcross.col(iit::rbd::AZ) * qd(LF_HAA_JOINT);
    
    //  - The bias force term:
    LF_hip_p += iit::rbd::vxIv(LF_hip_v, LF_hip_AI);
    
    // + Link LF_upperleg
    //  - The spatial velocity:
    LF_upperleg_v = (motionTransforms-> fr_LF_upperleg_X_fr_LF_hip) * LF_hip_v;
    LF_upperleg_v(iit::rbd::AZ) += qd(LF_HFE_JOINT);
    
    //  - The velocity-product acceleration term:
    iit::rbd::motionCrossProductMx<Scalar>(LF_upperleg_v, vcross);
    LF_upperleg_c = vcross.col(iit::rbd::AZ) * qd(LF_HFE_JOINT);
    
    //  - The bias force term:
    LF_upperleg_p += iit::rbd::vxIv(LF_upperleg_v, LF_upperleg_AI);
    
    // + Link LF_lowerleg
    //  - The spatial velocity:
    LF_lowerleg_v = (motionTransforms-> fr_LF_lowerleg_X_fr_LF_upperleg) * LF_upperleg_v;
    LF_lowerleg_v(iit::rbd::AZ) += qd(LF_KFE_JOINT);
    
    //  - The velocity-product acceleration term:
    iit::rbd::motionCrossProductMx<Scalar>(LF_lowerleg_v, vcross);
    LF_lowerleg_c = vcross.col(iit::rbd::AZ) * qd(LF_KFE_JOINT);
    
    //  - The bias force term:
    LF_lowerleg_p += iit::rbd::vxIv(LF_lowerleg_v, LF_lowerleg_AI);
    
    // + The floating base body
    torso_p += iit::rbd::vxIv(torso_v, torso_AI);
    
    // ---------------------- SECOND PASS ---------------------- //
    Matrix66S IaB;
    Force pa;
    
    // + Link LF_lowerleg
    LF_lowerleg_u = tau(LF_KFE_JOINT) - LF_lowerleg_p(iit::rbd::AZ);
    LF_lowerleg_U = LF_lowerleg_AI.col(iit::rbd::AZ);
    LF_lowerleg_D = LF_lowerleg_U(iit::rbd::AZ);
    
    iit::rbd::compute_Ia_revolute(LF_lowerleg_AI, LF_lowerleg_U, LF_lowerleg_D, Ia_r);  // same as: Ia_r = LF_lowerleg_AI - LF_lowerleg_U/LF_lowerleg_D * LF_lowerleg_U.transpose();
    pa = LF_lowerleg_p + Ia_r * LF_lowerleg_c + LF_lowerleg_U * LF_lowerleg_u/LF_lowerleg_D;
    ctransform_Ia_revolute(Ia_r, motionTransforms-> fr_LF_lowerleg_X_fr_LF_upperleg, IaB);
    LF_upperleg_AI += IaB;
    LF_upperleg_p += (motionTransforms-> fr_LF_lowerleg_X_fr_LF_upperleg).transpose() * pa;
    
    // + Link LF_upperleg
    LF_upperleg_u = tau(LF_HFE_JOINT) - LF_upperleg_p(iit::rbd::AZ);
    LF_upperleg_U = LF_upperleg_AI.col(iit::rbd::AZ);
    LF_upperleg_D = LF_upperleg_U(iit::rbd::AZ);
    
    iit::rbd::compute_Ia_revolute(LF_upperleg_AI, LF_upperleg_U, LF_upperleg_D, Ia_r);  // same as: Ia_r = LF_upperleg_AI - LF_upperleg_U/LF_upperleg_D * LF_upperleg_U.transpose();
    pa = LF_upperleg_p + Ia_r * LF_upperleg_c + LF_upperleg_U * LF_upperleg_u/LF_upperleg_D;
    ctransform_Ia_revolute(Ia_r, motionTransforms-> fr_LF_upperleg_X_fr_LF_hip, IaB);
    LF_hip_AI += IaB;
    LF_hip_p += (motionTransforms-> fr_LF_upperleg_X_fr_LF_hip).transpose() * pa;
    
    // + Link LF_hip
    LF_hip_u = tau(LF_HAA_JOINT) - LF_hip_p(iit::rbd::AZ);
    LF_hip_U = LF_hip_AI.col(iit::rbd::AZ);
    LF_hip_D = LF_hip_U(iit::rbd::AZ);
    
    iit::rbd::compute_Ia_revolute(LF_hip_AI, LF_hip_U, LF_hip_D, Ia_r);  // same as: Ia_r = LF_hip_AI - LF_hip_U/LF_hip_D * LF_hip_U.transpose();
    pa = LF_hip_p + Ia_r * LF_hip_c + LF_hip_U * LF_hip_u/LF_hip_D;
    ctransform_Ia_revolute(Ia_r, motionTransforms-> fr_LF_hip_X_fr_torso, IaB);
    torso_AI += IaB;
    torso_p += (motionTransforms-> fr_LF_hip_X_fr_torso).transpose() * pa;
    
    // + Link LH_lowerleg
    LH_lowerleg_u = tau(LH_KFE_JOINT) - LH_lowerleg_p(iit::rbd::AZ);
    LH_lowerleg_U = LH_lowerleg_AI.col(iit::rbd::AZ);
    LH_lowerleg_D = LH_lowerleg_U(iit::rbd::AZ);
    
    iit::rbd::compute_Ia_revolute(LH_lowerleg_AI, LH_lowerleg_U, LH_lowerleg_D, Ia_r);  // same as: Ia_r = LH_lowerleg_AI - LH_lowerleg_U/LH_lowerleg_D * LH_lowerleg_U.transpose();
    pa = LH_lowerleg_p + Ia_r * LH_lowerleg_c + LH_lowerleg_U * LH_lowerleg_u/LH_lowerleg_D;
    ctransform_Ia_revolute(Ia_r, motionTransforms-> fr_LH_lowerleg_X_fr_LH_upperleg, IaB);
    LH_upperleg_AI += IaB;
    LH_upperleg_p += (motionTransforms-> fr_LH_lowerleg_X_fr_LH_upperleg).transpose() * pa;
    
    // + Link LH_upperleg
    LH_upperleg_u = tau(LH_HFE_JOINT) - LH_upperleg_p(iit::rbd::AZ);
    LH_upperleg_U = LH_upperleg_AI.col(iit::rbd::AZ);
    LH_upperleg_D = LH_upperleg_U(iit::rbd::AZ);
    
    iit::rbd::compute_Ia_revolute(LH_upperleg_AI, LH_upperleg_U, LH_upperleg_D, Ia_r);  // same as: Ia_r = LH_upperleg_AI - LH_upperleg_U/LH_upperleg_D * LH_upperleg_U.transpose();
    pa = LH_upperleg_p + Ia_r * LH_upperleg_c + LH_upperleg_U * LH_upperleg_u/LH_upperleg_D;
    ctransform_Ia_revolute(Ia_r, motionTransforms-> fr_LH_upperleg_X_fr_LH_hip, IaB);
    LH_hip_AI += IaB;
    LH_hip_p += (motionTransforms-> fr_LH_upperleg_X_fr_LH_hip).transpose() * pa;
    
    // + Link LH_hip
    LH_hip_u = tau(LH_HAA_JOINT) - LH_hip_p(iit::rbd::AZ);
    LH_hip_U = LH_hip_AI.col(iit::rbd::AZ);
    LH_hip_D = LH_hip_U(iit::rbd::AZ);
    
    iit::rbd::compute_Ia_revolute(LH_hip_AI, LH_hip_U, LH_hip_D, Ia_r);  // same as: Ia_r = LH_hip_AI - LH_hip_U/LH_hip_D * LH_hip_U.transpose();
    pa = LH_hip_p + Ia_r * LH_hip_c + LH_hip_U * LH_hip_u/LH_hip_D;
    ctransform_Ia_revolute(Ia_r, motionTransforms-> fr_LH_hip_X_fr_torso, IaB);
    torso_AI += IaB;
    torso_p += (motionTransforms-> fr_LH_hip_X_fr_torso).transpose() * pa;
    
    // + Link RH_lowerleg
    RH_lowerleg_u = tau(RH_KFE_JOINT) - RH_lowerleg_p(iit::rbd::AZ);
    RH_lowerleg_U = RH_lowerleg_AI.col(iit::rbd::AZ);
    RH_lowerleg_D = RH_lowerleg_U(iit::rbd::AZ);
    
    iit::rbd::compute_Ia_revolute(RH_lowerleg_AI, RH_lowerleg_U, RH_lowerleg_D, Ia_r);  // same as: Ia_r = RH_lowerleg_AI - RH_lowerleg_U/RH_lowerleg_D * RH_lowerleg_U.transpose();
    pa = RH_lowerleg_p + Ia_r * RH_lowerleg_c + RH_lowerleg_U * RH_lowerleg_u/RH_lowerleg_D;
    ctransform_Ia_revolute(Ia_r, motionTransforms-> fr_RH_lowerleg_X_fr_RH_upperleg, IaB);
    RH_upperleg_AI += IaB;
    RH_upperleg_p += (motionTransforms-> fr_RH_lowerleg_X_fr_RH_upperleg).transpose() * pa;
    
    // + Link RH_upperleg
    RH_upperleg_u = tau(RH_HFE_JOINT) - RH_upperleg_p(iit::rbd::AZ);
    RH_upperleg_U = RH_upperleg_AI.col(iit::rbd::AZ);
    RH_upperleg_D = RH_upperleg_U(iit::rbd::AZ);
    
    iit::rbd::compute_Ia_revolute(RH_upperleg_AI, RH_upperleg_U, RH_upperleg_D, Ia_r);  // same as: Ia_r = RH_upperleg_AI - RH_upperleg_U/RH_upperleg_D * RH_upperleg_U.transpose();
    pa = RH_upperleg_p + Ia_r * RH_upperleg_c + RH_upperleg_U * RH_upperleg_u/RH_upperleg_D;
    ctransform_Ia_revolute(Ia_r, motionTransforms-> fr_RH_upperleg_X_fr_RH_hip, IaB);
    RH_hip_AI += IaB;
    RH_hip_p += (motionTransforms-> fr_RH_upperleg_X_fr_RH_hip).transpose() * pa;
    
    // + Link RH_hip
    RH_hip_u = tau(RH_HAA_JOINT) - RH_hip_p(iit::rbd::AZ);
    RH_hip_U = RH_hip_AI.col(iit::rbd::AZ);
    RH_hip_D = RH_hip_U(iit::rbd::AZ);
    
    iit::rbd::compute_Ia_revolute(RH_hip_AI, RH_hip_U, RH_hip_D, Ia_r);  // same as: Ia_r = RH_hip_AI - RH_hip_U/RH_hip_D * RH_hip_U.transpose();
    pa = RH_hip_p + Ia_r * RH_hip_c + RH_hip_U * RH_hip_u/RH_hip_D;
    ctransform_Ia_revolute(Ia_r, motionTransforms-> fr_RH_hip_X_fr_torso, IaB);
    torso_AI += IaB;
    torso_p += (motionTransforms-> fr_RH_hip_X_fr_torso).transpose() * pa;
    
    // + Link RF_lowerleg
    RF_lowerleg_u = tau(RF_KFE_JOINT) - RF_lowerleg_p(iit::rbd::AZ);
    RF_lowerleg_U = RF_lowerleg_AI.col(iit::rbd::AZ);
    RF_lowerleg_D = RF_lowerleg_U(iit::rbd::AZ);
    
    iit::rbd::compute_Ia_revolute(RF_lowerleg_AI, RF_lowerleg_U, RF_lowerleg_D, Ia_r);  // same as: Ia_r = RF_lowerleg_AI - RF_lowerleg_U/RF_lowerleg_D * RF_lowerleg_U.transpose();
    pa = RF_lowerleg_p + Ia_r * RF_lowerleg_c + RF_lowerleg_U * RF_lowerleg_u/RF_lowerleg_D;
    ctransform_Ia_revolute(Ia_r, motionTransforms-> fr_RF_lowerleg_X_fr_RF_upperleg, IaB);
    RF_upperleg_AI += IaB;
    RF_upperleg_p += (motionTransforms-> fr_RF_lowerleg_X_fr_RF_upperleg).transpose() * pa;
    
    // + Link RF_upperleg
    RF_upperleg_u = tau(RF_HFE_JOINT) - RF_upperleg_p(iit::rbd::AZ);
    RF_upperleg_U = RF_upperleg_AI.col(iit::rbd::AZ);
    RF_upperleg_D = RF_upperleg_U(iit::rbd::AZ);
    
    iit::rbd::compute_Ia_revolute(RF_upperleg_AI, RF_upperleg_U, RF_upperleg_D, Ia_r);  // same as: Ia_r = RF_upperleg_AI - RF_upperleg_U/RF_upperleg_D * RF_upperleg_U.transpose();
    pa = RF_upperleg_p + Ia_r * RF_upperleg_c + RF_upperleg_U * RF_upperleg_u/RF_upperleg_D;
    ctransform_Ia_revolute(Ia_r, motionTransforms-> fr_RF_upperleg_X_fr_RF_hip, IaB);
    RF_hip_AI += IaB;
    RF_hip_p += (motionTransforms-> fr_RF_upperleg_X_fr_RF_hip).transpose() * pa;
    
    // + Link RF_hip
    RF_hip_u = tau(RF_HAA_JOINT) - RF_hip_p(iit::rbd::AZ);
    RF_hip_U = RF_hip_AI.col(iit::rbd::AZ);
    RF_hip_D = RF_hip_U(iit::rbd::AZ);
    
    iit::rbd::compute_Ia_revolute(RF_hip_AI, RF_hip_U, RF_hip_D, Ia_r);  // same as: Ia_r = RF_hip_AI - RF_hip_U/RF_hip_D * RF_hip_U.transpose();
    pa = RF_hip_p + Ia_r * RF_hip_c + RF_hip_U * RF_hip_u/RF_hip_D;
    ctransform_Ia_revolute(Ia_r, motionTransforms-> fr_RF_hip_X_fr_torso, IaB);
    torso_AI += IaB;
    torso_p += (motionTransforms-> fr_RF_hip_X_fr_torso).transpose() * pa;
    
    // + The acceleration of the floating base torso, without gravity
    torso_a = - TRAIT::solve(torso_AI, torso_p);  // torso_a = - IA^-1 * torso_p
    
    // ---------------------- THIRD PASS ---------------------- //
    RF_hip_a = (motionTransforms-> fr_RF_hip_X_fr_torso) * torso_a + RF_hip_c;
    qdd(RF_HAA_JOINT) = (RF_hip_u - RF_hip_U.dot(RF_hip_a)) / RF_hip_D;
    RF_hip_a(iit::rbd::AZ) += qdd(RF_HAA_JOINT);
    
    RF_upperleg_a = (motionTransforms-> fr_RF_upperleg_X_fr_RF_hip) * RF_hip_a + RF_upperleg_c;
    qdd(RF_HFE_JOINT) = (RF_upperleg_u - RF_upperleg_U.dot(RF_upperleg_a)) / RF_upperleg_D;
    RF_upperleg_a(iit::rbd::AZ) += qdd(RF_HFE_JOINT);
    
    RF_lowerleg_a = (motionTransforms-> fr_RF_lowerleg_X_fr_RF_upperleg) * RF_upperleg_a + RF_lowerleg_c;
    qdd(RF_KFE_JOINT) = (RF_lowerleg_u - RF_lowerleg_U.dot(RF_lowerleg_a)) / RF_lowerleg_D;
    RF_lowerleg_a(iit::rbd::AZ) += qdd(RF_KFE_JOINT);
    
    RH_hip_a = (motionTransforms-> fr_RH_hip_X_fr_torso) * torso_a + RH_hip_c;
    qdd(RH_HAA_JOINT) = (RH_hip_u - RH_hip_U.dot(RH_hip_a)) / RH_hip_D;
    RH_hip_a(iit::rbd::AZ) += qdd(RH_HAA_JOINT);
    
    RH_upperleg_a = (motionTransforms-> fr_RH_upperleg_X_fr_RH_hip) * RH_hip_a + RH_upperleg_c;
    qdd(RH_HFE_JOINT) = (RH_upperleg_u - RH_upperleg_U.dot(RH_upperleg_a)) / RH_upperleg_D;
    RH_upperleg_a(iit::rbd::AZ) += qdd(RH_HFE_JOINT);
    
    RH_lowerleg_a = (motionTransforms-> fr_RH_lowerleg_X_fr_RH_upperleg) * RH_upperleg_a + RH_lowerleg_c;
    qdd(RH_KFE_JOINT) = (RH_lowerleg_u - RH_lowerleg_U.dot(RH_lowerleg_a)) / RH_lowerleg_D;
    RH_lowerleg_a(iit::rbd::AZ) += qdd(RH_KFE_JOINT);
    
    LH_hip_a = (motionTransforms-> fr_LH_hip_X_fr_torso) * torso_a + LH_hip_c;
    qdd(LH_HAA_JOINT) = (LH_hip_u - LH_hip_U.dot(LH_hip_a)) / LH_hip_D;
    LH_hip_a(iit::rbd::AZ) += qdd(LH_HAA_JOINT);
    
    LH_upperleg_a = (motionTransforms-> fr_LH_upperleg_X_fr_LH_hip) * LH_hip_a + LH_upperleg_c;
    qdd(LH_HFE_JOINT) = (LH_upperleg_u - LH_upperleg_U.dot(LH_upperleg_a)) / LH_upperleg_D;
    LH_upperleg_a(iit::rbd::AZ) += qdd(LH_HFE_JOINT);
    
    LH_lowerleg_a = (motionTransforms-> fr_LH_lowerleg_X_fr_LH_upperleg) * LH_upperleg_a + LH_lowerleg_c;
    qdd(LH_KFE_JOINT) = (LH_lowerleg_u - LH_lowerleg_U.dot(LH_lowerleg_a)) / LH_lowerleg_D;
    LH_lowerleg_a(iit::rbd::AZ) += qdd(LH_KFE_JOINT);
    
    LF_hip_a = (motionTransforms-> fr_LF_hip_X_fr_torso) * torso_a + LF_hip_c;
    qdd(LF_HAA_JOINT) = (LF_hip_u - LF_hip_U.dot(LF_hip_a)) / LF_hip_D;
    LF_hip_a(iit::rbd::AZ) += qdd(LF_HAA_JOINT);
    
    LF_upperleg_a = (motionTransforms-> fr_LF_upperleg_X_fr_LF_hip) * LF_hip_a + LF_upperleg_c;
    qdd(LF_HFE_JOINT) = (LF_upperleg_u - LF_upperleg_U.dot(LF_upperleg_a)) / LF_upperleg_D;
    LF_upperleg_a(iit::rbd::AZ) += qdd(LF_HFE_JOINT);
    
    LF_lowerleg_a = (motionTransforms-> fr_LF_lowerleg_X_fr_LF_upperleg) * LF_upperleg_a + LF_lowerleg_c;
    qdd(LF_KFE_JOINT) = (LF_lowerleg_u - LF_lowerleg_U.dot(LF_lowerleg_a)) / LF_lowerleg_D;
    LF_lowerleg_a(iit::rbd::AZ) += qdd(LF_KFE_JOINT);
    
    
    // + Add gravity to the acceleration of the floating base
    torso_a += g;
}
