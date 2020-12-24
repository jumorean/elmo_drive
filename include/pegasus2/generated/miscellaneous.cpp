#include <iit/rbd/utils.h>
#include "miscellaneous.h"

using namespace iit::pegasus2;
using namespace iit::pegasus2::dyn;

iit::rbd::Vector3d iit::pegasus2::getWholeBodyCOM(
    const InertiaProperties& inertiaProps,
    const HomogeneousTransforms& ht)
{
    iit::rbd::Vector3d tmpSum(iit::rbd::Vector3d::Zero());

    tmpSum += inertiaProps.getCOM_torso() * inertiaProps.getMass_torso();

    HomogeneousTransforms::MatrixType tmpX(HomogeneousTransforms::MatrixType::Identity());
    HomogeneousTransforms::MatrixType base_X_RF_HAA_joint_chain;
    HomogeneousTransforms::MatrixType base_X_RH_HAA_joint_chain;
    HomogeneousTransforms::MatrixType base_X_LH_HAA_joint_chain;
    HomogeneousTransforms::MatrixType base_X_LF_HAA_joint_chain;
    
    
    base_X_RF_HAA_joint_chain = tmpX * ht.fr_torso_X_fr_RF_hip;
    tmpSum += inertiaProps.getMass_RF_hip() *
            ( iit::rbd::Utils::transform(base_X_RF_HAA_joint_chain, inertiaProps.getCOM_RF_hip()));
    
    base_X_RF_HAA_joint_chain = base_X_RF_HAA_joint_chain * ht.fr_RF_hip_X_fr_RF_upperleg;
    tmpSum += inertiaProps.getMass_RF_upperleg() *
            ( iit::rbd::Utils::transform(base_X_RF_HAA_joint_chain, inertiaProps.getCOM_RF_upperleg()));
    
    base_X_RF_HAA_joint_chain = base_X_RF_HAA_joint_chain * ht.fr_RF_upperleg_X_fr_RF_lowerleg;
    tmpSum += inertiaProps.getMass_RF_lowerleg() *
            ( iit::rbd::Utils::transform(base_X_RF_HAA_joint_chain, inertiaProps.getCOM_RF_lowerleg()));
    
    base_X_RH_HAA_joint_chain = tmpX * ht.fr_torso_X_fr_RH_hip;
    tmpSum += inertiaProps.getMass_RH_hip() *
            ( iit::rbd::Utils::transform(base_X_RH_HAA_joint_chain, inertiaProps.getCOM_RH_hip()));
    
    base_X_RH_HAA_joint_chain = base_X_RH_HAA_joint_chain * ht.fr_RH_hip_X_fr_RH_upperleg;
    tmpSum += inertiaProps.getMass_RH_upperleg() *
            ( iit::rbd::Utils::transform(base_X_RH_HAA_joint_chain, inertiaProps.getCOM_RH_upperleg()));
    
    base_X_RH_HAA_joint_chain = base_X_RH_HAA_joint_chain * ht.fr_RH_upperleg_X_fr_RH_lowerleg;
    tmpSum += inertiaProps.getMass_RH_lowerleg() *
            ( iit::rbd::Utils::transform(base_X_RH_HAA_joint_chain, inertiaProps.getCOM_RH_lowerleg()));
    
    base_X_LH_HAA_joint_chain = tmpX * ht.fr_torso_X_fr_LH_hip;
    tmpSum += inertiaProps.getMass_LH_hip() *
            ( iit::rbd::Utils::transform(base_X_LH_HAA_joint_chain, inertiaProps.getCOM_LH_hip()));
    
    base_X_LH_HAA_joint_chain = base_X_LH_HAA_joint_chain * ht.fr_LH_hip_X_fr_LH_upperleg;
    tmpSum += inertiaProps.getMass_LH_upperleg() *
            ( iit::rbd::Utils::transform(base_X_LH_HAA_joint_chain, inertiaProps.getCOM_LH_upperleg()));
    
    base_X_LH_HAA_joint_chain = base_X_LH_HAA_joint_chain * ht.fr_LH_upperleg_X_fr_LH_lowerleg;
    tmpSum += inertiaProps.getMass_LH_lowerleg() *
            ( iit::rbd::Utils::transform(base_X_LH_HAA_joint_chain, inertiaProps.getCOM_LH_lowerleg()));
    
    base_X_LF_HAA_joint_chain = tmpX * ht.fr_torso_X_fr_LF_hip;
    tmpSum += inertiaProps.getMass_LF_hip() *
            ( iit::rbd::Utils::transform(base_X_LF_HAA_joint_chain, inertiaProps.getCOM_LF_hip()));
    
    base_X_LF_HAA_joint_chain = base_X_LF_HAA_joint_chain * ht.fr_LF_hip_X_fr_LF_upperleg;
    tmpSum += inertiaProps.getMass_LF_upperleg() *
            ( iit::rbd::Utils::transform(base_X_LF_HAA_joint_chain, inertiaProps.getCOM_LF_upperleg()));
    
    base_X_LF_HAA_joint_chain = base_X_LF_HAA_joint_chain * ht.fr_LF_upperleg_X_fr_LF_lowerleg;
    tmpSum += inertiaProps.getMass_LF_lowerleg() *
            ( iit::rbd::Utils::transform(base_X_LF_HAA_joint_chain, inertiaProps.getCOM_LF_lowerleg()));
    

    return tmpSum / inertiaProps.getTotalMass();
}

iit::rbd::Vector3d iit::pegasus2::getWholeBodyCOM(
    const InertiaProperties& inertiaProps,
    const JointState& q,
    HomogeneousTransforms& ht)
{
    // First updates the coordinate transforms that will be used by the routine
    ht.fr_torso_X_fr_RF_hip(q);
    ht.fr_torso_X_fr_RH_hip(q);
    ht.fr_torso_X_fr_LH_hip(q);
    ht.fr_torso_X_fr_LF_hip(q);
    ht.fr_RF_hip_X_fr_RF_upperleg(q);
    ht.fr_RF_upperleg_X_fr_RF_lowerleg(q);
    ht.fr_RH_hip_X_fr_RH_upperleg(q);
    ht.fr_RH_upperleg_X_fr_RH_lowerleg(q);
    ht.fr_LH_hip_X_fr_LH_upperleg(q);
    ht.fr_LH_upperleg_X_fr_LH_lowerleg(q);
    ht.fr_LF_hip_X_fr_LF_upperleg(q);
    ht.fr_LF_upperleg_X_fr_LF_lowerleg(q);

    // The actual calculus
    return getWholeBodyCOM(inertiaProps, ht);
}
