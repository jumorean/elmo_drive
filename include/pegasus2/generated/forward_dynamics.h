#ifndef IIT_ROBOT_PEGASUS2_FORWARD_DYNAMICS_H_
#define IIT_ROBOT_PEGASUS2_FORWARD_DYNAMICS_H_

#include <iit/rbd/rbd.h>
#include <iit/rbd/utils.h>
#include <iit/rbd/robcogen_commons.h>

#include "declarations.h"
#include "transforms.h"
#include "inertia_properties.h"
#include "link_data_map.h"

namespace iit {
namespace pegasus2 {
namespace dyn {

/**
 * The Forward Dynamics routine for the robot pegasus2.
 *
 * The parameters common to most of the methods are the joint status \c q, the
 * joint velocities \c qd and the joint forces \c tau. The accelerations \c qdd
 * will be filled with the computed values. Overloaded methods without the \c q
 * parameter use the current configuration of the robot; they are provided for
 * the sake of efficiency, in case the kinematics transforms of the robot have
 * already been updated elsewhere with the most recent configuration (eg by a
 * call to setJointStatus()), so that it would be useless to compute them again.
 */

namespace tpl {

template <typename TRAIT>
class ForwardDynamics {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    // Convenient type aliases:

    typedef typename TRAIT::Scalar Scalar;

    typedef iit::rbd::Core<Scalar> CoreS;

    typedef typename CoreS::ForceVector Force;
    typedef typename CoreS::VelocityVector Velocity;
    typedef typename CoreS::VelocityVector Acceleration;
    typedef typename CoreS::Column6D Column6DS;
    typedef typename CoreS::Matrix66 Matrix66S;
    typedef LinkDataMap<Force> ExtForces;
    typedef typename iit::pegasus2::tpl::JointState<Scalar> JointState;
    typedef iit::pegasus2::tpl::MotionTransforms<TRAIT> MTransforms;

public:
    /**
     * Default constructor
     * \param in the inertia properties of the links
     * \param tr the container of all the spatial motion transforms of
     *     the robot pegasus2, which will be used by this instance
     *     to compute the dynamics.
     */
    ForwardDynamics(InertiaProperties<TRAIT>& in, MTransforms& tr);
    /** \name Forward dynamics
     * The Articulated-Body-Algorithm to compute the joint accelerations
     */ ///@{
    /**
     * \param qdd the joint accelerations vector (output parameter).
     * \param torso_a
     * \param torso_v
     * \param g the gravity acceleration vector, expressed in the
     *          base coordinates
     * \param q the joint status vector
     * \param qd the joint velocities vector
     * \param tau the joint forces (torque or force)
     * \param fext the external forces, optional. Each force must be
     *              expressed in the reference frame of the link it is
     *              exerted on.
     */
    void fd(
       JointState& qdd, Acceleration& torso_a, // output parameters,
       const Velocity& torso_v, const Acceleration& g,
       const JointState& q, const JointState& qd, const JointState& tau, const ExtForces& fext = zeroExtForces);
    void fd(
        JointState& qdd, Acceleration& torso_a, // output parameters,
        const Velocity& torso_v, const Acceleration& g,
        const JointState& qd, const JointState& tau, const ExtForces& fext = zeroExtForces);
    ///@}

    /** Updates all the kinematics transforms used by this instance. */
    void setJointStatus(const JointState& q) const;

private:
    InertiaProperties<TRAIT>* inertiaProps;
    MTransforms* motionTransforms;

    Matrix66S vcross; // support variable
    Matrix66S Ia_r;   // support variable, articulated inertia in the case of a revolute joint
    // Link 'torso'
    Matrix66S torso_AI;
    Force torso_p;

    // Link 'RF_hip' :
    Matrix66S RF_hip_AI;
    Velocity RF_hip_a;
    Velocity RF_hip_v;
    Velocity RF_hip_c;
    Force    RF_hip_p;

    Column6DS RF_hip_U;
    Scalar RF_hip_D;
    Scalar RF_hip_u;
    // Link 'RF_upperleg' :
    Matrix66S RF_upperleg_AI;
    Velocity RF_upperleg_a;
    Velocity RF_upperleg_v;
    Velocity RF_upperleg_c;
    Force    RF_upperleg_p;

    Column6DS RF_upperleg_U;
    Scalar RF_upperleg_D;
    Scalar RF_upperleg_u;
    // Link 'RF_lowerleg' :
    Matrix66S RF_lowerleg_AI;
    Velocity RF_lowerleg_a;
    Velocity RF_lowerleg_v;
    Velocity RF_lowerleg_c;
    Force    RF_lowerleg_p;

    Column6DS RF_lowerleg_U;
    Scalar RF_lowerleg_D;
    Scalar RF_lowerleg_u;
    // Link 'RH_hip' :
    Matrix66S RH_hip_AI;
    Velocity RH_hip_a;
    Velocity RH_hip_v;
    Velocity RH_hip_c;
    Force    RH_hip_p;

    Column6DS RH_hip_U;
    Scalar RH_hip_D;
    Scalar RH_hip_u;
    // Link 'RH_upperleg' :
    Matrix66S RH_upperleg_AI;
    Velocity RH_upperleg_a;
    Velocity RH_upperleg_v;
    Velocity RH_upperleg_c;
    Force    RH_upperleg_p;

    Column6DS RH_upperleg_U;
    Scalar RH_upperleg_D;
    Scalar RH_upperleg_u;
    // Link 'RH_lowerleg' :
    Matrix66S RH_lowerleg_AI;
    Velocity RH_lowerleg_a;
    Velocity RH_lowerleg_v;
    Velocity RH_lowerleg_c;
    Force    RH_lowerleg_p;

    Column6DS RH_lowerleg_U;
    Scalar RH_lowerleg_D;
    Scalar RH_lowerleg_u;
    // Link 'LH_hip' :
    Matrix66S LH_hip_AI;
    Velocity LH_hip_a;
    Velocity LH_hip_v;
    Velocity LH_hip_c;
    Force    LH_hip_p;

    Column6DS LH_hip_U;
    Scalar LH_hip_D;
    Scalar LH_hip_u;
    // Link 'LH_upperleg' :
    Matrix66S LH_upperleg_AI;
    Velocity LH_upperleg_a;
    Velocity LH_upperleg_v;
    Velocity LH_upperleg_c;
    Force    LH_upperleg_p;

    Column6DS LH_upperleg_U;
    Scalar LH_upperleg_D;
    Scalar LH_upperleg_u;
    // Link 'LH_lowerleg' :
    Matrix66S LH_lowerleg_AI;
    Velocity LH_lowerleg_a;
    Velocity LH_lowerleg_v;
    Velocity LH_lowerleg_c;
    Force    LH_lowerleg_p;

    Column6DS LH_lowerleg_U;
    Scalar LH_lowerleg_D;
    Scalar LH_lowerleg_u;
    // Link 'LF_hip' :
    Matrix66S LF_hip_AI;
    Velocity LF_hip_a;
    Velocity LF_hip_v;
    Velocity LF_hip_c;
    Force    LF_hip_p;

    Column6DS LF_hip_U;
    Scalar LF_hip_D;
    Scalar LF_hip_u;
    // Link 'LF_upperleg' :
    Matrix66S LF_upperleg_AI;
    Velocity LF_upperleg_a;
    Velocity LF_upperleg_v;
    Velocity LF_upperleg_c;
    Force    LF_upperleg_p;

    Column6DS LF_upperleg_U;
    Scalar LF_upperleg_D;
    Scalar LF_upperleg_u;
    // Link 'LF_lowerleg' :
    Matrix66S LF_lowerleg_AI;
    Velocity LF_lowerleg_a;
    Velocity LF_lowerleg_v;
    Velocity LF_lowerleg_c;
    Force    LF_lowerleg_p;

    Column6DS LF_lowerleg_U;
    Scalar LF_lowerleg_D;
    Scalar LF_lowerleg_u;
private:
    static const ExtForces zeroExtForces;
};

template <typename TRAIT>
inline void ForwardDynamics<TRAIT>::setJointStatus(const JointState& q) const {
    (motionTransforms-> fr_RF_hip_X_fr_torso)(q);
    (motionTransforms-> fr_RF_upperleg_X_fr_RF_hip)(q);
    (motionTransforms-> fr_RF_lowerleg_X_fr_RF_upperleg)(q);
    (motionTransforms-> fr_RH_hip_X_fr_torso)(q);
    (motionTransforms-> fr_RH_upperleg_X_fr_RH_hip)(q);
    (motionTransforms-> fr_RH_lowerleg_X_fr_RH_upperleg)(q);
    (motionTransforms-> fr_LH_hip_X_fr_torso)(q);
    (motionTransforms-> fr_LH_upperleg_X_fr_LH_hip)(q);
    (motionTransforms-> fr_LH_lowerleg_X_fr_LH_upperleg)(q);
    (motionTransforms-> fr_LF_hip_X_fr_torso)(q);
    (motionTransforms-> fr_LF_upperleg_X_fr_LF_hip)(q);
    (motionTransforms-> fr_LF_lowerleg_X_fr_LF_upperleg)(q);
}

template <typename TRAIT>
inline void ForwardDynamics<TRAIT>::fd(
    JointState& qdd, Acceleration& torso_a, // output parameters,
    const Velocity& torso_v, const Acceleration& g,
    const JointState& q,
    const JointState& qd,
    const JointState& tau,
    const ExtForces& fext/* = zeroExtForces */)
{
    setJointStatus(q);
    fd(qdd, torso_a, torso_v, g, qd, tau, fext);
}

}

typedef tpl::ForwardDynamics<iit::rbd::DoubleTrait> ForwardDynamics;

}
}
}

#include "forward_dynamics.impl.h"

#endif
