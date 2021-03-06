#ifndef IIT_PEGASUS2_INVERSE_DYNAMICS_H_
#define IIT_PEGASUS2_INVERSE_DYNAMICS_H_

#include <iit/rbd/rbd.h>
#include <iit/rbd/InertiaMatrix.h>
#include <iit/rbd/utils.h>
#include <iit/rbd/robcogen_commons.h>
#include <iit/rbd/traits/DoubleTrait.h>

#include "declarations.h"
#include "inertia_properties.h"
#include "transforms.h"
#include "link_data_map.h"

namespace iit {
namespace pegasus2 {
namespace dyn {

/**
 * The Inverse Dynamics routine for the robot pegasus2.
 *
 * In addition to the full Newton-Euler algorithm, specialized versions to compute
 * only certain terms are provided.
 * The parameters common to most of the methods are the joint status vector \c q, the
 * joint velocity vector \c qd and the acceleration vector \c qdd.
 *
 * Additional overloaded methods are provided without the \c q parameter. These
 * methods use the current configuration of the robot; they are provided for the
 * sake of efficiency, in case the motion transforms of the robot have already
 * been updated elsewhere with the most recent configuration (eg by a call to
 * setJointStatus()), so that it is useless to compute them again.
 *
 * Whenever present, the external forces parameter is a set of external
 * wrenches acting on the robot links. Each wrench must be expressed in
 * the reference frame of the link it is excerted on.
 */

namespace tpl {

template <typename TRAIT>
class InverseDynamics {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    typedef typename TRAIT::Scalar Scalar;

    typedef iit::rbd::Core<Scalar> CoreS;

    typedef typename CoreS::ForceVector Force;
    typedef typename CoreS::VelocityVector Velocity;
    typedef typename CoreS::VelocityVector Acceleration;
    typedef typename CoreS::Matrix66 Matrix66s;
    typedef iit::rbd::tpl::InertiaMatrixDense<Scalar> InertiaMatrix;
    typedef iit::pegasus2::tpl::JointState<Scalar> JointState;
    typedef LinkDataMap<Force> ExtForces;
    typedef iit::pegasus2::tpl::MotionTransforms<TRAIT> MTransforms;
    typedef InertiaProperties<TRAIT> IProperties;

public:
    /**
     * Default constructor
     * \param in the inertia properties of the links
     * \param tr the container of all the spatial motion transforms of
     *     the robot pegasus2, which will be used by this instance
     *     to compute inverse-dynamics.
     */
    InverseDynamics(IProperties& in, MTransforms& tr);

    /** \name Inverse dynamics
     * The full algorithm for the inverse dynamics of this robot.
     *
     * All the spatial vectors in the parameters are expressed in base coordinates,
     * besides the external forces: each force must be expressed in the reference
     * frame of the link it is acting on.
     * \param[out] jForces the joint force vector required to achieve the desired accelerations
     * \param[out] baseAccel the spatial acceleration of the robot base
     * \param[in] g the gravity acceleration, as a spatial vector;
     *              gravity implicitly specifies the orientation of the base in space
     * \param[in] torso_v the spatial velocity of the base
     * \param[in] q the joint position vector
     * \param[in] qd the joint velocity vector
     * \param[in] qdd the desired joint acceleration vector
     * \param[in] fext the external forces acting on the links; this parameters
     *            defaults to zero
     */ ///@{
    void id(
        JointState& jForces, Acceleration& torso_a,
        const Acceleration& g, const Velocity& torso_v,
        const JointState& q, const JointState& qd, const JointState& qdd,
        const ExtForces& fext = zeroExtForces);
    void id(
        JointState& jForces, Acceleration& torso_a,
        const Acceleration& g, const Velocity& torso_v,
        const JointState& qd, const JointState& qdd,
        const ExtForces& fext = zeroExtForces);
    ///@}
    /** \name Inverse dynamics, fully actuated base
     * The inverse dynamics algorithm for the floating base robot,
     * in the assumption of a fully actuated base.
     *
     * All the spatial vectors in the parameters are expressed in base coordinates,
     * besides the external forces: each force must be expressed in the reference
     * frame of the link it is acting on.
     * \param[out] baseWrench the spatial force to be applied to the robot base to achieve
     *             the desired accelerations
     * \param[out] jForces the joint force vector required to achieve the desired accelerations
     * \param[in] g the gravity acceleration, as a spatial vector;
     *              gravity implicitly specifies the orientation of the base in space
     * \param[in] torso_v the spatial velocity of the base
     * \param[in] baseAccel the desired spatial acceleration of the robot base
     * \param[in] q the joint position vector
     * \param[in] qd the joint velocity vector
     * \param[in] qdd the desired joint acceleration vector
     * \param[in] fext the external forces acting on the links; this parameters
     *            defaults to zero
     */ ///@{
    void id_fully_actuated(
        Force& baseWrench, JointState& jForces,
        const Acceleration& g, const Velocity& torso_v, const Acceleration& baseAccel,
        const JointState& q, const JointState& qd, const JointState& qdd, const ExtForces& fext = zeroExtForces);
    void id_fully_actuated(
        Force& baseWrench, JointState& jForces,
        const Acceleration& g, const Velocity& torso_v, const Acceleration& baseAccel,
        const JointState& qd, const JointState& qdd, const ExtForces& fext = zeroExtForces);
    ///@}
    /** \name Gravity terms, fully actuated base
     */
    ///@{
    void G_terms_fully_actuated(
        Force& baseWrench, JointState& jForces,
        const Acceleration& g, const JointState& q);
    void G_terms_fully_actuated(
        Force& baseWrench, JointState& jForces,
        const Acceleration& g);
    ///@}
    /** \name Centrifugal and Coriolis terms, fully actuated base
     *
     * These functions take only velocity inputs, that is, they assume
     * a zero spatial acceleration of the base (in addition to zero acceleration
     * at the actuated joints).
     * Note that this is NOT the same as imposing zero acceleration
     * at the virtual 6-dof-floting-base joint, which would result, in general,
     * in a non-zero spatial acceleration of the base, due to velocity
     * product terms.
     */
    ///@{
    void C_terms_fully_actuated(
        Force& baseWrench, JointState& jForces,
        const Velocity& torso_v, const JointState& q, const JointState& qd);
    void C_terms_fully_actuated(
        Force& baseWrench, JointState& jForces,
        const Velocity& torso_v, const JointState& qd);
    ///@}
    /** Updates all the kinematics transforms used by the inverse dynamics routine. */
    void setJointStatus(const JointState& q) const;

public:
    /** \name Getters
     * These functions return various spatial quantities used internally
     * by the inverse dynamics routines, like the spatial acceleration
     * of the links.
     *
     * The getters can be useful to retrieve the additional data that is not
     * returned explicitly by the inverse dynamics routines even though it
     * is computed. For example, after a call to the inverse dynamics,
     * the spatial velocity of all the links has been determined and
     * can be accessed.
     *
     * However, beware that certain routines might not use some of the
     * spatial quantities, which therefore would retain their last value
     * without being updated nor reset (for example, the spatial velocity
     * of the links is unaffected by the computation of the gravity terms).
     */
    ///@{
    const Force& getForce_torso() const { return torso_f; }
    const Velocity& getVelocity_RF_hip() const { return RF_hip_v; }
    const Acceleration& getAcceleration_RF_hip() const { return RF_hip_a; }
    const Force& getForce_RF_hip() const { return RF_hip_f; }
    const Velocity& getVelocity_RF_upperleg() const { return RF_upperleg_v; }
    const Acceleration& getAcceleration_RF_upperleg() const { return RF_upperleg_a; }
    const Force& getForce_RF_upperleg() const { return RF_upperleg_f; }
    const Velocity& getVelocity_RF_lowerleg() const { return RF_lowerleg_v; }
    const Acceleration& getAcceleration_RF_lowerleg() const { return RF_lowerleg_a; }
    const Force& getForce_RF_lowerleg() const { return RF_lowerleg_f; }
    const Velocity& getVelocity_RH_hip() const { return RH_hip_v; }
    const Acceleration& getAcceleration_RH_hip() const { return RH_hip_a; }
    const Force& getForce_RH_hip() const { return RH_hip_f; }
    const Velocity& getVelocity_RH_upperleg() const { return RH_upperleg_v; }
    const Acceleration& getAcceleration_RH_upperleg() const { return RH_upperleg_a; }
    const Force& getForce_RH_upperleg() const { return RH_upperleg_f; }
    const Velocity& getVelocity_RH_lowerleg() const { return RH_lowerleg_v; }
    const Acceleration& getAcceleration_RH_lowerleg() const { return RH_lowerleg_a; }
    const Force& getForce_RH_lowerleg() const { return RH_lowerleg_f; }
    const Velocity& getVelocity_LH_hip() const { return LH_hip_v; }
    const Acceleration& getAcceleration_LH_hip() const { return LH_hip_a; }
    const Force& getForce_LH_hip() const { return LH_hip_f; }
    const Velocity& getVelocity_LH_upperleg() const { return LH_upperleg_v; }
    const Acceleration& getAcceleration_LH_upperleg() const { return LH_upperleg_a; }
    const Force& getForce_LH_upperleg() const { return LH_upperleg_f; }
    const Velocity& getVelocity_LH_lowerleg() const { return LH_lowerleg_v; }
    const Acceleration& getAcceleration_LH_lowerleg() const { return LH_lowerleg_a; }
    const Force& getForce_LH_lowerleg() const { return LH_lowerleg_f; }
    const Velocity& getVelocity_LF_hip() const { return LF_hip_v; }
    const Acceleration& getAcceleration_LF_hip() const { return LF_hip_a; }
    const Force& getForce_LF_hip() const { return LF_hip_f; }
    const Velocity& getVelocity_LF_upperleg() const { return LF_upperleg_v; }
    const Acceleration& getAcceleration_LF_upperleg() const { return LF_upperleg_a; }
    const Force& getForce_LF_upperleg() const { return LF_upperleg_f; }
    const Velocity& getVelocity_LF_lowerleg() const { return LF_lowerleg_v; }
    const Acceleration& getAcceleration_LF_lowerleg() const { return LF_lowerleg_a; }
    const Force& getForce_LF_lowerleg() const { return LF_lowerleg_f; }
    ///@}
protected:
    void secondPass_fullyActuated(JointState& jForces);

private:
    IProperties* inertiaProps;
    MTransforms* xm;
private:
    Matrix66s vcross; // support variable
    // Link 'RF_hip' :
    const InertiaMatrix& RF_hip_I;
    Velocity      RF_hip_v;
    Acceleration  RF_hip_a;
    Force         RF_hip_f;
    // Link 'RF_upperleg' :
    const InertiaMatrix& RF_upperleg_I;
    Velocity      RF_upperleg_v;
    Acceleration  RF_upperleg_a;
    Force         RF_upperleg_f;
    // Link 'RF_lowerleg' :
    const InertiaMatrix& RF_lowerleg_I;
    Velocity      RF_lowerleg_v;
    Acceleration  RF_lowerleg_a;
    Force         RF_lowerleg_f;
    // Link 'RH_hip' :
    const InertiaMatrix& RH_hip_I;
    Velocity      RH_hip_v;
    Acceleration  RH_hip_a;
    Force         RH_hip_f;
    // Link 'RH_upperleg' :
    const InertiaMatrix& RH_upperleg_I;
    Velocity      RH_upperleg_v;
    Acceleration  RH_upperleg_a;
    Force         RH_upperleg_f;
    // Link 'RH_lowerleg' :
    const InertiaMatrix& RH_lowerleg_I;
    Velocity      RH_lowerleg_v;
    Acceleration  RH_lowerleg_a;
    Force         RH_lowerleg_f;
    // Link 'LH_hip' :
    const InertiaMatrix& LH_hip_I;
    Velocity      LH_hip_v;
    Acceleration  LH_hip_a;
    Force         LH_hip_f;
    // Link 'LH_upperleg' :
    const InertiaMatrix& LH_upperleg_I;
    Velocity      LH_upperleg_v;
    Acceleration  LH_upperleg_a;
    Force         LH_upperleg_f;
    // Link 'LH_lowerleg' :
    const InertiaMatrix& LH_lowerleg_I;
    Velocity      LH_lowerleg_v;
    Acceleration  LH_lowerleg_a;
    Force         LH_lowerleg_f;
    // Link 'LF_hip' :
    const InertiaMatrix& LF_hip_I;
    Velocity      LF_hip_v;
    Acceleration  LF_hip_a;
    Force         LF_hip_f;
    // Link 'LF_upperleg' :
    const InertiaMatrix& LF_upperleg_I;
    Velocity      LF_upperleg_v;
    Acceleration  LF_upperleg_a;
    Force         LF_upperleg_f;
    // Link 'LF_lowerleg' :
    const InertiaMatrix& LF_lowerleg_I;
    Velocity      LF_lowerleg_v;
    Acceleration  LF_lowerleg_a;
    Force         LF_lowerleg_f;

    // The robot base
    const InertiaMatrix& torso_I;
    InertiaMatrix torso_Ic;
    Force         torso_f;
    // The composite inertia tensors
    InertiaMatrix RF_hip_Ic;
    InertiaMatrix RF_upperleg_Ic;
    const InertiaMatrix& RF_lowerleg_Ic;
    InertiaMatrix RH_hip_Ic;
    InertiaMatrix RH_upperleg_Ic;
    const InertiaMatrix& RH_lowerleg_Ic;
    InertiaMatrix LH_hip_Ic;
    InertiaMatrix LH_upperleg_Ic;
    const InertiaMatrix& LH_lowerleg_Ic;
    InertiaMatrix LF_hip_Ic;
    InertiaMatrix LF_upperleg_Ic;
    const InertiaMatrix& LF_lowerleg_Ic;

private:
    static const ExtForces zeroExtForces;
};

template <typename TRAIT>
inline void InverseDynamics<TRAIT>::setJointStatus(const JointState& q) const
{
    (xm->fr_RF_hip_X_fr_torso)(q);
    (xm->fr_RF_upperleg_X_fr_RF_hip)(q);
    (xm->fr_RF_lowerleg_X_fr_RF_upperleg)(q);
    (xm->fr_RH_hip_X_fr_torso)(q);
    (xm->fr_RH_upperleg_X_fr_RH_hip)(q);
    (xm->fr_RH_lowerleg_X_fr_RH_upperleg)(q);
    (xm->fr_LH_hip_X_fr_torso)(q);
    (xm->fr_LH_upperleg_X_fr_LH_hip)(q);
    (xm->fr_LH_lowerleg_X_fr_LH_upperleg)(q);
    (xm->fr_LF_hip_X_fr_torso)(q);
    (xm->fr_LF_upperleg_X_fr_LF_hip)(q);
    (xm->fr_LF_lowerleg_X_fr_LF_upperleg)(q);
}

template <typename TRAIT>
inline void InverseDynamics<TRAIT>::id(
    JointState& jForces, Acceleration& torso_a,
    const Acceleration& g, const Velocity& torso_v,
    const JointState& q, const JointState& qd, const JointState& qdd,
    const ExtForces& fext)
{
    setJointStatus(q);
    id(jForces, torso_a, g, torso_v,
       qd, qdd, fext);
}

template <typename TRAIT>
inline void InverseDynamics<TRAIT>::G_terms_fully_actuated(
    Force& baseWrench, JointState& jForces,
    const Acceleration& g, const JointState& q)
{
    setJointStatus(q);
    G_terms_fully_actuated(baseWrench, jForces, g);
}

template <typename TRAIT>
inline void InverseDynamics<TRAIT>::C_terms_fully_actuated(
    Force& baseWrench, JointState& jForces,
    const Velocity& torso_v, const JointState& q, const JointState& qd)
{
    setJointStatus(q);
    C_terms_fully_actuated(baseWrench, jForces, torso_v, qd);
}

template <typename TRAIT>
inline void InverseDynamics<TRAIT>::id_fully_actuated(
        Force& baseWrench, JointState& jForces,
        const Acceleration& g, const Velocity& torso_v, const Acceleration& baseAccel,
        const JointState& q, const JointState& qd, const JointState& qdd, const ExtForces& fext)
{
    setJointStatus(q);
    id_fully_actuated(baseWrench, jForces, g, torso_v,
        baseAccel, qd, qdd, fext);
}

}

typedef tpl::InverseDynamics<rbd::DoubleTrait> InverseDynamics;

}
}

}

#include "inverse_dynamics.impl.h"

#endif
