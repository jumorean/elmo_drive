#ifndef IIT_ROBOGEN__PEGASUS2_TRAITS_H_
#define IIT_ROBOGEN__PEGASUS2_TRAITS_H_

#include "declarations.h"
#include "transforms.h"
#include "inverse_dynamics.h"
#include "forward_dynamics.h"
#include "jsim.h"
#include "inertia_properties.h"
#include "jacobians.h"
#include <iit/rbd/traits/TraitSelector.h>


namespace iit {
namespace pegasus2 {

namespace tpl {

template <typename SCALAR>
struct Traits {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    typedef SCALAR S;

    typedef typename pegasus2::JointIdentifiers JointID;
    typedef typename pegasus2::LinkIdentifiers  LinkID;
    typedef typename iit::rbd::tpl::TraitSelector<SCALAR>::Trait Trait;

    typedef typename pegasus2::tpl::JointState<SCALAR> JointState;



    typedef typename pegasus2::tpl::HomogeneousTransforms<Trait> HomogeneousTransforms;
    typedef typename pegasus2::tpl::MotionTransforms<Trait> MotionTransforms;
    typedef typename pegasus2::tpl::ForceTransforms<Trait> ForceTransforms;
    typedef typename pegasus2::tpl::Jacobians<Trait> Jacobians;

    typedef typename iit::pegasus2::dyn::tpl::InertiaProperties<Trait> InertiaProperties;
    typedef typename iit::pegasus2::dyn::tpl::ForwardDynamics<Trait> FwdDynEngine;
    typedef typename iit::pegasus2::dyn::tpl::InverseDynamics<Trait> InvDynEngine;
    typedef typename iit::pegasus2::dyn::tpl::JSIM<Trait> JSIM;

    static const int joints_count = pegasus2::jointsCount;
    static const int links_count  = pegasus2::linksCount;
    static const bool floating_base = true;

    static inline const JointID* orderedJointIDs();
    static inline const LinkID*  orderedLinkIDs();
};

template <typename SCALAR>
inline const typename Traits<SCALAR>::JointID*  Traits<SCALAR>::orderedJointIDs() {
    return pegasus2::orderedJointIDs;
}
template <typename SCALAR>
inline const typename Traits<SCALAR>::LinkID*  Traits<SCALAR>::orderedLinkIDs() {
    return pegasus2::orderedLinkIDs;
}

}

typedef tpl::Traits<double> Traits; // default instantiation - backward compatibility...

}
}

#endif
