#ifndef IIT_ROBOT_PEGASUS2_DECLARATIONS_H_
#define IIT_ROBOT_PEGASUS2_DECLARATIONS_H_

#include <iit/rbd/rbd.h>

namespace iit {
namespace pegasus2 {

static const int JointSpaceDimension = 12;
static const int jointsCount = 12;
/** The total number of rigid bodies of this robot, including the base */
static const int linksCount  = 13;

namespace tpl {
template <typename SCALAR>
using Column12d = iit::rbd::PlainMatrix<SCALAR, 12, 1>;

template <typename SCALAR>
using JointState = Column12d<SCALAR>;
}

using Column12d = tpl::Column12d<double>;
typedef Column12d JointState;

enum JointIdentifiers {
    RF_HAA_JOINT = 0
    , RF_HFE_JOINT
    , RF_KFE_JOINT
    , RH_HAA_JOINT
    , RH_HFE_JOINT
    , RH_KFE_JOINT
    , LH_HAA_JOINT
    , LH_HFE_JOINT
    , LH_KFE_JOINT
    , LF_HAA_JOINT
    , LF_HFE_JOINT
    , LF_KFE_JOINT
};

enum LinkIdentifiers {
    TORSO = 0
    , RF_HIP
    , RF_UPPERLEG
    , RF_LOWERLEG
    , RH_HIP
    , RH_UPPERLEG
    , RH_LOWERLEG
    , LH_HIP
    , LH_UPPERLEG
    , LH_LOWERLEG
    , LF_HIP
    , LF_UPPERLEG
    , LF_LOWERLEG
};

static const JointIdentifiers orderedJointIDs[jointsCount] =
    {RF_HAA_JOINT,RF_HFE_JOINT,RF_KFE_JOINT,RH_HAA_JOINT,RH_HFE_JOINT,RH_KFE_JOINT,LH_HAA_JOINT,LH_HFE_JOINT,LH_KFE_JOINT,LF_HAA_JOINT,LF_HFE_JOINT,LF_KFE_JOINT};

static const LinkIdentifiers orderedLinkIDs[linksCount] =
    {TORSO,RF_HIP,RF_UPPERLEG,RF_LOWERLEG,RH_HIP,RH_UPPERLEG,RH_LOWERLEG,LH_HIP,LH_UPPERLEG,LH_LOWERLEG,LF_HIP,LF_UPPERLEG,LF_LOWERLEG};

}
}
#endif
