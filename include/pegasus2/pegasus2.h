
//
// Created by cda on 2020/8/18.
//




#ifndef _PEGASUS_2_H_____
#define _PEGASUS_2_H_____



#include <Eigen/Core>
#include <Eigen/StdVector>
#include "generated/declarations.h"
#include "generated/jsim.h"
#include "generated/jacobians.h"
#include "generated/traits.h"
#include "generated/forward_dynamics.h"
#include "generated/inertia_properties.h"
#include "generated/inverse_dynamics.h"
#include "generated/transforms.h"
#include "generated/link_data_map.h"


#include <ct/rbd/rbd.h>

// define namespace and base
#define ROBCOGEN_NS pegasus2
#define TARGET_NS pegasus2
// define the links
#define CT_BASE fr_torso
#define CT_L0 fr_RF_hip
#define CT_L1 fr_RF_upperleg
#define CT_L2 fr_RF_lowerleg

#define CT_L3 fr_RH_hip
#define CT_L4 fr_RH_upperleg
#define CT_L5 fr_RH_lowerleg

#define CT_L6 fr_LH_hip
#define CT_L7 fr_LH_upperleg
#define CT_L8 fr_LH_lowerleg

#define CT_L9 fr_LF_hip
#define CT_L10 fr_LF_upperleg
#define CT_L11 fr_LF_lowerleg

// define single end effector (could also be multiple)
#define CT_N_EE 4
#define CT_EE0 fr_RF_foot
#define CT_EE0_IS_ON_LINK 3   
#define CT_EE0_FIRST_JOINT 0  
#define CT_EE0_LAST_JOINT 2  

#define CT_EE1 fr_RH_foot
#define CT_EE1_IS_ON_LINK 6
#define CT_EE1_FIRST_JOINT 3
#define CT_EE1_LAST_JOINT 5

#define CT_EE2 fr_LH_foot
#define CT_EE2_IS_ON_LINK 9
#define CT_EE2_FIRST_JOINT 6
#define CT_EE2_LAST_JOINT 8

#define CT_EE3 fr_LF_foot
#define CT_EE3_IS_ON_LINK 12
#define CT_EE3_FIRST_JOINT 9
#define CT_EE3_LAST_JOINT 11

#include <ct/rbd/robot/robcogen/robcogenHelpers.h>
#endif // _PEGASUS_2_H_____
