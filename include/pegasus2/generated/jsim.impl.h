

//Implementation of default constructor
template <typename TRAIT>
iit::pegasus2::dyn::tpl::JSIM<TRAIT>::JSIM(IProperties& inertiaProperties, FTransforms& forceTransforms) :
    linkInertias(inertiaProperties),
    frcTransf( &forceTransforms ),
    RF_lowerleg_Ic(linkInertias.getTensor_RF_lowerleg()),
    RH_lowerleg_Ic(linkInertias.getTensor_RH_lowerleg()),
    LH_lowerleg_Ic(linkInertias.getTensor_LH_lowerleg()),
    LF_lowerleg_Ic(linkInertias.getTensor_LF_lowerleg())
{
    //Initialize the matrix itself
    this->setZero();
}

#define DATA tpl::JSIM<TRAIT>::operator()
#define Fcol(j) (tpl::JSIM<TRAIT>:: template block<6,1>(0,(j)+6))
#define F(i,j) DATA((i),(j)+6)

template <typename TRAIT>
const typename iit::pegasus2::dyn::tpl::JSIM<TRAIT>& iit::pegasus2::dyn::tpl::JSIM<TRAIT>::update(const JointState& state) {

    // Precomputes only once the coordinate transforms:
    frcTransf -> fr_LF_upperleg_X_fr_LF_lowerleg(state);
    frcTransf -> fr_LF_hip_X_fr_LF_upperleg(state);
    frcTransf -> fr_torso_X_fr_LF_hip(state);
    frcTransf -> fr_LH_upperleg_X_fr_LH_lowerleg(state);
    frcTransf -> fr_LH_hip_X_fr_LH_upperleg(state);
    frcTransf -> fr_torso_X_fr_LH_hip(state);
    frcTransf -> fr_RH_upperleg_X_fr_RH_lowerleg(state);
    frcTransf -> fr_RH_hip_X_fr_RH_upperleg(state);
    frcTransf -> fr_torso_X_fr_RH_hip(state);
    frcTransf -> fr_RF_upperleg_X_fr_RF_lowerleg(state);
    frcTransf -> fr_RF_hip_X_fr_RF_upperleg(state);
    frcTransf -> fr_torso_X_fr_RF_hip(state);

    // Initializes the composite inertia tensors
    torso_Ic = linkInertias.getTensor_torso();
    RF_hip_Ic = linkInertias.getTensor_RF_hip();
    RF_upperleg_Ic = linkInertias.getTensor_RF_upperleg();
    RH_hip_Ic = linkInertias.getTensor_RH_hip();
    RH_upperleg_Ic = linkInertias.getTensor_RH_upperleg();
    LH_hip_Ic = linkInertias.getTensor_LH_hip();
    LH_upperleg_Ic = linkInertias.getTensor_LH_upperleg();
    LF_hip_Ic = linkInertias.getTensor_LF_hip();
    LF_upperleg_Ic = linkInertias.getTensor_LF_upperleg();

    // "Bottom-up" loop to update the inertia-composite property of each link, for the current configuration

    // Link LF_lowerleg:
    iit::rbd::transformInertia<Scalar>(LF_lowerleg_Ic, frcTransf -> fr_LF_upperleg_X_fr_LF_lowerleg, Ic_spare);
    LF_upperleg_Ic += Ic_spare;

    Fcol(LF_KFE_JOINT) = LF_lowerleg_Ic.col(iit::rbd::AZ);
    DATA(LF_KFE_JOINT+6, LF_KFE_JOINT+6) = Fcol(LF_KFE_JOINT)(iit::rbd::AZ);

    Fcol(LF_KFE_JOINT) = frcTransf -> fr_LF_upperleg_X_fr_LF_lowerleg * Fcol(LF_KFE_JOINT);
    DATA(LF_KFE_JOINT+6, LF_HFE_JOINT+6) = F(iit::rbd::AZ,LF_KFE_JOINT);
    DATA(LF_HFE_JOINT+6, LF_KFE_JOINT+6) = DATA(LF_KFE_JOINT+6, LF_HFE_JOINT+6);
    Fcol(LF_KFE_JOINT) = frcTransf -> fr_LF_hip_X_fr_LF_upperleg * Fcol(LF_KFE_JOINT);
    DATA(LF_KFE_JOINT+6, LF_HAA_JOINT+6) = F(iit::rbd::AZ,LF_KFE_JOINT);
    DATA(LF_HAA_JOINT+6, LF_KFE_JOINT+6) = DATA(LF_KFE_JOINT+6, LF_HAA_JOINT+6);
    Fcol(LF_KFE_JOINT) = frcTransf -> fr_torso_X_fr_LF_hip * Fcol(LF_KFE_JOINT);

    // Link LF_upperleg:
    iit::rbd::transformInertia<Scalar>(LF_upperleg_Ic, frcTransf -> fr_LF_hip_X_fr_LF_upperleg, Ic_spare);
    LF_hip_Ic += Ic_spare;

    Fcol(LF_HFE_JOINT) = LF_upperleg_Ic.col(iit::rbd::AZ);
    DATA(LF_HFE_JOINT+6, LF_HFE_JOINT+6) = Fcol(LF_HFE_JOINT)(iit::rbd::AZ);

    Fcol(LF_HFE_JOINT) = frcTransf -> fr_LF_hip_X_fr_LF_upperleg * Fcol(LF_HFE_JOINT);
    DATA(LF_HFE_JOINT+6, LF_HAA_JOINT+6) = F(iit::rbd::AZ,LF_HFE_JOINT);
    DATA(LF_HAA_JOINT+6, LF_HFE_JOINT+6) = DATA(LF_HFE_JOINT+6, LF_HAA_JOINT+6);
    Fcol(LF_HFE_JOINT) = frcTransf -> fr_torso_X_fr_LF_hip * Fcol(LF_HFE_JOINT);

    // Link LF_hip:
    iit::rbd::transformInertia<Scalar>(LF_hip_Ic, frcTransf -> fr_torso_X_fr_LF_hip, Ic_spare);
    torso_Ic += Ic_spare;

    Fcol(LF_HAA_JOINT) = LF_hip_Ic.col(iit::rbd::AZ);
    DATA(LF_HAA_JOINT+6, LF_HAA_JOINT+6) = Fcol(LF_HAA_JOINT)(iit::rbd::AZ);

    Fcol(LF_HAA_JOINT) = frcTransf -> fr_torso_X_fr_LF_hip * Fcol(LF_HAA_JOINT);

    // Link LH_lowerleg:
    iit::rbd::transformInertia<Scalar>(LH_lowerleg_Ic, frcTransf -> fr_LH_upperleg_X_fr_LH_lowerleg, Ic_spare);
    LH_upperleg_Ic += Ic_spare;

    Fcol(LH_KFE_JOINT) = LH_lowerleg_Ic.col(iit::rbd::AZ);
    DATA(LH_KFE_JOINT+6, LH_KFE_JOINT+6) = Fcol(LH_KFE_JOINT)(iit::rbd::AZ);

    Fcol(LH_KFE_JOINT) = frcTransf -> fr_LH_upperleg_X_fr_LH_lowerleg * Fcol(LH_KFE_JOINT);
    DATA(LH_KFE_JOINT+6, LH_HFE_JOINT+6) = F(iit::rbd::AZ,LH_KFE_JOINT);
    DATA(LH_HFE_JOINT+6, LH_KFE_JOINT+6) = DATA(LH_KFE_JOINT+6, LH_HFE_JOINT+6);
    Fcol(LH_KFE_JOINT) = frcTransf -> fr_LH_hip_X_fr_LH_upperleg * Fcol(LH_KFE_JOINT);
    DATA(LH_KFE_JOINT+6, LH_HAA_JOINT+6) = F(iit::rbd::AZ,LH_KFE_JOINT);
    DATA(LH_HAA_JOINT+6, LH_KFE_JOINT+6) = DATA(LH_KFE_JOINT+6, LH_HAA_JOINT+6);
    Fcol(LH_KFE_JOINT) = frcTransf -> fr_torso_X_fr_LH_hip * Fcol(LH_KFE_JOINT);

    // Link LH_upperleg:
    iit::rbd::transformInertia<Scalar>(LH_upperleg_Ic, frcTransf -> fr_LH_hip_X_fr_LH_upperleg, Ic_spare);
    LH_hip_Ic += Ic_spare;

    Fcol(LH_HFE_JOINT) = LH_upperleg_Ic.col(iit::rbd::AZ);
    DATA(LH_HFE_JOINT+6, LH_HFE_JOINT+6) = Fcol(LH_HFE_JOINT)(iit::rbd::AZ);

    Fcol(LH_HFE_JOINT) = frcTransf -> fr_LH_hip_X_fr_LH_upperleg * Fcol(LH_HFE_JOINT);
    DATA(LH_HFE_JOINT+6, LH_HAA_JOINT+6) = F(iit::rbd::AZ,LH_HFE_JOINT);
    DATA(LH_HAA_JOINT+6, LH_HFE_JOINT+6) = DATA(LH_HFE_JOINT+6, LH_HAA_JOINT+6);
    Fcol(LH_HFE_JOINT) = frcTransf -> fr_torso_X_fr_LH_hip * Fcol(LH_HFE_JOINT);

    // Link LH_hip:
    iit::rbd::transformInertia<Scalar>(LH_hip_Ic, frcTransf -> fr_torso_X_fr_LH_hip, Ic_spare);
    torso_Ic += Ic_spare;

    Fcol(LH_HAA_JOINT) = LH_hip_Ic.col(iit::rbd::AZ);
    DATA(LH_HAA_JOINT+6, LH_HAA_JOINT+6) = Fcol(LH_HAA_JOINT)(iit::rbd::AZ);

    Fcol(LH_HAA_JOINT) = frcTransf -> fr_torso_X_fr_LH_hip * Fcol(LH_HAA_JOINT);

    // Link RH_lowerleg:
    iit::rbd::transformInertia<Scalar>(RH_lowerleg_Ic, frcTransf -> fr_RH_upperleg_X_fr_RH_lowerleg, Ic_spare);
    RH_upperleg_Ic += Ic_spare;

    Fcol(RH_KFE_JOINT) = RH_lowerleg_Ic.col(iit::rbd::AZ);
    DATA(RH_KFE_JOINT+6, RH_KFE_JOINT+6) = Fcol(RH_KFE_JOINT)(iit::rbd::AZ);

    Fcol(RH_KFE_JOINT) = frcTransf -> fr_RH_upperleg_X_fr_RH_lowerleg * Fcol(RH_KFE_JOINT);
    DATA(RH_KFE_JOINT+6, RH_HFE_JOINT+6) = F(iit::rbd::AZ,RH_KFE_JOINT);
    DATA(RH_HFE_JOINT+6, RH_KFE_JOINT+6) = DATA(RH_KFE_JOINT+6, RH_HFE_JOINT+6);
    Fcol(RH_KFE_JOINT) = frcTransf -> fr_RH_hip_X_fr_RH_upperleg * Fcol(RH_KFE_JOINT);
    DATA(RH_KFE_JOINT+6, RH_HAA_JOINT+6) = F(iit::rbd::AZ,RH_KFE_JOINT);
    DATA(RH_HAA_JOINT+6, RH_KFE_JOINT+6) = DATA(RH_KFE_JOINT+6, RH_HAA_JOINT+6);
    Fcol(RH_KFE_JOINT) = frcTransf -> fr_torso_X_fr_RH_hip * Fcol(RH_KFE_JOINT);

    // Link RH_upperleg:
    iit::rbd::transformInertia<Scalar>(RH_upperleg_Ic, frcTransf -> fr_RH_hip_X_fr_RH_upperleg, Ic_spare);
    RH_hip_Ic += Ic_spare;

    Fcol(RH_HFE_JOINT) = RH_upperleg_Ic.col(iit::rbd::AZ);
    DATA(RH_HFE_JOINT+6, RH_HFE_JOINT+6) = Fcol(RH_HFE_JOINT)(iit::rbd::AZ);

    Fcol(RH_HFE_JOINT) = frcTransf -> fr_RH_hip_X_fr_RH_upperleg * Fcol(RH_HFE_JOINT);
    DATA(RH_HFE_JOINT+6, RH_HAA_JOINT+6) = F(iit::rbd::AZ,RH_HFE_JOINT);
    DATA(RH_HAA_JOINT+6, RH_HFE_JOINT+6) = DATA(RH_HFE_JOINT+6, RH_HAA_JOINT+6);
    Fcol(RH_HFE_JOINT) = frcTransf -> fr_torso_X_fr_RH_hip * Fcol(RH_HFE_JOINT);

    // Link RH_hip:
    iit::rbd::transformInertia<Scalar>(RH_hip_Ic, frcTransf -> fr_torso_X_fr_RH_hip, Ic_spare);
    torso_Ic += Ic_spare;

    Fcol(RH_HAA_JOINT) = RH_hip_Ic.col(iit::rbd::AZ);
    DATA(RH_HAA_JOINT+6, RH_HAA_JOINT+6) = Fcol(RH_HAA_JOINT)(iit::rbd::AZ);

    Fcol(RH_HAA_JOINT) = frcTransf -> fr_torso_X_fr_RH_hip * Fcol(RH_HAA_JOINT);

    // Link RF_lowerleg:
    iit::rbd::transformInertia<Scalar>(RF_lowerleg_Ic, frcTransf -> fr_RF_upperleg_X_fr_RF_lowerleg, Ic_spare);
    RF_upperleg_Ic += Ic_spare;

    Fcol(RF_KFE_JOINT) = RF_lowerleg_Ic.col(iit::rbd::AZ);
    DATA(RF_KFE_JOINT+6, RF_KFE_JOINT+6) = Fcol(RF_KFE_JOINT)(iit::rbd::AZ);

    Fcol(RF_KFE_JOINT) = frcTransf -> fr_RF_upperleg_X_fr_RF_lowerleg * Fcol(RF_KFE_JOINT);
    DATA(RF_KFE_JOINT+6, RF_HFE_JOINT+6) = F(iit::rbd::AZ,RF_KFE_JOINT);
    DATA(RF_HFE_JOINT+6, RF_KFE_JOINT+6) = DATA(RF_KFE_JOINT+6, RF_HFE_JOINT+6);
    Fcol(RF_KFE_JOINT) = frcTransf -> fr_RF_hip_X_fr_RF_upperleg * Fcol(RF_KFE_JOINT);
    DATA(RF_KFE_JOINT+6, RF_HAA_JOINT+6) = F(iit::rbd::AZ,RF_KFE_JOINT);
    DATA(RF_HAA_JOINT+6, RF_KFE_JOINT+6) = DATA(RF_KFE_JOINT+6, RF_HAA_JOINT+6);
    Fcol(RF_KFE_JOINT) = frcTransf -> fr_torso_X_fr_RF_hip * Fcol(RF_KFE_JOINT);

    // Link RF_upperleg:
    iit::rbd::transformInertia<Scalar>(RF_upperleg_Ic, frcTransf -> fr_RF_hip_X_fr_RF_upperleg, Ic_spare);
    RF_hip_Ic += Ic_spare;

    Fcol(RF_HFE_JOINT) = RF_upperleg_Ic.col(iit::rbd::AZ);
    DATA(RF_HFE_JOINT+6, RF_HFE_JOINT+6) = Fcol(RF_HFE_JOINT)(iit::rbd::AZ);

    Fcol(RF_HFE_JOINT) = frcTransf -> fr_RF_hip_X_fr_RF_upperleg * Fcol(RF_HFE_JOINT);
    DATA(RF_HFE_JOINT+6, RF_HAA_JOINT+6) = F(iit::rbd::AZ,RF_HFE_JOINT);
    DATA(RF_HAA_JOINT+6, RF_HFE_JOINT+6) = DATA(RF_HFE_JOINT+6, RF_HAA_JOINT+6);
    Fcol(RF_HFE_JOINT) = frcTransf -> fr_torso_X_fr_RF_hip * Fcol(RF_HFE_JOINT);

    // Link RF_hip:
    iit::rbd::transformInertia<Scalar>(RF_hip_Ic, frcTransf -> fr_torso_X_fr_RF_hip, Ic_spare);
    torso_Ic += Ic_spare;

    Fcol(RF_HAA_JOINT) = RF_hip_Ic.col(iit::rbd::AZ);
    DATA(RF_HAA_JOINT+6, RF_HAA_JOINT+6) = Fcol(RF_HAA_JOINT)(iit::rbd::AZ);

    Fcol(RF_HAA_JOINT) = frcTransf -> fr_torso_X_fr_RF_hip * Fcol(RF_HAA_JOINT);

    // Copies the upper-right block into the lower-left block, after transposing
    JSIM<TRAIT>:: template block<12, 6>(6,0) = (JSIM<TRAIT>:: template block<6, 12>(0,6)).transpose();
    // The composite-inertia of the whole robot is the upper-left quadrant of the JSIM
    JSIM<TRAIT>:: template block<6,6>(0,0) = torso_Ic;
    return *this;
}

#undef DATA
#undef F

template <typename TRAIT>
void iit::pegasus2::dyn::tpl::JSIM<TRAIT>::computeL() {
    L = this -> template triangularView<Eigen::Lower>();
    // Joint LF_KFE_joint, index 11 :
    L(11, 11) = std::sqrt(L(11, 11));
    L(11, 10) = L(11, 10) / L(11, 11);
    L(11, 9) = L(11, 9) / L(11, 11);
    L(10, 10) = L(10, 10) - L(11, 10) * L(11, 10);
    L(10, 9) = L(10, 9) - L(11, 10) * L(11, 9);
    L(9, 9) = L(9, 9) - L(11, 9) * L(11, 9);
    
    // Joint LF_HFE_joint, index 10 :
    L(10, 10) = std::sqrt(L(10, 10));
    L(10, 9) = L(10, 9) / L(10, 10);
    L(9, 9) = L(9, 9) - L(10, 9) * L(10, 9);
    
    // Joint LF_HAA_joint, index 9 :
    L(9, 9) = std::sqrt(L(9, 9));
    
    // Joint LH_KFE_joint, index 8 :
    L(8, 8) = std::sqrt(L(8, 8));
    L(8, 7) = L(8, 7) / L(8, 8);
    L(8, 6) = L(8, 6) / L(8, 8);
    L(7, 7) = L(7, 7) - L(8, 7) * L(8, 7);
    L(7, 6) = L(7, 6) - L(8, 7) * L(8, 6);
    L(6, 6) = L(6, 6) - L(8, 6) * L(8, 6);
    
    // Joint LH_HFE_joint, index 7 :
    L(7, 7) = std::sqrt(L(7, 7));
    L(7, 6) = L(7, 6) / L(7, 7);
    L(6, 6) = L(6, 6) - L(7, 6) * L(7, 6);
    
    // Joint LH_HAA_joint, index 6 :
    L(6, 6) = std::sqrt(L(6, 6));
    
    // Joint RH_KFE_joint, index 5 :
    L(5, 5) = std::sqrt(L(5, 5));
    L(5, 4) = L(5, 4) / L(5, 5);
    L(5, 3) = L(5, 3) / L(5, 5);
    L(4, 4) = L(4, 4) - L(5, 4) * L(5, 4);
    L(4, 3) = L(4, 3) - L(5, 4) * L(5, 3);
    L(3, 3) = L(3, 3) - L(5, 3) * L(5, 3);
    
    // Joint RH_HFE_joint, index 4 :
    L(4, 4) = std::sqrt(L(4, 4));
    L(4, 3) = L(4, 3) / L(4, 4);
    L(3, 3) = L(3, 3) - L(4, 3) * L(4, 3);
    
    // Joint RH_HAA_joint, index 3 :
    L(3, 3) = std::sqrt(L(3, 3));
    
    // Joint RF_KFE_joint, index 2 :
    L(2, 2) = std::sqrt(L(2, 2));
    L(2, 1) = L(2, 1) / L(2, 2);
    L(2, 0) = L(2, 0) / L(2, 2);
    L(1, 1) = L(1, 1) - L(2, 1) * L(2, 1);
    L(1, 0) = L(1, 0) - L(2, 1) * L(2, 0);
    L(0, 0) = L(0, 0) - L(2, 0) * L(2, 0);
    
    // Joint RF_HFE_joint, index 1 :
    L(1, 1) = std::sqrt(L(1, 1));
    L(1, 0) = L(1, 0) / L(1, 1);
    L(0, 0) = L(0, 0) - L(1, 0) * L(1, 0);
    
    // Joint RF_HAA_joint, index 0 :
    L(0, 0) = std::sqrt(L(0, 0));
    
}

template <typename TRAIT>
void iit::pegasus2::dyn::tpl::JSIM<TRAIT>::computeInverse() {
    computeLInverse();

    inverse(0, 0) =  + (Linv(0, 0) * Linv(0, 0));
    inverse(1, 1) =  + (Linv(1, 0) * Linv(1, 0)) + (Linv(1, 1) * Linv(1, 1));
    inverse(1, 0) =  + (Linv(1, 0) * Linv(0, 0));
    inverse(0, 1) = inverse(1, 0);
    inverse(2, 2) =  + (Linv(2, 0) * Linv(2, 0)) + (Linv(2, 1) * Linv(2, 1)) + (Linv(2, 2) * Linv(2, 2));
    inverse(2, 1) =  + (Linv(2, 0) * Linv(1, 0)) + (Linv(2, 1) * Linv(1, 1));
    inverse(1, 2) = inverse(2, 1);
    inverse(2, 0) =  + (Linv(2, 0) * Linv(0, 0));
    inverse(0, 2) = inverse(2, 0);
    inverse(3, 3) =  + (Linv(3, 3) * Linv(3, 3));
    inverse(4, 4) =  + (Linv(4, 3) * Linv(4, 3)) + (Linv(4, 4) * Linv(4, 4));
    inverse(4, 3) =  + (Linv(4, 3) * Linv(3, 3));
    inverse(3, 4) = inverse(4, 3);
    inverse(5, 5) =  + (Linv(5, 3) * Linv(5, 3)) + (Linv(5, 4) * Linv(5, 4)) + (Linv(5, 5) * Linv(5, 5));
    inverse(5, 4) =  + (Linv(5, 3) * Linv(4, 3)) + (Linv(5, 4) * Linv(4, 4));
    inverse(4, 5) = inverse(5, 4);
    inverse(5, 3) =  + (Linv(5, 3) * Linv(3, 3));
    inverse(3, 5) = inverse(5, 3);
    inverse(6, 6) =  + (Linv(6, 6) * Linv(6, 6));
    inverse(7, 7) =  + (Linv(7, 6) * Linv(7, 6)) + (Linv(7, 7) * Linv(7, 7));
    inverse(7, 6) =  + (Linv(7, 6) * Linv(6, 6));
    inverse(6, 7) = inverse(7, 6);
    inverse(8, 8) =  + (Linv(8, 6) * Linv(8, 6)) + (Linv(8, 7) * Linv(8, 7)) + (Linv(8, 8) * Linv(8, 8));
    inverse(8, 7) =  + (Linv(8, 6) * Linv(7, 6)) + (Linv(8, 7) * Linv(7, 7));
    inverse(7, 8) = inverse(8, 7);
    inverse(8, 6) =  + (Linv(8, 6) * Linv(6, 6));
    inverse(6, 8) = inverse(8, 6);
    inverse(9, 9) =  + (Linv(9, 9) * Linv(9, 9));
    inverse(10, 10) =  + (Linv(10, 9) * Linv(10, 9)) + (Linv(10, 10) * Linv(10, 10));
    inverse(10, 9) =  + (Linv(10, 9) * Linv(9, 9));
    inverse(9, 10) = inverse(10, 9);
    inverse(11, 11) =  + (Linv(11, 9) * Linv(11, 9)) + (Linv(11, 10) * Linv(11, 10)) + (Linv(11, 11) * Linv(11, 11));
    inverse(11, 10) =  + (Linv(11, 9) * Linv(10, 9)) + (Linv(11, 10) * Linv(10, 10));
    inverse(10, 11) = inverse(11, 10);
    inverse(11, 9) =  + (Linv(11, 9) * Linv(9, 9));
    inverse(9, 11) = inverse(11, 9);
}

template <typename TRAIT>
void iit::pegasus2::dyn::tpl::JSIM<TRAIT>::computeLInverse() {
    //assumes L has been computed already
    Linv(0, 0) = 1 / L(0, 0);
    Linv(1, 1) = 1 / L(1, 1);
    Linv(2, 2) = 1 / L(2, 2);
    Linv(3, 3) = 1 / L(3, 3);
    Linv(4, 4) = 1 / L(4, 4);
    Linv(5, 5) = 1 / L(5, 5);
    Linv(6, 6) = 1 / L(6, 6);
    Linv(7, 7) = 1 / L(7, 7);
    Linv(8, 8) = 1 / L(8, 8);
    Linv(9, 9) = 1 / L(9, 9);
    Linv(10, 10) = 1 / L(10, 10);
    Linv(11, 11) = 1 / L(11, 11);
    Linv(1, 0) = - Linv(0, 0) * ((Linv(1, 1) * L(1, 0)) + 0);
    Linv(2, 1) = - Linv(1, 1) * ((Linv(2, 2) * L(2, 1)) + 0);
    Linv(2, 0) = - Linv(0, 0) * ((Linv(2, 1) * L(1, 0)) + (Linv(2, 2) * L(2, 0)) + 0);
    Linv(4, 3) = - Linv(3, 3) * ((Linv(4, 4) * L(4, 3)) + 0);
    Linv(5, 4) = - Linv(4, 4) * ((Linv(5, 5) * L(5, 4)) + 0);
    Linv(5, 3) = - Linv(3, 3) * ((Linv(5, 4) * L(4, 3)) + (Linv(5, 5) * L(5, 3)) + 0);
    Linv(7, 6) = - Linv(6, 6) * ((Linv(7, 7) * L(7, 6)) + 0);
    Linv(8, 7) = - Linv(7, 7) * ((Linv(8, 8) * L(8, 7)) + 0);
    Linv(8, 6) = - Linv(6, 6) * ((Linv(8, 7) * L(7, 6)) + (Linv(8, 8) * L(8, 6)) + 0);
    Linv(10, 9) = - Linv(9, 9) * ((Linv(10, 10) * L(10, 9)) + 0);
    Linv(11, 10) = - Linv(10, 10) * ((Linv(11, 11) * L(11, 10)) + 0);
    Linv(11, 9) = - Linv(9, 9) * ((Linv(11, 10) * L(10, 9)) + (Linv(11, 11) * L(11, 9)) + 0);
}

