template <typename TRAIT>
iit::pegasus2::dyn::tpl::InertiaProperties<TRAIT>::InertiaProperties()
{
    com_torso = iit::rbd::Vector3d(-0.0010516929,-2.821861E-4,0.56233364).cast<Scalar>();
    tensor_torso.fill(
        Scalar(18.301807),
        com_torso,
        rbd::Utils::buildInertiaTensor(
                Scalar(0.04337955),
                Scalar(0.099749476),
                Scalar(0.11043683),
                Scalar(-1.18898846E-7),
                Scalar(1.6083291E-5),
                Scalar(1.3488655E-7)) );

    com_RF_hip = iit::rbd::Vector3d(3.0E-5,-0.0251,-0.0048).cast<Scalar>();
    tensor_RF_hip.fill(
        Scalar(1.45622),
        com_RF_hip,
        rbd::Utils::buildInertiaTensor(
                Scalar(0.0036745183),
                Scalar(0.0018164385),
                Scalar(0.002656237),
                Scalar(1.93852E-6),
                Scalar(-1.2978E-7),
                Scalar(-3.40281E-6)) );

    com_RF_upperleg = iit::rbd::Vector3d(0.15777,3.1E-4,0.06355).cast<Scalar>();
    tensor_RF_upperleg.fill(
        Scalar(1.72052),
        com_RF_upperleg,
        rbd::Utils::buildInertiaTensor(
                Scalar(0.010112616),
                Scalar(0.10585742),
                Scalar(0.09674735),
                Scalar(3.439098E-5),
                Scalar(0.008956144),
                Scalar(3.26932E-5)) );

    com_RF_lowerleg = iit::rbd::Vector3d(0.19252,0.00186,-0.00113).cast<Scalar>();
    tensor_RF_lowerleg.fill(
        Scalar(0.35123),
        com_RF_lowerleg,
        rbd::Utils::buildInertiaTensor(
                Scalar(1.07957E-4),
                Scalar(0.021312878),
                Scalar(0.021345625),
                Scalar(1.4509186E-4),
                Scalar(-1.12179E-6),
                Scalar(-3.231E-8)) );

    com_RH_hip = iit::rbd::Vector3d(3.0E-5,-0.0251,-0.0048).cast<Scalar>();
    tensor_RH_hip.fill(
        Scalar(1.45622),
        com_RH_hip,
        rbd::Utils::buildInertiaTensor(
                Scalar(0.0036745183),
                Scalar(0.0018164385),
                Scalar(0.002656237),
                Scalar(1.93852E-6),
                Scalar(-1.2978E-7),
                Scalar(-3.40281E-6)) );

    com_RH_upperleg = iit::rbd::Vector3d(0.15777,3.1E-4,0.06355).cast<Scalar>();
    tensor_RH_upperleg.fill(
        Scalar(1.72052),
        com_RH_upperleg,
        rbd::Utils::buildInertiaTensor(
                Scalar(0.010112616),
                Scalar(0.10585742),
                Scalar(0.09674735),
                Scalar(3.439098E-5),
                Scalar(0.008956144),
                Scalar(3.26932E-5)) );

    com_RH_lowerleg = iit::rbd::Vector3d(0.19252,0.00186,0.00113).cast<Scalar>();
    tensor_RH_lowerleg.fill(
        Scalar(0.35123),
        com_RH_lowerleg,
        rbd::Utils::buildInertiaTensor(
                Scalar(1.07957E-4),
                Scalar(0.021312878),
                Scalar(0.021345625),
                Scalar(1.4509186E-4),
                Scalar(1.12179E-6),
                Scalar(3.231E-8)) );

    com_LH_hip = iit::rbd::Vector3d(3.0E-5,0.0251,-0.0048).cast<Scalar>();
    tensor_LH_hip.fill(
        Scalar(1.45622),
        com_LH_hip,
        rbd::Utils::buildInertiaTensor(
                Scalar(0.0036745183),
                Scalar(0.0018164385),
                Scalar(0.002656237),
                Scalar(-1.93852E-6),
                Scalar(-1.2978E-7),
                Scalar(3.40281E-6)) );

    com_LH_upperleg = iit::rbd::Vector3d(0.15777,3.1E-4,-0.06355).cast<Scalar>();
    tensor_LH_upperleg.fill(
        Scalar(1.72052),
        com_LH_upperleg,
        rbd::Utils::buildInertiaTensor(
                Scalar(0.010112616),
                Scalar(0.10585742),
                Scalar(0.09674735),
                Scalar(3.439098E-5),
                Scalar(-0.008956144),
                Scalar(-3.26932E-5)) );

    com_LH_lowerleg = iit::rbd::Vector3d(0.19252,0.00186,-0.00113).cast<Scalar>();
    tensor_LH_lowerleg.fill(
        Scalar(0.35123),
        com_LH_lowerleg,
        rbd::Utils::buildInertiaTensor(
                Scalar(1.07957E-4),
                Scalar(0.021312878),
                Scalar(0.021345625),
                Scalar(1.4509186E-4),
                Scalar(-1.12179E-6),
                Scalar(-3.231E-8)) );

    com_LF_hip = iit::rbd::Vector3d(3.0E-5,0.0251,-0.0048).cast<Scalar>();
    tensor_LF_hip.fill(
        Scalar(1.45622),
        com_LF_hip,
        rbd::Utils::buildInertiaTensor(
                Scalar(0.0036745183),
                Scalar(0.0018164385),
                Scalar(0.002656237),
                Scalar(-1.93852E-6),
                Scalar(-1.2978E-7),
                Scalar(3.40281E-6)) );

    com_LF_upperleg = iit::rbd::Vector3d(0.15777,3.1E-4,-0.06355).cast<Scalar>();
    tensor_LF_upperleg.fill(
        Scalar(1.72052),
        com_LF_upperleg,
        rbd::Utils::buildInertiaTensor(
                Scalar(0.010112616),
                Scalar(0.10585742),
                Scalar(0.09674735),
                Scalar(3.439098E-5),
                Scalar(-0.008956144),
                Scalar(-3.26932E-5)) );

    com_LF_lowerleg = iit::rbd::Vector3d(0.19252,0.00186,-0.00113).cast<Scalar>();
    tensor_LF_lowerleg.fill(
        Scalar(0.35123),
        com_LF_lowerleg,
        rbd::Utils::buildInertiaTensor(
                Scalar(1.07957E-4),
                Scalar(0.021312878),
                Scalar(0.021345625),
                Scalar(1.4509186E-4),
                Scalar(-1.12179E-6),
                Scalar(-3.231E-8)) );

}

