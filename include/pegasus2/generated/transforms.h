
#ifndef PEGASUS2_TRANSFORMS_H_
#define PEGASUS2_TRANSFORMS_H_

#include <Eigen/Dense>
#include <iit/rbd/TransformsBase.h>
#include "declarations.h"
#include <iit/rbd/traits/DoubleTrait.h>
#include "kinematics_parameters.h"

namespace iit{

namespace pegasus2 {

template<typename SCALAR, class M>
class TransformMotion : public iit::rbd::SpatialTransformBase<tpl::JointState<SCALAR>, M> {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

template<typename SCALAR, class M>
class TransformForce : public iit::rbd::SpatialTransformBase<tpl::JointState<SCALAR>, M> {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

template<typename SCALAR, class M>
class TransformHomogeneous : public iit::rbd::HomogeneousTransformBase<tpl::JointState<SCALAR>, M> {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};


namespace tpl{


/**
 * The class for the 4-by-4 coordinates transformation matrices for
 * spatial motion vectors.
 */
template <typename TRAIT>
class HomogeneousTransforms {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    typedef typename TRAIT::Scalar Scalar;

    typedef JointState<Scalar> JState;
    class Dummy {};
    typedef typename TransformHomogeneous<Scalar, Dummy>::MatrixType MatrixType;
public:
	class Type_fr_RF_hip_X_fr_RF_upperleg: public TransformHomogeneous<Scalar, Type_fr_RF_hip_X_fr_RF_upperleg>
	{
	public:
		EIGEN_MAKE_ALIGNED_OPERATOR_NEW
		Type_fr_RF_hip_X_fr_RF_upperleg();
		const Type_fr_RF_hip_X_fr_RF_upperleg& update(const JState&);
	protected:
	};

	class Type_fr_RF_upperleg_X_fr_RF_hip: public TransformHomogeneous<Scalar, Type_fr_RF_upperleg_X_fr_RF_hip>
	{
	public:
		EIGEN_MAKE_ALIGNED_OPERATOR_NEW
		Type_fr_RF_upperleg_X_fr_RF_hip();
		const Type_fr_RF_upperleg_X_fr_RF_hip& update(const JState&);
	protected:
	};

	class Type_fr_RF_upperleg_X_fr_RF_lowerleg: public TransformHomogeneous<Scalar, Type_fr_RF_upperleg_X_fr_RF_lowerleg>
	{
	public:
		EIGEN_MAKE_ALIGNED_OPERATOR_NEW
		Type_fr_RF_upperleg_X_fr_RF_lowerleg();
		const Type_fr_RF_upperleg_X_fr_RF_lowerleg& update(const JState&);
	protected:
	};

	class Type_fr_RF_lowerleg_X_fr_RF_upperleg: public TransformHomogeneous<Scalar, Type_fr_RF_lowerleg_X_fr_RF_upperleg>
	{
	public:
		EIGEN_MAKE_ALIGNED_OPERATOR_NEW
		Type_fr_RF_lowerleg_X_fr_RF_upperleg();
		const Type_fr_RF_lowerleg_X_fr_RF_upperleg& update(const JState&);
	protected:
	};

	class Type_fr_RH_hip_X_fr_RH_upperleg: public TransformHomogeneous<Scalar, Type_fr_RH_hip_X_fr_RH_upperleg>
	{
	public:
		EIGEN_MAKE_ALIGNED_OPERATOR_NEW
		Type_fr_RH_hip_X_fr_RH_upperleg();
		const Type_fr_RH_hip_X_fr_RH_upperleg& update(const JState&);
	protected:
	};

	class Type_fr_RH_upperleg_X_fr_RH_hip: public TransformHomogeneous<Scalar, Type_fr_RH_upperleg_X_fr_RH_hip>
	{
	public:
		EIGEN_MAKE_ALIGNED_OPERATOR_NEW
		Type_fr_RH_upperleg_X_fr_RH_hip();
		const Type_fr_RH_upperleg_X_fr_RH_hip& update(const JState&);
	protected:
	};

	class Type_fr_RH_upperleg_X_fr_RH_lowerleg: public TransformHomogeneous<Scalar, Type_fr_RH_upperleg_X_fr_RH_lowerleg>
	{
	public:
		EIGEN_MAKE_ALIGNED_OPERATOR_NEW
		Type_fr_RH_upperleg_X_fr_RH_lowerleg();
		const Type_fr_RH_upperleg_X_fr_RH_lowerleg& update(const JState&);
	protected:
	};

	class Type_fr_RH_lowerleg_X_fr_RH_upperleg: public TransformHomogeneous<Scalar, Type_fr_RH_lowerleg_X_fr_RH_upperleg>
	{
	public:
		EIGEN_MAKE_ALIGNED_OPERATOR_NEW
		Type_fr_RH_lowerleg_X_fr_RH_upperleg();
		const Type_fr_RH_lowerleg_X_fr_RH_upperleg& update(const JState&);
	protected:
	};

	class Type_fr_LH_hip_X_fr_LH_upperleg: public TransformHomogeneous<Scalar, Type_fr_LH_hip_X_fr_LH_upperleg>
	{
	public:
		EIGEN_MAKE_ALIGNED_OPERATOR_NEW
		Type_fr_LH_hip_X_fr_LH_upperleg();
		const Type_fr_LH_hip_X_fr_LH_upperleg& update(const JState&);
	protected:
	};

	class Type_fr_LH_upperleg_X_fr_LH_hip: public TransformHomogeneous<Scalar, Type_fr_LH_upperleg_X_fr_LH_hip>
	{
	public:
		EIGEN_MAKE_ALIGNED_OPERATOR_NEW
		Type_fr_LH_upperleg_X_fr_LH_hip();
		const Type_fr_LH_upperleg_X_fr_LH_hip& update(const JState&);
	protected:
	};

	class Type_fr_LH_upperleg_X_fr_LH_lowerleg: public TransformHomogeneous<Scalar, Type_fr_LH_upperleg_X_fr_LH_lowerleg>
	{
	public:
		EIGEN_MAKE_ALIGNED_OPERATOR_NEW
		Type_fr_LH_upperleg_X_fr_LH_lowerleg();
		const Type_fr_LH_upperleg_X_fr_LH_lowerleg& update(const JState&);
	protected:
	};

	class Type_fr_LH_lowerleg_X_fr_LH_upperleg: public TransformHomogeneous<Scalar, Type_fr_LH_lowerleg_X_fr_LH_upperleg>
	{
	public:
		EIGEN_MAKE_ALIGNED_OPERATOR_NEW
		Type_fr_LH_lowerleg_X_fr_LH_upperleg();
		const Type_fr_LH_lowerleg_X_fr_LH_upperleg& update(const JState&);
	protected:
	};

	class Type_fr_LF_hip_X_fr_LF_upperleg: public TransformHomogeneous<Scalar, Type_fr_LF_hip_X_fr_LF_upperleg>
	{
	public:
		EIGEN_MAKE_ALIGNED_OPERATOR_NEW
		Type_fr_LF_hip_X_fr_LF_upperleg();
		const Type_fr_LF_hip_X_fr_LF_upperleg& update(const JState&);
	protected:
	};

	class Type_fr_LF_upperleg_X_fr_LF_hip: public TransformHomogeneous<Scalar, Type_fr_LF_upperleg_X_fr_LF_hip>
	{
	public:
		EIGEN_MAKE_ALIGNED_OPERATOR_NEW
		Type_fr_LF_upperleg_X_fr_LF_hip();
		const Type_fr_LF_upperleg_X_fr_LF_hip& update(const JState&);
	protected:
	};

	class Type_fr_LF_upperleg_X_fr_LF_lowerleg: public TransformHomogeneous<Scalar, Type_fr_LF_upperleg_X_fr_LF_lowerleg>
	{
	public:
		EIGEN_MAKE_ALIGNED_OPERATOR_NEW
		Type_fr_LF_upperleg_X_fr_LF_lowerleg();
		const Type_fr_LF_upperleg_X_fr_LF_lowerleg& update(const JState&);
	protected:
	};

	class Type_fr_LF_lowerleg_X_fr_LF_upperleg: public TransformHomogeneous<Scalar, Type_fr_LF_lowerleg_X_fr_LF_upperleg>
	{
	public:
		EIGEN_MAKE_ALIGNED_OPERATOR_NEW
		Type_fr_LF_lowerleg_X_fr_LF_upperleg();
		const Type_fr_LF_lowerleg_X_fr_LF_upperleg& update(const JState&);
	protected:
	};

	class Type_fr_torso_X_fr_RF_hip: public TransformHomogeneous<Scalar, Type_fr_torso_X_fr_RF_hip>
	{
	public:
		EIGEN_MAKE_ALIGNED_OPERATOR_NEW
		Type_fr_torso_X_fr_RF_hip();
		const Type_fr_torso_X_fr_RF_hip& update(const JState&);
	protected:
	};

	class Type_fr_RF_hip_X_fr_torso: public TransformHomogeneous<Scalar, Type_fr_RF_hip_X_fr_torso>
	{
	public:
		EIGEN_MAKE_ALIGNED_OPERATOR_NEW
		Type_fr_RF_hip_X_fr_torso();
		const Type_fr_RF_hip_X_fr_torso& update(const JState&);
	protected:
	};

	class Type_fr_torso_X_fr_RF_upperleg: public TransformHomogeneous<Scalar, Type_fr_torso_X_fr_RF_upperleg>
	{
	public:
		EIGEN_MAKE_ALIGNED_OPERATOR_NEW
		Type_fr_torso_X_fr_RF_upperleg();
		const Type_fr_torso_X_fr_RF_upperleg& update(const JState&);
	protected:
	};

	class Type_fr_RF_upperleg_X_fr_torso: public TransformHomogeneous<Scalar, Type_fr_RF_upperleg_X_fr_torso>
	{
	public:
		EIGEN_MAKE_ALIGNED_OPERATOR_NEW
		Type_fr_RF_upperleg_X_fr_torso();
		const Type_fr_RF_upperleg_X_fr_torso& update(const JState&);
	protected:
	};

	class Type_fr_torso_X_fr_RF_lowerleg: public TransformHomogeneous<Scalar, Type_fr_torso_X_fr_RF_lowerleg>
	{
	public:
		EIGEN_MAKE_ALIGNED_OPERATOR_NEW
		Type_fr_torso_X_fr_RF_lowerleg();
		const Type_fr_torso_X_fr_RF_lowerleg& update(const JState&);
	protected:
	};

	class Type_fr_RF_lowerleg_X_fr_torso: public TransformHomogeneous<Scalar, Type_fr_RF_lowerleg_X_fr_torso>
	{
	public:
		EIGEN_MAKE_ALIGNED_OPERATOR_NEW
		Type_fr_RF_lowerleg_X_fr_torso();
		const Type_fr_RF_lowerleg_X_fr_torso& update(const JState&);
	protected:
	};

	class Type_fr_torso_X_fr_RH_hip: public TransformHomogeneous<Scalar, Type_fr_torso_X_fr_RH_hip>
	{
	public:
		EIGEN_MAKE_ALIGNED_OPERATOR_NEW
		Type_fr_torso_X_fr_RH_hip();
		const Type_fr_torso_X_fr_RH_hip& update(const JState&);
	protected:
	};

	class Type_fr_RH_hip_X_fr_torso: public TransformHomogeneous<Scalar, Type_fr_RH_hip_X_fr_torso>
	{
	public:
		EIGEN_MAKE_ALIGNED_OPERATOR_NEW
		Type_fr_RH_hip_X_fr_torso();
		const Type_fr_RH_hip_X_fr_torso& update(const JState&);
	protected:
	};

	class Type_fr_torso_X_fr_RH_upperleg: public TransformHomogeneous<Scalar, Type_fr_torso_X_fr_RH_upperleg>
	{
	public:
		EIGEN_MAKE_ALIGNED_OPERATOR_NEW
		Type_fr_torso_X_fr_RH_upperleg();
		const Type_fr_torso_X_fr_RH_upperleg& update(const JState&);
	protected:
	};

	class Type_fr_RH_upperleg_X_fr_torso: public TransformHomogeneous<Scalar, Type_fr_RH_upperleg_X_fr_torso>
	{
	public:
		EIGEN_MAKE_ALIGNED_OPERATOR_NEW
		Type_fr_RH_upperleg_X_fr_torso();
		const Type_fr_RH_upperleg_X_fr_torso& update(const JState&);
	protected:
	};

	class Type_fr_torso_X_fr_RH_lowerleg: public TransformHomogeneous<Scalar, Type_fr_torso_X_fr_RH_lowerleg>
	{
	public:
		EIGEN_MAKE_ALIGNED_OPERATOR_NEW
		Type_fr_torso_X_fr_RH_lowerleg();
		const Type_fr_torso_X_fr_RH_lowerleg& update(const JState&);
	protected:
	};

	class Type_fr_RH_lowerleg_X_fr_torso: public TransformHomogeneous<Scalar, Type_fr_RH_lowerleg_X_fr_torso>
	{
	public:
		EIGEN_MAKE_ALIGNED_OPERATOR_NEW
		Type_fr_RH_lowerleg_X_fr_torso();
		const Type_fr_RH_lowerleg_X_fr_torso& update(const JState&);
	protected:
	};

	class Type_fr_torso_X_fr_LH_hip: public TransformHomogeneous<Scalar, Type_fr_torso_X_fr_LH_hip>
	{
	public:
		EIGEN_MAKE_ALIGNED_OPERATOR_NEW
		Type_fr_torso_X_fr_LH_hip();
		const Type_fr_torso_X_fr_LH_hip& update(const JState&);
	protected:
	};

	class Type_fr_LH_hip_X_fr_torso: public TransformHomogeneous<Scalar, Type_fr_LH_hip_X_fr_torso>
	{
	public:
		EIGEN_MAKE_ALIGNED_OPERATOR_NEW
		Type_fr_LH_hip_X_fr_torso();
		const Type_fr_LH_hip_X_fr_torso& update(const JState&);
	protected:
	};

	class Type_fr_torso_X_fr_LH_upperleg: public TransformHomogeneous<Scalar, Type_fr_torso_X_fr_LH_upperleg>
	{
	public:
		EIGEN_MAKE_ALIGNED_OPERATOR_NEW
		Type_fr_torso_X_fr_LH_upperleg();
		const Type_fr_torso_X_fr_LH_upperleg& update(const JState&);
	protected:
	};

	class Type_fr_LH_upperleg_X_fr_torso: public TransformHomogeneous<Scalar, Type_fr_LH_upperleg_X_fr_torso>
	{
	public:
		EIGEN_MAKE_ALIGNED_OPERATOR_NEW
		Type_fr_LH_upperleg_X_fr_torso();
		const Type_fr_LH_upperleg_X_fr_torso& update(const JState&);
	protected:
	};

	class Type_fr_torso_X_fr_LH_lowerleg: public TransformHomogeneous<Scalar, Type_fr_torso_X_fr_LH_lowerleg>
	{
	public:
		EIGEN_MAKE_ALIGNED_OPERATOR_NEW
		Type_fr_torso_X_fr_LH_lowerleg();
		const Type_fr_torso_X_fr_LH_lowerleg& update(const JState&);
	protected:
	};

	class Type_fr_LH_lowerleg_X_fr_torso: public TransformHomogeneous<Scalar, Type_fr_LH_lowerleg_X_fr_torso>
	{
	public:
		EIGEN_MAKE_ALIGNED_OPERATOR_NEW
		Type_fr_LH_lowerleg_X_fr_torso();
		const Type_fr_LH_lowerleg_X_fr_torso& update(const JState&);
	protected:
	};

	class Type_fr_torso_X_fr_LF_hip: public TransformHomogeneous<Scalar, Type_fr_torso_X_fr_LF_hip>
	{
	public:
		EIGEN_MAKE_ALIGNED_OPERATOR_NEW
		Type_fr_torso_X_fr_LF_hip();
		const Type_fr_torso_X_fr_LF_hip& update(const JState&);
	protected:
	};

	class Type_fr_LF_hip_X_fr_torso: public TransformHomogeneous<Scalar, Type_fr_LF_hip_X_fr_torso>
	{
	public:
		EIGEN_MAKE_ALIGNED_OPERATOR_NEW
		Type_fr_LF_hip_X_fr_torso();
		const Type_fr_LF_hip_X_fr_torso& update(const JState&);
	protected:
	};

	class Type_fr_torso_X_fr_LF_upperleg: public TransformHomogeneous<Scalar, Type_fr_torso_X_fr_LF_upperleg>
	{
	public:
		EIGEN_MAKE_ALIGNED_OPERATOR_NEW
		Type_fr_torso_X_fr_LF_upperleg();
		const Type_fr_torso_X_fr_LF_upperleg& update(const JState&);
	protected:
	};

	class Type_fr_LF_upperleg_X_fr_torso: public TransformHomogeneous<Scalar, Type_fr_LF_upperleg_X_fr_torso>
	{
	public:
		EIGEN_MAKE_ALIGNED_OPERATOR_NEW
		Type_fr_LF_upperleg_X_fr_torso();
		const Type_fr_LF_upperleg_X_fr_torso& update(const JState&);
	protected:
	};

	class Type_fr_torso_X_fr_LF_lowerleg: public TransformHomogeneous<Scalar, Type_fr_torso_X_fr_LF_lowerleg>
	{
	public:
		EIGEN_MAKE_ALIGNED_OPERATOR_NEW
		Type_fr_torso_X_fr_LF_lowerleg();
		const Type_fr_torso_X_fr_LF_lowerleg& update(const JState&);
	protected:
	};

	class Type_fr_LF_lowerleg_X_fr_torso: public TransformHomogeneous<Scalar, Type_fr_LF_lowerleg_X_fr_torso>
	{
	public:
		EIGEN_MAKE_ALIGNED_OPERATOR_NEW
		Type_fr_LF_lowerleg_X_fr_torso();
		const Type_fr_LF_lowerleg_X_fr_torso& update(const JState&);
	protected:
	};

	class Type_fr_torso_X_fr_RF_foot: public TransformHomogeneous<Scalar, Type_fr_torso_X_fr_RF_foot>
	{
	public:
		EIGEN_MAKE_ALIGNED_OPERATOR_NEW
		Type_fr_torso_X_fr_RF_foot();
		const Type_fr_torso_X_fr_RF_foot& update(const JState&);
	protected:
	};

	class Type_fr_RF_foot_X_fr_torso: public TransformHomogeneous<Scalar, Type_fr_RF_foot_X_fr_torso>
	{
	public:
		EIGEN_MAKE_ALIGNED_OPERATOR_NEW
		Type_fr_RF_foot_X_fr_torso();
		const Type_fr_RF_foot_X_fr_torso& update(const JState&);
	protected:
	};

	class Type_fr_torso_X_fr_RH_foot: public TransformHomogeneous<Scalar, Type_fr_torso_X_fr_RH_foot>
	{
	public:
		EIGEN_MAKE_ALIGNED_OPERATOR_NEW
		Type_fr_torso_X_fr_RH_foot();
		const Type_fr_torso_X_fr_RH_foot& update(const JState&);
	protected:
	};

	class Type_fr_RH_foot_X_fr_torso: public TransformHomogeneous<Scalar, Type_fr_RH_foot_X_fr_torso>
	{
	public:
		EIGEN_MAKE_ALIGNED_OPERATOR_NEW
		Type_fr_RH_foot_X_fr_torso();
		const Type_fr_RH_foot_X_fr_torso& update(const JState&);
	protected:
	};

	class Type_fr_torso_X_fr_LH_foot: public TransformHomogeneous<Scalar, Type_fr_torso_X_fr_LH_foot>
	{
	public:
		EIGEN_MAKE_ALIGNED_OPERATOR_NEW
		Type_fr_torso_X_fr_LH_foot();
		const Type_fr_torso_X_fr_LH_foot& update(const JState&);
	protected:
	};

	class Type_fr_LH_foot_X_fr_torso: public TransformHomogeneous<Scalar, Type_fr_LH_foot_X_fr_torso>
	{
	public:
		EIGEN_MAKE_ALIGNED_OPERATOR_NEW
		Type_fr_LH_foot_X_fr_torso();
		const Type_fr_LH_foot_X_fr_torso& update(const JState&);
	protected:
	};

	class Type_fr_torso_X_fr_LF_foot: public TransformHomogeneous<Scalar, Type_fr_torso_X_fr_LF_foot>
	{
	public:
		EIGEN_MAKE_ALIGNED_OPERATOR_NEW
		Type_fr_torso_X_fr_LF_foot();
		const Type_fr_torso_X_fr_LF_foot& update(const JState&);
	protected:
	};

	class Type_fr_LF_foot_X_fr_torso: public TransformHomogeneous<Scalar, Type_fr_LF_foot_X_fr_torso>
	{
	public:
		EIGEN_MAKE_ALIGNED_OPERATOR_NEW
		Type_fr_LF_foot_X_fr_torso();
		const Type_fr_LF_foot_X_fr_torso& update(const JState&);
	protected:
	};



public:
    HomogeneousTransforms();
    void updateParameters();
	Type_fr_RF_hip_X_fr_RF_upperleg fr_RF_hip_X_fr_RF_upperleg;
	Type_fr_RF_upperleg_X_fr_RF_hip fr_RF_upperleg_X_fr_RF_hip;
	Type_fr_RF_upperleg_X_fr_RF_lowerleg fr_RF_upperleg_X_fr_RF_lowerleg;
	Type_fr_RF_lowerleg_X_fr_RF_upperleg fr_RF_lowerleg_X_fr_RF_upperleg;
	Type_fr_RH_hip_X_fr_RH_upperleg fr_RH_hip_X_fr_RH_upperleg;
	Type_fr_RH_upperleg_X_fr_RH_hip fr_RH_upperleg_X_fr_RH_hip;
	Type_fr_RH_upperleg_X_fr_RH_lowerleg fr_RH_upperleg_X_fr_RH_lowerleg;
	Type_fr_RH_lowerleg_X_fr_RH_upperleg fr_RH_lowerleg_X_fr_RH_upperleg;
	Type_fr_LH_hip_X_fr_LH_upperleg fr_LH_hip_X_fr_LH_upperleg;
	Type_fr_LH_upperleg_X_fr_LH_hip fr_LH_upperleg_X_fr_LH_hip;
	Type_fr_LH_upperleg_X_fr_LH_lowerleg fr_LH_upperleg_X_fr_LH_lowerleg;
	Type_fr_LH_lowerleg_X_fr_LH_upperleg fr_LH_lowerleg_X_fr_LH_upperleg;
	Type_fr_LF_hip_X_fr_LF_upperleg fr_LF_hip_X_fr_LF_upperleg;
	Type_fr_LF_upperleg_X_fr_LF_hip fr_LF_upperleg_X_fr_LF_hip;
	Type_fr_LF_upperleg_X_fr_LF_lowerleg fr_LF_upperleg_X_fr_LF_lowerleg;
	Type_fr_LF_lowerleg_X_fr_LF_upperleg fr_LF_lowerleg_X_fr_LF_upperleg;
	Type_fr_torso_X_fr_RF_hip fr_torso_X_fr_RF_hip;
	Type_fr_RF_hip_X_fr_torso fr_RF_hip_X_fr_torso;
	Type_fr_torso_X_fr_RF_upperleg fr_torso_X_fr_RF_upperleg;
	Type_fr_RF_upperleg_X_fr_torso fr_RF_upperleg_X_fr_torso;
	Type_fr_torso_X_fr_RF_lowerleg fr_torso_X_fr_RF_lowerleg;
	Type_fr_RF_lowerleg_X_fr_torso fr_RF_lowerleg_X_fr_torso;
	Type_fr_torso_X_fr_RH_hip fr_torso_X_fr_RH_hip;
	Type_fr_RH_hip_X_fr_torso fr_RH_hip_X_fr_torso;
	Type_fr_torso_X_fr_RH_upperleg fr_torso_X_fr_RH_upperleg;
	Type_fr_RH_upperleg_X_fr_torso fr_RH_upperleg_X_fr_torso;
	Type_fr_torso_X_fr_RH_lowerleg fr_torso_X_fr_RH_lowerleg;
	Type_fr_RH_lowerleg_X_fr_torso fr_RH_lowerleg_X_fr_torso;
	Type_fr_torso_X_fr_LH_hip fr_torso_X_fr_LH_hip;
	Type_fr_LH_hip_X_fr_torso fr_LH_hip_X_fr_torso;
	Type_fr_torso_X_fr_LH_upperleg fr_torso_X_fr_LH_upperleg;
	Type_fr_LH_upperleg_X_fr_torso fr_LH_upperleg_X_fr_torso;
	Type_fr_torso_X_fr_LH_lowerleg fr_torso_X_fr_LH_lowerleg;
	Type_fr_LH_lowerleg_X_fr_torso fr_LH_lowerleg_X_fr_torso;
	Type_fr_torso_X_fr_LF_hip fr_torso_X_fr_LF_hip;
	Type_fr_LF_hip_X_fr_torso fr_LF_hip_X_fr_torso;
	Type_fr_torso_X_fr_LF_upperleg fr_torso_X_fr_LF_upperleg;
	Type_fr_LF_upperleg_X_fr_torso fr_LF_upperleg_X_fr_torso;
	Type_fr_torso_X_fr_LF_lowerleg fr_torso_X_fr_LF_lowerleg;
	Type_fr_LF_lowerleg_X_fr_torso fr_LF_lowerleg_X_fr_torso;
	Type_fr_torso_X_fr_RF_foot fr_torso_X_fr_RF_foot;
	Type_fr_RF_foot_X_fr_torso fr_RF_foot_X_fr_torso;
	Type_fr_torso_X_fr_RH_foot fr_torso_X_fr_RH_foot;
	Type_fr_RH_foot_X_fr_torso fr_RH_foot_X_fr_torso;
	Type_fr_torso_X_fr_LH_foot fr_torso_X_fr_LH_foot;
	Type_fr_LH_foot_X_fr_torso fr_LH_foot_X_fr_torso;
	Type_fr_torso_X_fr_LF_foot fr_torso_X_fr_LF_foot;
	Type_fr_LF_foot_X_fr_torso fr_LF_foot_X_fr_torso;


}; // HomogeneousTransforms

/**
 * The class for the 6-by-6 coordinates transformation matrices for
 * spatial motion vectors.
 */
template <typename TRAIT>
class MotionTransforms {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    typedef typename TRAIT::Scalar Scalar;

    typedef JointState<Scalar> JState;
    class Dummy {};
    typedef typename TransformMotion<Scalar, Dummy>::MatrixType MatrixType;
public:
	class Type_fr_RF_hip_X_fr_RF_upperleg: public TransformMotion<Scalar, Type_fr_RF_hip_X_fr_RF_upperleg>
	{
	public:
		EIGEN_MAKE_ALIGNED_OPERATOR_NEW
		Type_fr_RF_hip_X_fr_RF_upperleg();
		const Type_fr_RF_hip_X_fr_RF_upperleg& update(const JState&);
	protected:
	};

	class Type_fr_RF_upperleg_X_fr_RF_hip: public TransformMotion<Scalar, Type_fr_RF_upperleg_X_fr_RF_hip>
	{
	public:
		EIGEN_MAKE_ALIGNED_OPERATOR_NEW
		Type_fr_RF_upperleg_X_fr_RF_hip();
		const Type_fr_RF_upperleg_X_fr_RF_hip& update(const JState&);
	protected:
	};

	class Type_fr_RF_upperleg_X_fr_RF_lowerleg: public TransformMotion<Scalar, Type_fr_RF_upperleg_X_fr_RF_lowerleg>
	{
	public:
		EIGEN_MAKE_ALIGNED_OPERATOR_NEW
		Type_fr_RF_upperleg_X_fr_RF_lowerleg();
		const Type_fr_RF_upperleg_X_fr_RF_lowerleg& update(const JState&);
	protected:
	};

	class Type_fr_RF_lowerleg_X_fr_RF_upperleg: public TransformMotion<Scalar, Type_fr_RF_lowerleg_X_fr_RF_upperleg>
	{
	public:
		EIGEN_MAKE_ALIGNED_OPERATOR_NEW
		Type_fr_RF_lowerleg_X_fr_RF_upperleg();
		const Type_fr_RF_lowerleg_X_fr_RF_upperleg& update(const JState&);
	protected:
	};

	class Type_fr_RH_hip_X_fr_RH_upperleg: public TransformMotion<Scalar, Type_fr_RH_hip_X_fr_RH_upperleg>
	{
	public:
		EIGEN_MAKE_ALIGNED_OPERATOR_NEW
		Type_fr_RH_hip_X_fr_RH_upperleg();
		const Type_fr_RH_hip_X_fr_RH_upperleg& update(const JState&);
	protected:
	};

	class Type_fr_RH_upperleg_X_fr_RH_hip: public TransformMotion<Scalar, Type_fr_RH_upperleg_X_fr_RH_hip>
	{
	public:
		EIGEN_MAKE_ALIGNED_OPERATOR_NEW
		Type_fr_RH_upperleg_X_fr_RH_hip();
		const Type_fr_RH_upperleg_X_fr_RH_hip& update(const JState&);
	protected:
	};

	class Type_fr_RH_upperleg_X_fr_RH_lowerleg: public TransformMotion<Scalar, Type_fr_RH_upperleg_X_fr_RH_lowerleg>
	{
	public:
		EIGEN_MAKE_ALIGNED_OPERATOR_NEW
		Type_fr_RH_upperleg_X_fr_RH_lowerleg();
		const Type_fr_RH_upperleg_X_fr_RH_lowerleg& update(const JState&);
	protected:
	};

	class Type_fr_RH_lowerleg_X_fr_RH_upperleg: public TransformMotion<Scalar, Type_fr_RH_lowerleg_X_fr_RH_upperleg>
	{
	public:
		EIGEN_MAKE_ALIGNED_OPERATOR_NEW
		Type_fr_RH_lowerleg_X_fr_RH_upperleg();
		const Type_fr_RH_lowerleg_X_fr_RH_upperleg& update(const JState&);
	protected:
	};

	class Type_fr_LH_hip_X_fr_LH_upperleg: public TransformMotion<Scalar, Type_fr_LH_hip_X_fr_LH_upperleg>
	{
	public:
		EIGEN_MAKE_ALIGNED_OPERATOR_NEW
		Type_fr_LH_hip_X_fr_LH_upperleg();
		const Type_fr_LH_hip_X_fr_LH_upperleg& update(const JState&);
	protected:
	};

	class Type_fr_LH_upperleg_X_fr_LH_hip: public TransformMotion<Scalar, Type_fr_LH_upperleg_X_fr_LH_hip>
	{
	public:
		EIGEN_MAKE_ALIGNED_OPERATOR_NEW
		Type_fr_LH_upperleg_X_fr_LH_hip();
		const Type_fr_LH_upperleg_X_fr_LH_hip& update(const JState&);
	protected:
	};

	class Type_fr_LH_upperleg_X_fr_LH_lowerleg: public TransformMotion<Scalar, Type_fr_LH_upperleg_X_fr_LH_lowerleg>
	{
	public:
		EIGEN_MAKE_ALIGNED_OPERATOR_NEW
		Type_fr_LH_upperleg_X_fr_LH_lowerleg();
		const Type_fr_LH_upperleg_X_fr_LH_lowerleg& update(const JState&);
	protected:
	};

	class Type_fr_LH_lowerleg_X_fr_LH_upperleg: public TransformMotion<Scalar, Type_fr_LH_lowerleg_X_fr_LH_upperleg>
	{
	public:
		EIGEN_MAKE_ALIGNED_OPERATOR_NEW
		Type_fr_LH_lowerleg_X_fr_LH_upperleg();
		const Type_fr_LH_lowerleg_X_fr_LH_upperleg& update(const JState&);
	protected:
	};

	class Type_fr_LF_hip_X_fr_LF_upperleg: public TransformMotion<Scalar, Type_fr_LF_hip_X_fr_LF_upperleg>
	{
	public:
		EIGEN_MAKE_ALIGNED_OPERATOR_NEW
		Type_fr_LF_hip_X_fr_LF_upperleg();
		const Type_fr_LF_hip_X_fr_LF_upperleg& update(const JState&);
	protected:
	};

	class Type_fr_LF_upperleg_X_fr_LF_hip: public TransformMotion<Scalar, Type_fr_LF_upperleg_X_fr_LF_hip>
	{
	public:
		EIGEN_MAKE_ALIGNED_OPERATOR_NEW
		Type_fr_LF_upperleg_X_fr_LF_hip();
		const Type_fr_LF_upperleg_X_fr_LF_hip& update(const JState&);
	protected:
	};

	class Type_fr_LF_upperleg_X_fr_LF_lowerleg: public TransformMotion<Scalar, Type_fr_LF_upperleg_X_fr_LF_lowerleg>
	{
	public:
		EIGEN_MAKE_ALIGNED_OPERATOR_NEW
		Type_fr_LF_upperleg_X_fr_LF_lowerleg();
		const Type_fr_LF_upperleg_X_fr_LF_lowerleg& update(const JState&);
	protected:
	};

	class Type_fr_LF_lowerleg_X_fr_LF_upperleg: public TransformMotion<Scalar, Type_fr_LF_lowerleg_X_fr_LF_upperleg>
	{
	public:
		EIGEN_MAKE_ALIGNED_OPERATOR_NEW
		Type_fr_LF_lowerleg_X_fr_LF_upperleg();
		const Type_fr_LF_lowerleg_X_fr_LF_upperleg& update(const JState&);
	protected:
	};

	class Type_fr_torso_X_fr_RF_hip: public TransformMotion<Scalar, Type_fr_torso_X_fr_RF_hip>
	{
	public:
		EIGEN_MAKE_ALIGNED_OPERATOR_NEW
		Type_fr_torso_X_fr_RF_hip();
		const Type_fr_torso_X_fr_RF_hip& update(const JState&);
	protected:
	};

	class Type_fr_RF_hip_X_fr_torso: public TransformMotion<Scalar, Type_fr_RF_hip_X_fr_torso>
	{
	public:
		EIGEN_MAKE_ALIGNED_OPERATOR_NEW
		Type_fr_RF_hip_X_fr_torso();
		const Type_fr_RF_hip_X_fr_torso& update(const JState&);
	protected:
	};

	class Type_fr_torso_X_fr_RF_upperleg: public TransformMotion<Scalar, Type_fr_torso_X_fr_RF_upperleg>
	{
	public:
		EIGEN_MAKE_ALIGNED_OPERATOR_NEW
		Type_fr_torso_X_fr_RF_upperleg();
		const Type_fr_torso_X_fr_RF_upperleg& update(const JState&);
	protected:
	};

	class Type_fr_RF_upperleg_X_fr_torso: public TransformMotion<Scalar, Type_fr_RF_upperleg_X_fr_torso>
	{
	public:
		EIGEN_MAKE_ALIGNED_OPERATOR_NEW
		Type_fr_RF_upperleg_X_fr_torso();
		const Type_fr_RF_upperleg_X_fr_torso& update(const JState&);
	protected:
	};

	class Type_fr_torso_X_fr_RF_lowerleg: public TransformMotion<Scalar, Type_fr_torso_X_fr_RF_lowerleg>
	{
	public:
		EIGEN_MAKE_ALIGNED_OPERATOR_NEW
		Type_fr_torso_X_fr_RF_lowerleg();
		const Type_fr_torso_X_fr_RF_lowerleg& update(const JState&);
	protected:
	};

	class Type_fr_RF_lowerleg_X_fr_torso: public TransformMotion<Scalar, Type_fr_RF_lowerleg_X_fr_torso>
	{
	public:
		EIGEN_MAKE_ALIGNED_OPERATOR_NEW
		Type_fr_RF_lowerleg_X_fr_torso();
		const Type_fr_RF_lowerleg_X_fr_torso& update(const JState&);
	protected:
	};

	class Type_fr_torso_X_fr_RH_hip: public TransformMotion<Scalar, Type_fr_torso_X_fr_RH_hip>
	{
	public:
		EIGEN_MAKE_ALIGNED_OPERATOR_NEW
		Type_fr_torso_X_fr_RH_hip();
		const Type_fr_torso_X_fr_RH_hip& update(const JState&);
	protected:
	};

	class Type_fr_RH_hip_X_fr_torso: public TransformMotion<Scalar, Type_fr_RH_hip_X_fr_torso>
	{
	public:
		EIGEN_MAKE_ALIGNED_OPERATOR_NEW
		Type_fr_RH_hip_X_fr_torso();
		const Type_fr_RH_hip_X_fr_torso& update(const JState&);
	protected:
	};

	class Type_fr_torso_X_fr_RH_upperleg: public TransformMotion<Scalar, Type_fr_torso_X_fr_RH_upperleg>
	{
	public:
		EIGEN_MAKE_ALIGNED_OPERATOR_NEW
		Type_fr_torso_X_fr_RH_upperleg();
		const Type_fr_torso_X_fr_RH_upperleg& update(const JState&);
	protected:
	};

	class Type_fr_RH_upperleg_X_fr_torso: public TransformMotion<Scalar, Type_fr_RH_upperleg_X_fr_torso>
	{
	public:
		EIGEN_MAKE_ALIGNED_OPERATOR_NEW
		Type_fr_RH_upperleg_X_fr_torso();
		const Type_fr_RH_upperleg_X_fr_torso& update(const JState&);
	protected:
	};

	class Type_fr_torso_X_fr_RH_lowerleg: public TransformMotion<Scalar, Type_fr_torso_X_fr_RH_lowerleg>
	{
	public:
		EIGEN_MAKE_ALIGNED_OPERATOR_NEW
		Type_fr_torso_X_fr_RH_lowerleg();
		const Type_fr_torso_X_fr_RH_lowerleg& update(const JState&);
	protected:
	};

	class Type_fr_RH_lowerleg_X_fr_torso: public TransformMotion<Scalar, Type_fr_RH_lowerleg_X_fr_torso>
	{
	public:
		EIGEN_MAKE_ALIGNED_OPERATOR_NEW
		Type_fr_RH_lowerleg_X_fr_torso();
		const Type_fr_RH_lowerleg_X_fr_torso& update(const JState&);
	protected:
	};

	class Type_fr_torso_X_fr_LH_hip: public TransformMotion<Scalar, Type_fr_torso_X_fr_LH_hip>
	{
	public:
		EIGEN_MAKE_ALIGNED_OPERATOR_NEW
		Type_fr_torso_X_fr_LH_hip();
		const Type_fr_torso_X_fr_LH_hip& update(const JState&);
	protected:
	};

	class Type_fr_LH_hip_X_fr_torso: public TransformMotion<Scalar, Type_fr_LH_hip_X_fr_torso>
	{
	public:
		EIGEN_MAKE_ALIGNED_OPERATOR_NEW
		Type_fr_LH_hip_X_fr_torso();
		const Type_fr_LH_hip_X_fr_torso& update(const JState&);
	protected:
	};

	class Type_fr_torso_X_fr_LH_upperleg: public TransformMotion<Scalar, Type_fr_torso_X_fr_LH_upperleg>
	{
	public:
		EIGEN_MAKE_ALIGNED_OPERATOR_NEW
		Type_fr_torso_X_fr_LH_upperleg();
		const Type_fr_torso_X_fr_LH_upperleg& update(const JState&);
	protected:
	};

	class Type_fr_LH_upperleg_X_fr_torso: public TransformMotion<Scalar, Type_fr_LH_upperleg_X_fr_torso>
	{
	public:
		EIGEN_MAKE_ALIGNED_OPERATOR_NEW
		Type_fr_LH_upperleg_X_fr_torso();
		const Type_fr_LH_upperleg_X_fr_torso& update(const JState&);
	protected:
	};

	class Type_fr_torso_X_fr_LH_lowerleg: public TransformMotion<Scalar, Type_fr_torso_X_fr_LH_lowerleg>
	{
	public:
		EIGEN_MAKE_ALIGNED_OPERATOR_NEW
		Type_fr_torso_X_fr_LH_lowerleg();
		const Type_fr_torso_X_fr_LH_lowerleg& update(const JState&);
	protected:
	};

	class Type_fr_LH_lowerleg_X_fr_torso: public TransformMotion<Scalar, Type_fr_LH_lowerleg_X_fr_torso>
	{
	public:
		EIGEN_MAKE_ALIGNED_OPERATOR_NEW
		Type_fr_LH_lowerleg_X_fr_torso();
		const Type_fr_LH_lowerleg_X_fr_torso& update(const JState&);
	protected:
	};

	class Type_fr_torso_X_fr_LF_hip: public TransformMotion<Scalar, Type_fr_torso_X_fr_LF_hip>
	{
	public:
		EIGEN_MAKE_ALIGNED_OPERATOR_NEW
		Type_fr_torso_X_fr_LF_hip();
		const Type_fr_torso_X_fr_LF_hip& update(const JState&);
	protected:
	};

	class Type_fr_LF_hip_X_fr_torso: public TransformMotion<Scalar, Type_fr_LF_hip_X_fr_torso>
	{
	public:
		EIGEN_MAKE_ALIGNED_OPERATOR_NEW
		Type_fr_LF_hip_X_fr_torso();
		const Type_fr_LF_hip_X_fr_torso& update(const JState&);
	protected:
	};

	class Type_fr_torso_X_fr_LF_upperleg: public TransformMotion<Scalar, Type_fr_torso_X_fr_LF_upperleg>
	{
	public:
		EIGEN_MAKE_ALIGNED_OPERATOR_NEW
		Type_fr_torso_X_fr_LF_upperleg();
		const Type_fr_torso_X_fr_LF_upperleg& update(const JState&);
	protected:
	};

	class Type_fr_LF_upperleg_X_fr_torso: public TransformMotion<Scalar, Type_fr_LF_upperleg_X_fr_torso>
	{
	public:
		EIGEN_MAKE_ALIGNED_OPERATOR_NEW
		Type_fr_LF_upperleg_X_fr_torso();
		const Type_fr_LF_upperleg_X_fr_torso& update(const JState&);
	protected:
	};

	class Type_fr_torso_X_fr_LF_lowerleg: public TransformMotion<Scalar, Type_fr_torso_X_fr_LF_lowerleg>
	{
	public:
		EIGEN_MAKE_ALIGNED_OPERATOR_NEW
		Type_fr_torso_X_fr_LF_lowerleg();
		const Type_fr_torso_X_fr_LF_lowerleg& update(const JState&);
	protected:
	};

	class Type_fr_LF_lowerleg_X_fr_torso: public TransformMotion<Scalar, Type_fr_LF_lowerleg_X_fr_torso>
	{
	public:
		EIGEN_MAKE_ALIGNED_OPERATOR_NEW
		Type_fr_LF_lowerleg_X_fr_torso();
		const Type_fr_LF_lowerleg_X_fr_torso& update(const JState&);
	protected:
	};

	class Type_fr_torso_X_fr_RF_foot: public TransformMotion<Scalar, Type_fr_torso_X_fr_RF_foot>
	{
	public:
		EIGEN_MAKE_ALIGNED_OPERATOR_NEW
		Type_fr_torso_X_fr_RF_foot();
		const Type_fr_torso_X_fr_RF_foot& update(const JState&);
	protected:
	};

	class Type_fr_RF_foot_X_fr_torso: public TransformMotion<Scalar, Type_fr_RF_foot_X_fr_torso>
	{
	public:
		EIGEN_MAKE_ALIGNED_OPERATOR_NEW
		Type_fr_RF_foot_X_fr_torso();
		const Type_fr_RF_foot_X_fr_torso& update(const JState&);
	protected:
	};

	class Type_fr_torso_X_fr_RH_foot: public TransformMotion<Scalar, Type_fr_torso_X_fr_RH_foot>
	{
	public:
		EIGEN_MAKE_ALIGNED_OPERATOR_NEW
		Type_fr_torso_X_fr_RH_foot();
		const Type_fr_torso_X_fr_RH_foot& update(const JState&);
	protected:
	};

	class Type_fr_RH_foot_X_fr_torso: public TransformMotion<Scalar, Type_fr_RH_foot_X_fr_torso>
	{
	public:
		EIGEN_MAKE_ALIGNED_OPERATOR_NEW
		Type_fr_RH_foot_X_fr_torso();
		const Type_fr_RH_foot_X_fr_torso& update(const JState&);
	protected:
	};

	class Type_fr_torso_X_fr_LH_foot: public TransformMotion<Scalar, Type_fr_torso_X_fr_LH_foot>
	{
	public:
		EIGEN_MAKE_ALIGNED_OPERATOR_NEW
		Type_fr_torso_X_fr_LH_foot();
		const Type_fr_torso_X_fr_LH_foot& update(const JState&);
	protected:
	};

	class Type_fr_LH_foot_X_fr_torso: public TransformMotion<Scalar, Type_fr_LH_foot_X_fr_torso>
	{
	public:
		EIGEN_MAKE_ALIGNED_OPERATOR_NEW
		Type_fr_LH_foot_X_fr_torso();
		const Type_fr_LH_foot_X_fr_torso& update(const JState&);
	protected:
	};

	class Type_fr_torso_X_fr_LF_foot: public TransformMotion<Scalar, Type_fr_torso_X_fr_LF_foot>
	{
	public:
		EIGEN_MAKE_ALIGNED_OPERATOR_NEW
		Type_fr_torso_X_fr_LF_foot();
		const Type_fr_torso_X_fr_LF_foot& update(const JState&);
	protected:
	};

	class Type_fr_LF_foot_X_fr_torso: public TransformMotion<Scalar, Type_fr_LF_foot_X_fr_torso>
	{
	public:
		EIGEN_MAKE_ALIGNED_OPERATOR_NEW
		Type_fr_LF_foot_X_fr_torso();
		const Type_fr_LF_foot_X_fr_torso& update(const JState&);
	protected:
	};



public:
    MotionTransforms();
    void updateParameters();
	Type_fr_RF_hip_X_fr_RF_upperleg fr_RF_hip_X_fr_RF_upperleg;
	Type_fr_RF_upperleg_X_fr_RF_hip fr_RF_upperleg_X_fr_RF_hip;
	Type_fr_RF_upperleg_X_fr_RF_lowerleg fr_RF_upperleg_X_fr_RF_lowerleg;
	Type_fr_RF_lowerleg_X_fr_RF_upperleg fr_RF_lowerleg_X_fr_RF_upperleg;
	Type_fr_RH_hip_X_fr_RH_upperleg fr_RH_hip_X_fr_RH_upperleg;
	Type_fr_RH_upperleg_X_fr_RH_hip fr_RH_upperleg_X_fr_RH_hip;
	Type_fr_RH_upperleg_X_fr_RH_lowerleg fr_RH_upperleg_X_fr_RH_lowerleg;
	Type_fr_RH_lowerleg_X_fr_RH_upperleg fr_RH_lowerleg_X_fr_RH_upperleg;
	Type_fr_LH_hip_X_fr_LH_upperleg fr_LH_hip_X_fr_LH_upperleg;
	Type_fr_LH_upperleg_X_fr_LH_hip fr_LH_upperleg_X_fr_LH_hip;
	Type_fr_LH_upperleg_X_fr_LH_lowerleg fr_LH_upperleg_X_fr_LH_lowerleg;
	Type_fr_LH_lowerleg_X_fr_LH_upperleg fr_LH_lowerleg_X_fr_LH_upperleg;
	Type_fr_LF_hip_X_fr_LF_upperleg fr_LF_hip_X_fr_LF_upperleg;
	Type_fr_LF_upperleg_X_fr_LF_hip fr_LF_upperleg_X_fr_LF_hip;
	Type_fr_LF_upperleg_X_fr_LF_lowerleg fr_LF_upperleg_X_fr_LF_lowerleg;
	Type_fr_LF_lowerleg_X_fr_LF_upperleg fr_LF_lowerleg_X_fr_LF_upperleg;
	Type_fr_torso_X_fr_RF_hip fr_torso_X_fr_RF_hip;
	Type_fr_RF_hip_X_fr_torso fr_RF_hip_X_fr_torso;
	Type_fr_torso_X_fr_RF_upperleg fr_torso_X_fr_RF_upperleg;
	Type_fr_RF_upperleg_X_fr_torso fr_RF_upperleg_X_fr_torso;
	Type_fr_torso_X_fr_RF_lowerleg fr_torso_X_fr_RF_lowerleg;
	Type_fr_RF_lowerleg_X_fr_torso fr_RF_lowerleg_X_fr_torso;
	Type_fr_torso_X_fr_RH_hip fr_torso_X_fr_RH_hip;
	Type_fr_RH_hip_X_fr_torso fr_RH_hip_X_fr_torso;
	Type_fr_torso_X_fr_RH_upperleg fr_torso_X_fr_RH_upperleg;
	Type_fr_RH_upperleg_X_fr_torso fr_RH_upperleg_X_fr_torso;
	Type_fr_torso_X_fr_RH_lowerleg fr_torso_X_fr_RH_lowerleg;
	Type_fr_RH_lowerleg_X_fr_torso fr_RH_lowerleg_X_fr_torso;
	Type_fr_torso_X_fr_LH_hip fr_torso_X_fr_LH_hip;
	Type_fr_LH_hip_X_fr_torso fr_LH_hip_X_fr_torso;
	Type_fr_torso_X_fr_LH_upperleg fr_torso_X_fr_LH_upperleg;
	Type_fr_LH_upperleg_X_fr_torso fr_LH_upperleg_X_fr_torso;
	Type_fr_torso_X_fr_LH_lowerleg fr_torso_X_fr_LH_lowerleg;
	Type_fr_LH_lowerleg_X_fr_torso fr_LH_lowerleg_X_fr_torso;
	Type_fr_torso_X_fr_LF_hip fr_torso_X_fr_LF_hip;
	Type_fr_LF_hip_X_fr_torso fr_LF_hip_X_fr_torso;
	Type_fr_torso_X_fr_LF_upperleg fr_torso_X_fr_LF_upperleg;
	Type_fr_LF_upperleg_X_fr_torso fr_LF_upperleg_X_fr_torso;
	Type_fr_torso_X_fr_LF_lowerleg fr_torso_X_fr_LF_lowerleg;
	Type_fr_LF_lowerleg_X_fr_torso fr_LF_lowerleg_X_fr_torso;
	Type_fr_torso_X_fr_RF_foot fr_torso_X_fr_RF_foot;
	Type_fr_RF_foot_X_fr_torso fr_RF_foot_X_fr_torso;
	Type_fr_torso_X_fr_RH_foot fr_torso_X_fr_RH_foot;
	Type_fr_RH_foot_X_fr_torso fr_RH_foot_X_fr_torso;
	Type_fr_torso_X_fr_LH_foot fr_torso_X_fr_LH_foot;
	Type_fr_LH_foot_X_fr_torso fr_LH_foot_X_fr_torso;
	Type_fr_torso_X_fr_LF_foot fr_torso_X_fr_LF_foot;
	Type_fr_LF_foot_X_fr_torso fr_LF_foot_X_fr_torso;


}; // MotionTransforms

/**
 * The class for the 6-by-6 coordinates transformation matrices for
 * spatial motion vectors.
 */
template <typename TRAIT>
class ForceTransforms {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    typedef typename TRAIT::Scalar Scalar;

    typedef JointState<Scalar> JState;
    class Dummy {};
    typedef typename TransformForce<Scalar, Dummy>::MatrixType MatrixType;
public:
	class Type_fr_RF_hip_X_fr_RF_upperleg: public TransformForce<Scalar, Type_fr_RF_hip_X_fr_RF_upperleg>
	{
	public:
		EIGEN_MAKE_ALIGNED_OPERATOR_NEW
		Type_fr_RF_hip_X_fr_RF_upperleg();
		const Type_fr_RF_hip_X_fr_RF_upperleg& update(const JState&);
	protected:
	};

	class Type_fr_RF_upperleg_X_fr_RF_hip: public TransformForce<Scalar, Type_fr_RF_upperleg_X_fr_RF_hip>
	{
	public:
		EIGEN_MAKE_ALIGNED_OPERATOR_NEW
		Type_fr_RF_upperleg_X_fr_RF_hip();
		const Type_fr_RF_upperleg_X_fr_RF_hip& update(const JState&);
	protected:
	};

	class Type_fr_RF_upperleg_X_fr_RF_lowerleg: public TransformForce<Scalar, Type_fr_RF_upperleg_X_fr_RF_lowerleg>
	{
	public:
		EIGEN_MAKE_ALIGNED_OPERATOR_NEW
		Type_fr_RF_upperleg_X_fr_RF_lowerleg();
		const Type_fr_RF_upperleg_X_fr_RF_lowerleg& update(const JState&);
	protected:
	};

	class Type_fr_RF_lowerleg_X_fr_RF_upperleg: public TransformForce<Scalar, Type_fr_RF_lowerleg_X_fr_RF_upperleg>
	{
	public:
		EIGEN_MAKE_ALIGNED_OPERATOR_NEW
		Type_fr_RF_lowerleg_X_fr_RF_upperleg();
		const Type_fr_RF_lowerleg_X_fr_RF_upperleg& update(const JState&);
	protected:
	};

	class Type_fr_RH_hip_X_fr_RH_upperleg: public TransformForce<Scalar, Type_fr_RH_hip_X_fr_RH_upperleg>
	{
	public:
		EIGEN_MAKE_ALIGNED_OPERATOR_NEW
		Type_fr_RH_hip_X_fr_RH_upperleg();
		const Type_fr_RH_hip_X_fr_RH_upperleg& update(const JState&);
	protected:
	};

	class Type_fr_RH_upperleg_X_fr_RH_hip: public TransformForce<Scalar, Type_fr_RH_upperleg_X_fr_RH_hip>
	{
	public:
		EIGEN_MAKE_ALIGNED_OPERATOR_NEW
		Type_fr_RH_upperleg_X_fr_RH_hip();
		const Type_fr_RH_upperleg_X_fr_RH_hip& update(const JState&);
	protected:
	};

	class Type_fr_RH_upperleg_X_fr_RH_lowerleg: public TransformForce<Scalar, Type_fr_RH_upperleg_X_fr_RH_lowerleg>
	{
	public:
		EIGEN_MAKE_ALIGNED_OPERATOR_NEW
		Type_fr_RH_upperleg_X_fr_RH_lowerleg();
		const Type_fr_RH_upperleg_X_fr_RH_lowerleg& update(const JState&);
	protected:
	};

	class Type_fr_RH_lowerleg_X_fr_RH_upperleg: public TransformForce<Scalar, Type_fr_RH_lowerleg_X_fr_RH_upperleg>
	{
	public:
		EIGEN_MAKE_ALIGNED_OPERATOR_NEW
		Type_fr_RH_lowerleg_X_fr_RH_upperleg();
		const Type_fr_RH_lowerleg_X_fr_RH_upperleg& update(const JState&);
	protected:
	};

	class Type_fr_LH_hip_X_fr_LH_upperleg: public TransformForce<Scalar, Type_fr_LH_hip_X_fr_LH_upperleg>
	{
	public:
		EIGEN_MAKE_ALIGNED_OPERATOR_NEW
		Type_fr_LH_hip_X_fr_LH_upperleg();
		const Type_fr_LH_hip_X_fr_LH_upperleg& update(const JState&);
	protected:
	};

	class Type_fr_LH_upperleg_X_fr_LH_hip: public TransformForce<Scalar, Type_fr_LH_upperleg_X_fr_LH_hip>
	{
	public:
		EIGEN_MAKE_ALIGNED_OPERATOR_NEW
		Type_fr_LH_upperleg_X_fr_LH_hip();
		const Type_fr_LH_upperleg_X_fr_LH_hip& update(const JState&);
	protected:
	};

	class Type_fr_LH_upperleg_X_fr_LH_lowerleg: public TransformForce<Scalar, Type_fr_LH_upperleg_X_fr_LH_lowerleg>
	{
	public:
		EIGEN_MAKE_ALIGNED_OPERATOR_NEW
		Type_fr_LH_upperleg_X_fr_LH_lowerleg();
		const Type_fr_LH_upperleg_X_fr_LH_lowerleg& update(const JState&);
	protected:
	};

	class Type_fr_LH_lowerleg_X_fr_LH_upperleg: public TransformForce<Scalar, Type_fr_LH_lowerleg_X_fr_LH_upperleg>
	{
	public:
		EIGEN_MAKE_ALIGNED_OPERATOR_NEW
		Type_fr_LH_lowerleg_X_fr_LH_upperleg();
		const Type_fr_LH_lowerleg_X_fr_LH_upperleg& update(const JState&);
	protected:
	};

	class Type_fr_LF_hip_X_fr_LF_upperleg: public TransformForce<Scalar, Type_fr_LF_hip_X_fr_LF_upperleg>
	{
	public:
		EIGEN_MAKE_ALIGNED_OPERATOR_NEW
		Type_fr_LF_hip_X_fr_LF_upperleg();
		const Type_fr_LF_hip_X_fr_LF_upperleg& update(const JState&);
	protected:
	};

	class Type_fr_LF_upperleg_X_fr_LF_hip: public TransformForce<Scalar, Type_fr_LF_upperleg_X_fr_LF_hip>
	{
	public:
		EIGEN_MAKE_ALIGNED_OPERATOR_NEW
		Type_fr_LF_upperleg_X_fr_LF_hip();
		const Type_fr_LF_upperleg_X_fr_LF_hip& update(const JState&);
	protected:
	};

	class Type_fr_LF_upperleg_X_fr_LF_lowerleg: public TransformForce<Scalar, Type_fr_LF_upperleg_X_fr_LF_lowerleg>
	{
	public:
		EIGEN_MAKE_ALIGNED_OPERATOR_NEW
		Type_fr_LF_upperleg_X_fr_LF_lowerleg();
		const Type_fr_LF_upperleg_X_fr_LF_lowerleg& update(const JState&);
	protected:
	};

	class Type_fr_LF_lowerleg_X_fr_LF_upperleg: public TransformForce<Scalar, Type_fr_LF_lowerleg_X_fr_LF_upperleg>
	{
	public:
		EIGEN_MAKE_ALIGNED_OPERATOR_NEW
		Type_fr_LF_lowerleg_X_fr_LF_upperleg();
		const Type_fr_LF_lowerleg_X_fr_LF_upperleg& update(const JState&);
	protected:
	};

	class Type_fr_torso_X_fr_RF_hip: public TransformForce<Scalar, Type_fr_torso_X_fr_RF_hip>
	{
	public:
		EIGEN_MAKE_ALIGNED_OPERATOR_NEW
		Type_fr_torso_X_fr_RF_hip();
		const Type_fr_torso_X_fr_RF_hip& update(const JState&);
	protected:
	};

	class Type_fr_RF_hip_X_fr_torso: public TransformForce<Scalar, Type_fr_RF_hip_X_fr_torso>
	{
	public:
		EIGEN_MAKE_ALIGNED_OPERATOR_NEW
		Type_fr_RF_hip_X_fr_torso();
		const Type_fr_RF_hip_X_fr_torso& update(const JState&);
	protected:
	};

	class Type_fr_torso_X_fr_RF_upperleg: public TransformForce<Scalar, Type_fr_torso_X_fr_RF_upperleg>
	{
	public:
		EIGEN_MAKE_ALIGNED_OPERATOR_NEW
		Type_fr_torso_X_fr_RF_upperleg();
		const Type_fr_torso_X_fr_RF_upperleg& update(const JState&);
	protected:
	};

	class Type_fr_RF_upperleg_X_fr_torso: public TransformForce<Scalar, Type_fr_RF_upperleg_X_fr_torso>
	{
	public:
		EIGEN_MAKE_ALIGNED_OPERATOR_NEW
		Type_fr_RF_upperleg_X_fr_torso();
		const Type_fr_RF_upperleg_X_fr_torso& update(const JState&);
	protected:
	};

	class Type_fr_torso_X_fr_RF_lowerleg: public TransformForce<Scalar, Type_fr_torso_X_fr_RF_lowerleg>
	{
	public:
		EIGEN_MAKE_ALIGNED_OPERATOR_NEW
		Type_fr_torso_X_fr_RF_lowerleg();
		const Type_fr_torso_X_fr_RF_lowerleg& update(const JState&);
	protected:
	};

	class Type_fr_RF_lowerleg_X_fr_torso: public TransformForce<Scalar, Type_fr_RF_lowerleg_X_fr_torso>
	{
	public:
		EIGEN_MAKE_ALIGNED_OPERATOR_NEW
		Type_fr_RF_lowerleg_X_fr_torso();
		const Type_fr_RF_lowerleg_X_fr_torso& update(const JState&);
	protected:
	};

	class Type_fr_torso_X_fr_RH_hip: public TransformForce<Scalar, Type_fr_torso_X_fr_RH_hip>
	{
	public:
		EIGEN_MAKE_ALIGNED_OPERATOR_NEW
		Type_fr_torso_X_fr_RH_hip();
		const Type_fr_torso_X_fr_RH_hip& update(const JState&);
	protected:
	};

	class Type_fr_RH_hip_X_fr_torso: public TransformForce<Scalar, Type_fr_RH_hip_X_fr_torso>
	{
	public:
		EIGEN_MAKE_ALIGNED_OPERATOR_NEW
		Type_fr_RH_hip_X_fr_torso();
		const Type_fr_RH_hip_X_fr_torso& update(const JState&);
	protected:
	};

	class Type_fr_torso_X_fr_RH_upperleg: public TransformForce<Scalar, Type_fr_torso_X_fr_RH_upperleg>
	{
	public:
		EIGEN_MAKE_ALIGNED_OPERATOR_NEW
		Type_fr_torso_X_fr_RH_upperleg();
		const Type_fr_torso_X_fr_RH_upperleg& update(const JState&);
	protected:
	};

	class Type_fr_RH_upperleg_X_fr_torso: public TransformForce<Scalar, Type_fr_RH_upperleg_X_fr_torso>
	{
	public:
		EIGEN_MAKE_ALIGNED_OPERATOR_NEW
		Type_fr_RH_upperleg_X_fr_torso();
		const Type_fr_RH_upperleg_X_fr_torso& update(const JState&);
	protected:
	};

	class Type_fr_torso_X_fr_RH_lowerleg: public TransformForce<Scalar, Type_fr_torso_X_fr_RH_lowerleg>
	{
	public:
		EIGEN_MAKE_ALIGNED_OPERATOR_NEW
		Type_fr_torso_X_fr_RH_lowerleg();
		const Type_fr_torso_X_fr_RH_lowerleg& update(const JState&);
	protected:
	};

	class Type_fr_RH_lowerleg_X_fr_torso: public TransformForce<Scalar, Type_fr_RH_lowerleg_X_fr_torso>
	{
	public:
		EIGEN_MAKE_ALIGNED_OPERATOR_NEW
		Type_fr_RH_lowerleg_X_fr_torso();
		const Type_fr_RH_lowerleg_X_fr_torso& update(const JState&);
	protected:
	};

	class Type_fr_torso_X_fr_LH_hip: public TransformForce<Scalar, Type_fr_torso_X_fr_LH_hip>
	{
	public:
		EIGEN_MAKE_ALIGNED_OPERATOR_NEW
		Type_fr_torso_X_fr_LH_hip();
		const Type_fr_torso_X_fr_LH_hip& update(const JState&);
	protected:
	};

	class Type_fr_LH_hip_X_fr_torso: public TransformForce<Scalar, Type_fr_LH_hip_X_fr_torso>
	{
	public:
		EIGEN_MAKE_ALIGNED_OPERATOR_NEW
		Type_fr_LH_hip_X_fr_torso();
		const Type_fr_LH_hip_X_fr_torso& update(const JState&);
	protected:
	};

	class Type_fr_torso_X_fr_LH_upperleg: public TransformForce<Scalar, Type_fr_torso_X_fr_LH_upperleg>
	{
	public:
		EIGEN_MAKE_ALIGNED_OPERATOR_NEW
		Type_fr_torso_X_fr_LH_upperleg();
		const Type_fr_torso_X_fr_LH_upperleg& update(const JState&);
	protected:
	};

	class Type_fr_LH_upperleg_X_fr_torso: public TransformForce<Scalar, Type_fr_LH_upperleg_X_fr_torso>
	{
	public:
		EIGEN_MAKE_ALIGNED_OPERATOR_NEW
		Type_fr_LH_upperleg_X_fr_torso();
		const Type_fr_LH_upperleg_X_fr_torso& update(const JState&);
	protected:
	};

	class Type_fr_torso_X_fr_LH_lowerleg: public TransformForce<Scalar, Type_fr_torso_X_fr_LH_lowerleg>
	{
	public:
		EIGEN_MAKE_ALIGNED_OPERATOR_NEW
		Type_fr_torso_X_fr_LH_lowerleg();
		const Type_fr_torso_X_fr_LH_lowerleg& update(const JState&);
	protected:
	};

	class Type_fr_LH_lowerleg_X_fr_torso: public TransformForce<Scalar, Type_fr_LH_lowerleg_X_fr_torso>
	{
	public:
		EIGEN_MAKE_ALIGNED_OPERATOR_NEW
		Type_fr_LH_lowerleg_X_fr_torso();
		const Type_fr_LH_lowerleg_X_fr_torso& update(const JState&);
	protected:
	};

	class Type_fr_torso_X_fr_LF_hip: public TransformForce<Scalar, Type_fr_torso_X_fr_LF_hip>
	{
	public:
		EIGEN_MAKE_ALIGNED_OPERATOR_NEW
		Type_fr_torso_X_fr_LF_hip();
		const Type_fr_torso_X_fr_LF_hip& update(const JState&);
	protected:
	};

	class Type_fr_LF_hip_X_fr_torso: public TransformForce<Scalar, Type_fr_LF_hip_X_fr_torso>
	{
	public:
		EIGEN_MAKE_ALIGNED_OPERATOR_NEW
		Type_fr_LF_hip_X_fr_torso();
		const Type_fr_LF_hip_X_fr_torso& update(const JState&);
	protected:
	};

	class Type_fr_torso_X_fr_LF_upperleg: public TransformForce<Scalar, Type_fr_torso_X_fr_LF_upperleg>
	{
	public:
		EIGEN_MAKE_ALIGNED_OPERATOR_NEW
		Type_fr_torso_X_fr_LF_upperleg();
		const Type_fr_torso_X_fr_LF_upperleg& update(const JState&);
	protected:
	};

	class Type_fr_LF_upperleg_X_fr_torso: public TransformForce<Scalar, Type_fr_LF_upperleg_X_fr_torso>
	{
	public:
		EIGEN_MAKE_ALIGNED_OPERATOR_NEW
		Type_fr_LF_upperleg_X_fr_torso();
		const Type_fr_LF_upperleg_X_fr_torso& update(const JState&);
	protected:
	};

	class Type_fr_torso_X_fr_LF_lowerleg: public TransformForce<Scalar, Type_fr_torso_X_fr_LF_lowerleg>
	{
	public:
		EIGEN_MAKE_ALIGNED_OPERATOR_NEW
		Type_fr_torso_X_fr_LF_lowerleg();
		const Type_fr_torso_X_fr_LF_lowerleg& update(const JState&);
	protected:
	};

	class Type_fr_LF_lowerleg_X_fr_torso: public TransformForce<Scalar, Type_fr_LF_lowerleg_X_fr_torso>
	{
	public:
		EIGEN_MAKE_ALIGNED_OPERATOR_NEW
		Type_fr_LF_lowerleg_X_fr_torso();
		const Type_fr_LF_lowerleg_X_fr_torso& update(const JState&);
	protected:
	};

	class Type_fr_torso_X_fr_RF_foot: public TransformForce<Scalar, Type_fr_torso_X_fr_RF_foot>
	{
	public:
		EIGEN_MAKE_ALIGNED_OPERATOR_NEW
		Type_fr_torso_X_fr_RF_foot();
		const Type_fr_torso_X_fr_RF_foot& update(const JState&);
	protected:
	};

	class Type_fr_RF_foot_X_fr_torso: public TransformForce<Scalar, Type_fr_RF_foot_X_fr_torso>
	{
	public:
		EIGEN_MAKE_ALIGNED_OPERATOR_NEW
		Type_fr_RF_foot_X_fr_torso();
		const Type_fr_RF_foot_X_fr_torso& update(const JState&);
	protected:
	};

	class Type_fr_torso_X_fr_RH_foot: public TransformForce<Scalar, Type_fr_torso_X_fr_RH_foot>
	{
	public:
		EIGEN_MAKE_ALIGNED_OPERATOR_NEW
		Type_fr_torso_X_fr_RH_foot();
		const Type_fr_torso_X_fr_RH_foot& update(const JState&);
	protected:
	};

	class Type_fr_RH_foot_X_fr_torso: public TransformForce<Scalar, Type_fr_RH_foot_X_fr_torso>
	{
	public:
		EIGEN_MAKE_ALIGNED_OPERATOR_NEW
		Type_fr_RH_foot_X_fr_torso();
		const Type_fr_RH_foot_X_fr_torso& update(const JState&);
	protected:
	};

	class Type_fr_torso_X_fr_LH_foot: public TransformForce<Scalar, Type_fr_torso_X_fr_LH_foot>
	{
	public:
		EIGEN_MAKE_ALIGNED_OPERATOR_NEW
		Type_fr_torso_X_fr_LH_foot();
		const Type_fr_torso_X_fr_LH_foot& update(const JState&);
	protected:
	};

	class Type_fr_LH_foot_X_fr_torso: public TransformForce<Scalar, Type_fr_LH_foot_X_fr_torso>
	{
	public:
		EIGEN_MAKE_ALIGNED_OPERATOR_NEW
		Type_fr_LH_foot_X_fr_torso();
		const Type_fr_LH_foot_X_fr_torso& update(const JState&);
	protected:
	};

	class Type_fr_torso_X_fr_LF_foot: public TransformForce<Scalar, Type_fr_torso_X_fr_LF_foot>
	{
	public:
		EIGEN_MAKE_ALIGNED_OPERATOR_NEW
		Type_fr_torso_X_fr_LF_foot();
		const Type_fr_torso_X_fr_LF_foot& update(const JState&);
	protected:
	};

	class Type_fr_LF_foot_X_fr_torso: public TransformForce<Scalar, Type_fr_LF_foot_X_fr_torso>
	{
	public:
		EIGEN_MAKE_ALIGNED_OPERATOR_NEW
		Type_fr_LF_foot_X_fr_torso();
		const Type_fr_LF_foot_X_fr_torso& update(const JState&);
	protected:
	};



public:
    ForceTransforms();
    void updateParameters();
	Type_fr_RF_hip_X_fr_RF_upperleg fr_RF_hip_X_fr_RF_upperleg;
	Type_fr_RF_upperleg_X_fr_RF_hip fr_RF_upperleg_X_fr_RF_hip;
	Type_fr_RF_upperleg_X_fr_RF_lowerleg fr_RF_upperleg_X_fr_RF_lowerleg;
	Type_fr_RF_lowerleg_X_fr_RF_upperleg fr_RF_lowerleg_X_fr_RF_upperleg;
	Type_fr_RH_hip_X_fr_RH_upperleg fr_RH_hip_X_fr_RH_upperleg;
	Type_fr_RH_upperleg_X_fr_RH_hip fr_RH_upperleg_X_fr_RH_hip;
	Type_fr_RH_upperleg_X_fr_RH_lowerleg fr_RH_upperleg_X_fr_RH_lowerleg;
	Type_fr_RH_lowerleg_X_fr_RH_upperleg fr_RH_lowerleg_X_fr_RH_upperleg;
	Type_fr_LH_hip_X_fr_LH_upperleg fr_LH_hip_X_fr_LH_upperleg;
	Type_fr_LH_upperleg_X_fr_LH_hip fr_LH_upperleg_X_fr_LH_hip;
	Type_fr_LH_upperleg_X_fr_LH_lowerleg fr_LH_upperleg_X_fr_LH_lowerleg;
	Type_fr_LH_lowerleg_X_fr_LH_upperleg fr_LH_lowerleg_X_fr_LH_upperleg;
	Type_fr_LF_hip_X_fr_LF_upperleg fr_LF_hip_X_fr_LF_upperleg;
	Type_fr_LF_upperleg_X_fr_LF_hip fr_LF_upperleg_X_fr_LF_hip;
	Type_fr_LF_upperleg_X_fr_LF_lowerleg fr_LF_upperleg_X_fr_LF_lowerleg;
	Type_fr_LF_lowerleg_X_fr_LF_upperleg fr_LF_lowerleg_X_fr_LF_upperleg;
	Type_fr_torso_X_fr_RF_hip fr_torso_X_fr_RF_hip;
	Type_fr_RF_hip_X_fr_torso fr_RF_hip_X_fr_torso;
	Type_fr_torso_X_fr_RF_upperleg fr_torso_X_fr_RF_upperleg;
	Type_fr_RF_upperleg_X_fr_torso fr_RF_upperleg_X_fr_torso;
	Type_fr_torso_X_fr_RF_lowerleg fr_torso_X_fr_RF_lowerleg;
	Type_fr_RF_lowerleg_X_fr_torso fr_RF_lowerleg_X_fr_torso;
	Type_fr_torso_X_fr_RH_hip fr_torso_X_fr_RH_hip;
	Type_fr_RH_hip_X_fr_torso fr_RH_hip_X_fr_torso;
	Type_fr_torso_X_fr_RH_upperleg fr_torso_X_fr_RH_upperleg;
	Type_fr_RH_upperleg_X_fr_torso fr_RH_upperleg_X_fr_torso;
	Type_fr_torso_X_fr_RH_lowerleg fr_torso_X_fr_RH_lowerleg;
	Type_fr_RH_lowerleg_X_fr_torso fr_RH_lowerleg_X_fr_torso;
	Type_fr_torso_X_fr_LH_hip fr_torso_X_fr_LH_hip;
	Type_fr_LH_hip_X_fr_torso fr_LH_hip_X_fr_torso;
	Type_fr_torso_X_fr_LH_upperleg fr_torso_X_fr_LH_upperleg;
	Type_fr_LH_upperleg_X_fr_torso fr_LH_upperleg_X_fr_torso;
	Type_fr_torso_X_fr_LH_lowerleg fr_torso_X_fr_LH_lowerleg;
	Type_fr_LH_lowerleg_X_fr_torso fr_LH_lowerleg_X_fr_torso;
	Type_fr_torso_X_fr_LF_hip fr_torso_X_fr_LF_hip;
	Type_fr_LF_hip_X_fr_torso fr_LF_hip_X_fr_torso;
	Type_fr_torso_X_fr_LF_upperleg fr_torso_X_fr_LF_upperleg;
	Type_fr_LF_upperleg_X_fr_torso fr_LF_upperleg_X_fr_torso;
	Type_fr_torso_X_fr_LF_lowerleg fr_torso_X_fr_LF_lowerleg;
	Type_fr_LF_lowerleg_X_fr_torso fr_LF_lowerleg_X_fr_torso;
	Type_fr_torso_X_fr_RF_foot fr_torso_X_fr_RF_foot;
	Type_fr_RF_foot_X_fr_torso fr_RF_foot_X_fr_torso;
	Type_fr_torso_X_fr_RH_foot fr_torso_X_fr_RH_foot;
	Type_fr_RH_foot_X_fr_torso fr_RH_foot_X_fr_torso;
	Type_fr_torso_X_fr_LH_foot fr_torso_X_fr_LH_foot;
	Type_fr_LH_foot_X_fr_torso fr_LH_foot_X_fr_torso;
	Type_fr_torso_X_fr_LF_foot fr_torso_X_fr_LF_foot;
	Type_fr_LF_foot_X_fr_torso fr_LF_foot_X_fr_torso;


}; // ForceTransforms






} // namespace tpl

using MotionTransforms = tpl::MotionTransforms<rbd::DoubleTrait>;
using ForceTransforms = tpl::ForceTransforms<rbd::DoubleTrait>;
using HomogeneousTransforms = tpl::HomogeneousTransforms<rbd::DoubleTrait>;



} // namespace pegasus2
} // namespace iit

#include "transforms.impl.h"

#endif // PEGASUS2_TRANSFORMS_H_
