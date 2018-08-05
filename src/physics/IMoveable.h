#ifndef IMOVEABLE_H
#define IMOVEABLE_H

#include "../utils/math.h"
#include <exception>

#define default_linearDampingCoefficient 0.999
#define default_angularDampingCoefficient 0.999

class IMoveable
{
	protected :
	
	se3* Pose;
	
	Mat<float>* LinearVelocity;
	Mat<float>* AngularVelocity;
	/**
	 * Amount of damping applied to angular velocity
	 * in order to remove numerical instabality-based energy
	 */
	float linearDampingCoefficient;
	float angularDampingCoefficient;
	
//-----------------------------------------------------
//-----------------------------------------------------
	public :
	
	
	IMoveable();
	IMoveable( const se3& Pose_);													//Orientation has to be initialized from Pose.
	IMoveable( const se3& Pose_, const Mat<float>& Lvel, const Mat<float>& Avel);	//Orientation has to be initialized from Pose.
	
	virtual ~IMoveable();
	
	
	//---------------------------------------------------------
	//---------------------------------------------------------
	
		
	se3 getPose() const	;
	
	Mat<float> getPosition();
		
	Quat getOrientation();
	
	Mat<float> getMatOrientation();
	
	Mat<float> getTransformation()	const;
	
	Mat<float> getLinearVelocity()	const;
	
	Mat<float> getAngularVelocity()	const;


	Mat<float>& getLinearVelocity();
	
	Mat<float>& getAngularVelocity();

	const float& getLinearDampingCoefficient() const
	{	
		return this->linearDampingCoefficient;
	}

	const float& getAngularDampingCoefficient() const
	{	
		return this->angularDampingCoefficient;
	}

	
	//---------------------------------------------------------
	//---------------------------------------------------------
	
	void setPose( const se3& pose_);
	void setPosition( const Mat<float>& t_);
	void setOrientation( const Quat& q);
	void setMatOrientation( const Mat<float>& q);
	void setW( const Mat<float>& w);
	void setLinearVelocity( const Mat<float>& lvel);
	void setAngularVelocity( const Mat<float>& avel);
	
};

#endif
