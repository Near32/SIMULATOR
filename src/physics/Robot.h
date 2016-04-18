#ifndef ROBOT_H
#define ROBOT_H

#include "RigidBody.h"



struct RobotParamP1
{
	public :
	
	unsigned int idxSWARM;
};

typedef struct RobotParamP1 IRobotParam;


class Robot : public RigidBody
{
	public :
	
	//--------------------------------------------------------
	//--------------------------------------------------------
	
	//deprecated constructor-functions :
	Robot();
	Robot(const std::string& name_, int id_, bool isActive_);
	Robot(const se3& Pose_, const std::string& name_, int id_, bool isActive_ = true);
	Robot(const se3& Pose_, const Mat<float>& Lvel, const Mat<float>& Avel);
	//do not use....
	
	
	//preferred constructor-functions :
	Robot(const se3& Pose_, const std::string& name_, int id_, ShapeType shtype, bool isActive_ = true);
	Robot(const std::string& name_, int id_, bool isActive_, const se3& Pose_);
	Robot(const std::string& name_, int id_, bool isActive_, const se3& Pose_, const Mat<float>& Lvel, const Mat<float>& Avel);
		
	~Robot();
	
	
	void process( float timeStep, bool& isComputing);
	// function used to handle the control law of the robot in velocity.
	
		
	protected :
	
	IRobotParam* robotParam;
	
};



#endif
