#include "Robot.h"


//#define debuglvl1


Robot::Robot() : RigidBody()
{
	this->isRobot = true;
	robotParam = NULL;
}


Robot::Robot(const std::string& name_, int id_, bool isActive_) : RigidBody(name_, id_, isActive_)
{	
	this->isRobot = true;
	robotParam = NULL;
}

Robot::Robot(const se3& Pose_, const std::string& name_, int id_, bool isActive_) : RigidBody(Pose_, name_, id_, isActive_)
{	
	this->isRobot = true;
	robotParam = NULL;
}

Robot::Robot(const se3& Pose_, const std::string& name_, int id_, ShapeType shtype, bool isActive_) : RigidBody( Pose_, name_, id_, shtype, isActive_)
{	
	this->isRobot = true;
	robotParam = NULL;
}

Robot::Robot(const se3& Pose_, const Mat<float>& Lvel, const Mat<float>& Avel) : RigidBody( Pose_, Lvel,  Avel)
{
	this->isRobot = true;
	robotParam = NULL;
}


Robot::Robot(const std::string& name_, int id_, bool isActive_, const se3& Pose_) : RigidBody( name_, id_, isActive_, Pose_)
{
	this->isRobot = true;
	robotParam = NULL;
}

Robot::Robot(const std::string& name_, int id_, bool isActive_, const se3& Pose_, const Mat<float>& Lvel, const Mat<float>& Avel) : RigidBody(name_, id_, isActive_, Pose_, Lvel, Avel)
{
	this->isRobot = true;
	robotParam = NULL;
}

Robot::~Robot()
{
	if(robotParam != NULL)
	{
		delete robotParam;
	}
}



//-----------------------------------------------
//-----------------------------------------------
//-----------------------------------------------
//-----------------------------------------------




void Robot::process( float timeStep, bool& isComputing)
{
	this->getPosition().afficher();
	this->getLinearVelocity().afficher();
	this->getAngularVelocity().afficher();
	
	//Computing phiNext :
	//TODO :
	float phiNext = PI/4.0f;
	
	//linear and angular feedback variables : 
	float kv = 0.5f;
	float kw = 0.5f;
	
	//Convergence Rates :
	float cvrateSigma = 0.9f;
	float cvrateA = 0.9f;
	
	//Cycle Parameters :
	float desiredR = 10.0f;
	float couplingStrength = 0.1f;
	
	
	Mat<float> centerOfRotation(50.0f, 3,1);
	centerOfRotation.set( 0.0f, 3,1);
	float cx = centerOfRotation.get(1,1);
	float cy = centerOfRotation.get(2,1);
	float cz = centerOfRotation.get(3,1);
	
	//Global frame coordinate system :
	Mat<float> currentPosition(this->getPosition());
	float x = currentPosition.get(1,1);
	float y = currentPosition.get(2,1);
	float z = currentPosition.get(3,1);
	

	//Coordinates in the frame related to the center of the cycle : center == origin.
	float xC = x-cx;
	float yC = y-cy;
	float zC = z-cz;
	
	//Computing theta :
	float robotHeadingX,robotHeadingY,robotHeadingZ;
	Qt2Euler( this->getOrientation(), &robotHeadingX, &robotHeadingY, &robotHeadingZ);
	float robotAngleZ = atan21(yC,xC);
	float theta = robotHeadingZ - robotAngleZ;
	
	
	float currentR = sqrt( pow(xC,2)+pow(yC,2) );
	
	float Rdot = cvrateA * ( 1 - pow(currentR,2)/pow(desiredR,2) );
	float kapla = cvrateSigma + couplingStrength*sin( phiNext );
	
	//Robot polar velocities :
	float lvR = kv * ( Rdot * cos( theta ) + currentR * kapla * sin( theta ) );
	float avR = kw * ( currentR * kapla * cos( theta ) - Rdot * sin( theta ) );
	
	//Global XY velocities :
	Mat<float> lv(0.0f,3,1);
	Mat<float> av(0.0f,3,1);
	
	//1) let us cancel the gravity :
	lv.set( 9.83f*timeStep*this->getMass(), 3,1);
	//2) let us implement the control law :
	lv.set( lvR*cos(robotAngleZ), 1,1);
	lv.set( lvR*sin(robotAngleZ), 2,1);
	
	av.set(avR, 3,1);
	
	if( isnanM(lv) )
	{
		//TODO: ?
	}
	else
	{
		this->setLinearVelocity(lv);
	}
	
	if(isnanM(av) )
	{
		//TODO : ?
	}
	else
	{
		this->setAngularVelocity( av);
	}
	
	
	std::cout << "ROBOT : " << this->name << " : UPDATED." << std::endl;
	
	this->getPosition().afficher();
	this->getLinearVelocity().afficher();
	this->getAngularVelocity().afficher();
}




