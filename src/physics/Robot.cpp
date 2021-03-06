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
	//Computing phiNext :
	//TODO :
	
	/*
	float phiNext = PI/8.0f;
	
	//linear and angular feedback variables : 
	float kv = 0.5f;
	float kw = 0.5f;
	
	//Convergence Rates :
	float cvrateSigma = 0.9f;
	float cvrateA = 0.9f;
	
	//Cycle Parameters :
	float desiredR = 1.0f;
	float couplingStrength = 0.1f;
	
	
	Mat<float> centerOfRotation(5.0f, 3,1);
	centerOfRotation.set( this->getPosition().get(3,1), 3,1);
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
	
	std::cout << " ROBOT ANGLE Z : " << robotHeadingZ * 180.0f/PI << std::endl;
	
	float currentR = sqrt( pow(xC,2)+pow(yC,2) );
	
	float Rdot = cvrateA * ( 1 - pow(currentR,2)/pow(desiredR,2) );
	float kapla = cvrateSigma + couplingStrength*sin( phiNext );
	*/
	//Robot polar velocities :
	//float lvR = kv * ( Rdot * cos( theta ) + currentR * kapla * sin( theta ) );
	float lvR = 1.0f;
	//float avR = kw * ( currentR * kapla * cos( theta ) - Rdot * sin( theta ) );
	float avR = PI/8;//2*PI*10;
	
	//Global XY velocities :
	Mat<float> lv(0.0f,3,1);
	Mat<float> av(0.0f,3,1);
	float scaler = 1e0f;
	
	//1) let us cancel the gravity :
	//UNIT : Newton*s
	//lv.set( 9.83f*timeStep*this->getMass(), 3,1);
	//2) let us implement the control law :
	
	//lv.set( scaler*lvR*cos(robotAngleZ), 1,1);
	//lv.set( (-1.0f)*scaler*sin(robotAngleZ), 1,1);
	lv.set( lvR, 1,1);
	//lv.set( scaler*lvR*sin(robotAngleZ), 2,1);
	//lv.set( scaler*cos(robotAngleZ), 2,1);
	lv.set( lvR, 2,1);
	
	//UNIT : RADIANS/s :
	//av.set(avR, 1,1);
	//av.set(avR, 2,1);
	av.set(avR, 3,1);
	
	
	if( isnanM(lv) )
	{
		//TODO: ?
	}
	else
	{
		this->setLinearVelocity(lv);
		//this->getLinearVelocity() += lv;
		//this->addForce( (this->getMass()/timeStep)*lv);
	}
	
	if(isnanM(av) )
	{
		//TODO : ?
	}
	else
	{
		this->setAngularVelocity( av);
		//this->getAngularVelocity() += av;
		//this->addTorque( (1.0f/timeStep)*(this->getInertialWorld()*av) );
	}
	
#ifdef debuglvl1	
	std::cout << "ROBOT : " << this->name << " : UPDATED." << std::endl;
	
	//this->getPosition().afficher();
	this->getPose().getSE3().afficher();
	this->getLinearVelocity().afficher();
	this->getAngularVelocity().afficher();
#endif
}




