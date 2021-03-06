#include "RigidBody.h"


//#define debuglvl1
Mat<float> one(1.0f,1,1);


RigidBody::RigidBody() : ISimulationObject(), IMoveable(), userForce( Mat<float>((float)0,3,1) ), userTorque(Mat<float>((float)0,3,1)), isFixed(false), canCollide(true), mass(1.0f), imass(1.0f), Inertia(Identity3), iInertia(Identity3), ptrShape(new SphereShape(this)),isRobot(false)
{
	type = TSORigidBody;
}


RigidBody::RigidBody(const std::string& name_, int id_, bool isActive_) : ISimulationObject(name_,id_,isActive_), IMoveable(), userForce( Mat<float>((float)0,3,1) ), userTorque(Mat<float>((float)0,3,1)), isFixed(false), canCollide(true), mass(1.0f), imass(1.0f), Inertia(Identity3), iInertia(Identity3), ptrShape(new SphereShape(this)),isRobot(false)
{	
	type = TSORigidBody;
}

RigidBody::RigidBody(const se3& Pose_, const std::string& name_, int id_, bool isActive_) : ISimulationObject(name_,id_,isActive_), IMoveable(Pose_), userForce( Mat<float>((float)0,3,1) ), userTorque(Mat<float>((float)0,3,1)), isFixed(false), canCollide(true), mass(1.0f), imass(1.0f), Inertia(Identity3), iInertia(Identity3), ptrShape(new SphereShape(this)),isRobot(false)
{	
	type = TSORigidBody;
}

RigidBody::RigidBody(const se3& Pose_, const std::string& name_, int id_, ShapeType shtype, bool isActive_) : ISimulationObject(name_,id_,isActive_), IMoveable(Pose_), userForce( Mat<float>((float)0,3,1) ), userTorque(Mat<float>((float)0,3,1)), isFixed(false), canCollide(true), mass(1.0f), imass(1.0f), Inertia(Identity3), iInertia(Identity3),isRobot(false)
{	

	if(shtype == BOX)
	{
		//ptrShape = std::unique_ptr<IShape>(new BoxShape(this));
		ptrShape = new BoxShape(this);
		computeInertia();
	}
	else
	{
		//ptrShape = std::unique_ptr<IShape>(new SphereShape(this));
		ptrShape = new SphereShape(this);
	}
	
	type = TSORigidBody;
}

RigidBody::RigidBody(const se3& Pose_, const Mat<float>& Lvel, const Mat<float>& Avel) : ISimulationObject(), IMoveable(Pose_,Lvel,Avel), userForce(Mat<float>((float)0,3,1) ), userTorque(Mat<float>((float)0,3,1) ), isFixed(false), canCollide(true), mass(1.0f), imass(1.0f), Inertia(Identity3), iInertia(Identity3), ptrShape(new SphereShape(this)),isRobot(false)
{
	type = TSORigidBody;
}


RigidBody::RigidBody(const std::string& name_, int id_, bool isActive_, const se3& Pose_) : ISimulationObject(name_,id_,isActive_), IMoveable(Pose_),userForce(Mat<float>((float)0,3,1) ), userTorque(Mat<float>((float)0,3,1) ), isFixed(false), canCollide(true), mass(1.0f), imass(1.0f), Inertia(Identity3), iInertia(Identity3), ptrShape(new SphereShape(this)),isRobot(false)
{
	type = TSORigidBody;
}

RigidBody::RigidBody(const std::string& name_, int id_, bool isActive_, const se3& Pose_, const Mat<float>& Lvel, const Mat<float>& Avel) : ISimulationObject(name_,id_,isActive_), IMoveable(Pose_,Lvel,Avel),userForce(Mat<float>((float)0,3,1) ), userTorque(Mat<float>((float)0,3,1) ), isFixed(false), canCollide(true), mass(1.0f), imass(1.0f), Inertia(Identity3), iInertia(Identity3), ptrShape(new SphereShape(this)),isRobot(false)
{
	type = TSORigidBody;
}


/*	
RigidBody::RigidBody(const std::string& name_, int id_, bool isActive_, const se3& Pose_, std::unique_ptr<IShape> ptrShape_) : ISimulationObject(name_,id_,isActive_), IMoveable(Pose_), userForce(Mat<float>((float)0,3,1) ), userTorque(Mat<float>((float)0,3,1) ), isFixed(false), canCollide(true), mass(1.0f), imass(1.0f), Inertia(Identity3), iInertia(Identity3), ptrShape(std::move(ptrShape_))
{
	type = TSORigidBody;
}

RigidBody::RigidBody(const std::string& name_, int id_, bool isActive_, const se3& Pose_, const Mat<float>& Lvel, const Mat<float>& Avel, std::unique_ptr<IShape> ptrShape_) : ISimulationObject(name_,id_,isActive_), IMoveable(Pose_,Lvel,Avel),userForce(Mat<float>((float)0,3,1) ), userTorque(Mat<float>((float)0,3,1) ), isFixed(false), canCollide(true), mass(1.0f), imass(1.0f), Inertia(Identity3), iInertia(Identity3), ptrShape( std::move(ptrShape_))
{
	type = TSORigidBody;
}
*/

RigidBody::~RigidBody()
{
	delete ptrShape;
}



//TODO : implantation
void RigidBody::Render(const se3& WorldTransformation)
{

}


void RigidBody::computeInertia()
{
	//it is assumed that the ptrShape is initialized.
	switch(ptrShape->getShapeType())
	{
		case BOX :
		{
		float lx = ((BoxShape&)(*ptrShape)).getHeight();
		float ly = ((BoxShape&)(*ptrShape)).getWidth();
		float lz = ((BoxShape&)(*ptrShape)).getDepth();
		
		Inertia = Mat<float>(0.0f,3,3);
		Inertia.set( (mass/12)*(ly*ly+lz*lz), 1,1); 
		Inertia.set( (mass/12)*(lx*lx+lz*lz), 2,2);
		Inertia.set( (mass/12)*(ly*ly+lx*lx), 3,3);
		
		iInertia = Inertia;
		iInertia.set( 1.0f/iInertia.get(1,1), 1,1);
		iInertia.set( 1.0f/iInertia.get(2,2), 2,2);
		iInertia.set( 1.0f/iInertia.get(3,3), 3,3);
		
		transformInertiaTensorL2W();
		}
		break;
		
		case SPHERE :
		{
			float r = ((SphereShape&)(*ptrShape)).getBRadius();
			r = 2.0f*mass*r*r/5.0f;
			
			Inertia = Mat<float>(0.0f,3,3);
			Inertia.set( r, 1,1); 
			Inertia.set( r, 2,2);
			Inertia.set( r, 3,3);
			
			iInertia = Inertia;
			iInertia.set( 1.0f/r, 1,1); 
			iInertia.set( 1.0f/r, 2,2);
			iInertia.set( 1.0f/r, 3,3);
			
			transformInertiaTensorL2W();
			
		}
		break;
		
		default :
		{
		//TODO....
		}
		break;
	}
}

Mat<float> RigidBody::transformInertiaTensorL2W()
{
	iInertiaWorld =  extract( Pose->exp(), 1,1, 3,3);
	iInertiaWorld = transpose(iInertiaWorld)*(iInertia*iInertiaWorld);
	return iInertiaWorld;
}


void RigidBody::addForce(const Mat<float>& force)
{
	userForce += force;
}

void RigidBody::addForceAtWorldPoint(const Mat<float>& force, const Mat<float>& pointW)
{
	addForce(force);
	addTorque( crossproductV( pointW-extract(Pose->exp(), 1,4, 3,4), force) );		
}

void RigidBody::addForceAtBodyPoint(const Mat<float>& force, const Mat<float>& pointL)
{
	/*
	addForce(force);
	addTorque( crossproductV( pointW-, force) );
	*/
	Mat<float> pointW( this->getPointInWorld(pointL) );
	Mat<float> forceW( this->getVectorInWorld(force) );
	this->addForceAtWorldPoint(forceW, pointW);
}

void RigidBody::addForceLocalAtBodyPoint(const Mat<float>& forceL, const Mat<float>& pointL)
{
	Mat<float> pointW( this->getPointInWorld(pointL) );
	Mat<float> forceW( this->getVectorInWorld(forceL) );
	this->addForceAtWorldPoint(forceW, pointW);
}

void RigidBody::addForceWorldAtBodyPoint(const Mat<float>& forceW, const Mat<float>& pointL)
{
	Mat<float> pointW( this->getPointInWorld(pointL) );
	this->addForceAtWorldPoint(forceW, pointW);
}
void RigidBody::addTorque(const Mat<float>& torque)
{
	userTorque += torque;
}

void RigidBody::clearUser()
{
	userForce *= 0.0f;
	userTorque *= 0.0f;
}


void RigidBody::calculateDerivedData()
{
	//Derived the Inertial Tensor in the world coordinate frame :
	transformInertiaTensorL2W();

}

Mat<float> RigidBody::getPointInWorld( const Mat<float>& pointL)
{
	Mat<float> transM_L2W(Pose->getTransformationL2W());
	//return transpose( extract(Pose->exp(), 1,1, 3,3)) * (pointL+extract(Pose->exp(), 1,4, 3,4) );	
	//return transpose( extract(pose, 1,1, 3,3)) * (pointL)+Pose->getT();	
	return extract( transM_L2W*operatorC(pointL,one), 1,1, 3,1);
}

Mat<float> RigidBody::getVectorInWorld( const Mat<float>& vectorL)
{
	return transpose( extract(Pose->exp(), 1,1, 3,3)) * (vectorL);	
}

Mat<float> RigidBody::getPointInLocal( const Mat<float>& pointW)
{
	Mat<float> transM_W2L(Pose->getTransformationW2L());
	return extract( transM_W2L*operatorC(pointW,one), 1,1, 3,1);
}


Mat<float> RigidBody::getVelocityPointWInWorld( const Mat<float>& pointW)
{
	return this->getLinearVelocity()+crossproductV(this->getAngularVelocity(),this->getPosition()-pointW);	
}

Mat<float> RigidBody::getAxisInWorld( const Mat<float>& aL)
{
	//return transpose( extract(Pose->exp(), 1,1, 3,3)) * aL;
	Mat<float> R_L2W( extract( Pose->getTransformationL2W(), 1,1, 3,3) );
	return R_L2W * aL;	
}

Mat<float> RigidBody::getAxisInLocal( const Mat<float>& aW)
{
	//return extract(Pose->exp(), 1,1, 3,3) * aW;
	Mat<float> R_W2L( extract( Pose->getTransformationW2L(), 1,1, 3,3) );
	return R_W2L * aW;	
}


Mat<float> RigidBody::getForceAccumulator() const
{
	return userForce;
}

Mat<float> RigidBody::getTorqueAccumulator() const
{ 	
	return userTorque;
}


