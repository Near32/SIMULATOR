#include "IIntegrator.h"
#include "../Simulation.h"

//#define NORMALIZE_QUAT


IIntegrator::IIntegrator(Simulation* sim) : sim(sim)
{

}

IIntegrator::~IIntegrator()
{

}

//---------------------------------------------
//---------------------------------------------

//---------------------------------------------
//---------------------------------------------

ExplicitEulerIntegrator::ExplicitEulerIntegrator(Simulation* sim) : IIntegrator(sim)
{

}

ExplicitEulerIntegrator::~ExplicitEulerIntegrator()
{

}	

/*
void ExplicitEulerIntegrator::integrate(float dt)
{
	assert(dt > 0.0f);
	
	//int nbrSimulatedObjects = sim->simulatedObjects.size();
	std::vector<std::unique_ptr<ISimulationObject> >::iterator it = sim->simulatedObjects.begin();
	
	while( it != sim->simulatedObjects.end())
	{
		if( (*it)->getType() == TSORigidBody)
		{
			((RigidBody&)(*(*it))).calculateDerivedData();
			
			
			//Linear :
			Mat<float> resultingLinearAcceleration(  ((RigidBody&)(*(*it))).getIMass() * ((RigidBody&)(*(*it))).getForceAccumulator());
	
			((RigidBody&)(*(*it))).setLinearVelocity( ((RigidBody&)(*(*it))).getLinearVelocity() + dt*resultingLinearAcceleration);
	
			//velocity *= pow( damping, duration);
			
			((RigidBody&)(*(*it))).setPosition( ((RigidBody&)(*(*it))).getPosition() + dt*((RigidBody&)(*(*it))).getLinearVelocity());
	
			//Angular :
			Mat<float> resultingAngularAcceleration(  ((RigidBody&)(*(*it))).getInverseInertialWorld() * ((RigidBody&)(*(*it))).getTorqueAccumulator());
	
			((RigidBody&)(*(*it))).setAngularVelocity( ((RigidBody&)(*(*it))).getAngularVelocity() + dt*resultingAngularAcceleration);
	
			//velocity *= pow( damping, duration);
			
			//----------------------------------------------
			//Quaternion : via Euler Angles...
			//Mat<float> deltaQEuler(dt*getAngularVelocity);
			//it->setOrientation( Qt_Mul( Euler2Qt( deltaQEuler.get(1,1), deltaQEuler.get(2,1), deltaQEuler.get(3,1)), it->getOrientation() ) );
			//----------------------------------------------
			//----------------------------------------------
			//Quaternion : via qrond matrix : watch out for the quaternion definition : here : (x y z w)^T...
			Mat<float> qrond(4,3);
			Quat q( ((RigidBody&)(*(*it))).getOrientation() );
			qrond.set( q.w, 1,1);
			qrond.set( q.w, 2,2);
			qrond.set( q.w, 3,3);
			
			qrond.set( q.x, 2,3);
			qrond.set( -q.x, 3,2);
			qrond.set( -q.x, 4,1);
			
			qrond.set( q.y, 3,1);
			qrond.set( -q.y, 1,3);
			qrond.set( -q.y, 4,2);
			
			qrond.set( q.z, 1,2);
			qrond.set( -q.z, 2,1);
			qrond.set( -q.z, 4,3);
			
			((RigidBody&)(*(*it))).setOrientation( ((RigidBody&)(*(*it))).getOrientation()+Mat2Qt( dt * (qrond* ((RigidBody&)(*(*it))).getAngularVelocity()) ) );
			//----------------------------------------------
			
			//-------------------
			((RigidBody&)(*(*it))).clearUser();
		}
	}
}
*/

void ExplicitEulerIntegrator::integrateVelocities(float dt)
{
	assert(dt > 0.0f);
	
	//int nbrSimulatedObjects = sim->simulatedObjects.size();
	std::vector<std::unique_ptr<ISimulationObject> >::iterator it = sim->simulatedObjects.begin();
	
	for( auto& it : sim->simulatedObjects)
	{
		if( it->getType() == TSORigidBody)
		{
			((RigidBody*)(it.get()))->calculateDerivedData();
			
			
			//Linear :
			Mat<float> resultingLinearAcceleration(  ((RigidBody*)(it.get()))->getIMass() * ((RigidBody*)(it.get()))->getForceAccumulator());
	
			((RigidBody*)(it.get()))->setLinearVelocity( ((RigidBody*)(it.get()))->getLinearVelocity() + dt*resultingLinearAcceleration);
	
			//velocity *= pow( damping, duration);
			
			//--------------------------------------
			//--------------------------------------
			//--------------------------------------
			
										
			//Angular :
			Mat<float> resultingAngularAcceleration(  ((RigidBody*)(it.get()))->getInverseInertialWorld() * ((RigidBody*)(it.get()))->getTorqueAccumulator());
	
			((RigidBody*)(it.get()))->setAngularVelocity( ((RigidBody*)(it.get()))->getAngularVelocity() + dt*resultingAngularAcceleration);
			//velocity *= pow( damping, duration);
		}
	}
	
}


void ExplicitEulerIntegrator::integratePositions(float dt)
{
	assert(dt > 0.0f);
	
	//int nbrSimulatedObjects = sim->simulatedObjects.size();
	std::vector<std::unique_ptr<ISimulationObject> >::iterator it = sim->simulatedObjects.begin();
	
	for( auto& it : sim->simulatedObjects)
	{
		if( it->getType() == TSORigidBody)
		{
			//------------------------------------------------
			//Linear :
			((RigidBody*)(it.get()))->setPosition( ((RigidBody*)(it.get()))->getPosition() + dt * ((RigidBody*)(it.get()))->getLinearVelocity());
			
			
			//Angular :
			//----------------------------------------------
			//Quaternion : via Euler Angles...
			//Mat<float> deltaQEuler(dt*getAngularVelocity);
			//it->setOrientation( Qt_Mul( Euler2Qt( deltaQEuler.get(1,1), deltaQEuler.get(2,1), deltaQEuler.get(3,1)), it->getOrientation() ) );
			//----------------------------------------------
			//----------------------------------------------
			//Quaternion : via qrond matrix : watch out for the quaternion definition : here : (x y z w)^T...
			Mat<float> qrond(4,3);
			Quat q( ((RigidBody*)(it.get()))->getOrientation() );
			
			//----------------------------
			//let us normalize q :
			#ifdef NORMALIZE_QUAT
			if(q.w != 0.0f)
			{
				q.x /= q.w;
				q.y /= q.w;
				q.z /= q.w;
				q.w = 1.0f;
			}
			#endif
			//---------------------------
			
			//METHOD 1 :
			/*
			qrond.set( q.w, 1,1);
			qrond.set( q.w, 2,2);
			qrond.set( q.w, 3,3);
			
			qrond.set( q.x, 2,3);
			qrond.set( -q.x, 3,2);
			qrond.set( -q.x, 4,1);
			
			qrond.set( q.y, 3,1);
			qrond.set( -q.y, 1,3);
			qrond.set( -q.y, 4,2);
			
			qrond.set( q.z, 1,2);
			qrond.set( -q.z, 2,1);
			qrond.set( -q.z, 4,3);
			
			Quat res( ((RigidBody*)(it.get()))->getOrientation()+Mat2Qt( dt * (qrond* ((RigidBody*)(it.get()))->getAngularVelocity()) ) );
			*/
			
			
			//METHOD 2 : BARAFF rigid Body 1 
			Quat av;
			Mat<float> avv( ((RigidBody*)(it.get()))->getAngularVelocity() );
			av.x = avv.get(1,1);
			av.y = avv.get(2,1);
			av.z = avv.get(3,1);
			av.w = 0;
			
			Quat res( q+0.5f * Qt_Mul( av, q) );
			
			//----------------------------
			
			//std::cout << " INTEGRATOR : BEFORE : " << it->getName() << std::endl;
			//transpose( ((RigidBody*)(it.get()))->getMatOrientation() ).afficher();
			float roll,pitch,yaw;
			transpose( Qt2Mat<float>(q) ).afficher();
			Qt2Euler( ((RigidBody*)(it.get()))->getOrientation(), &roll, &pitch, &yaw);
			
			/*
			std::cout << " r p y : " << roll << " ; " << pitch << " ; " << yaw << std::endl;
			std::cout << " VEL : " << std::endl;
			transpose( avv ).afficher();
			*/
			
			roll += avv.get(1,1)*dt;
			pitch += avv.get(2,1)*dt;
			yaw += avv.get(3,1)*dt;
			
			//std::cout << " after vel : r p y : " << roll << " ; " << pitch << " ; " << yaw << std::endl;
			
			
			
			//let us normalize res :
			#ifdef NORMALIZE_QUAT
			if(res.w != 0.0f)
			{
				res.x /= res.w;
				res.y /= res.w;
				res.z /= res.w;
				res.w = 1.0f;
			}
			#endif
			//---------------------------
			
			((RigidBody*)(it.get()))->setOrientation( res );
			//----------------------------------------------

			/*
			std::cout << " INTEGRATOR : " << it->getName() << std::endl;
			transpose( ((RigidBody*)(it.get()))->getMatOrientation() ).afficher();
			Qt2Euler( ((RigidBody*)(it.get()))->getOrientation(), &roll, &pitch, &yaw);
			std::cout << " r p y : " << roll << " ; " << pitch << " ; " << yaw << std::endl;
			*/
							
			//-------------------
			((RigidBody*)(it.get()))->clearUser();
		}
	}
}


/*
void ExplicitEulerIntegrator::integratePositions(float dt)
{
	assert(dt > 0.0f);
	
	//int nbrSimulatedObjects = sim->simulatedObjects.size();
	std::vector<std::unique_ptr<ISimulationObject> >::iterator it = sim->simulatedObjects.begin();
	
	for( auto& it : sim->simulatedObjects)
	{
		if( it->getType() == TSORigidBody)
		{
			//------------------------------------------------
			//Linear :
			((RigidBody*)(it.get()))->setPosition( ((RigidBody*)(it.get()))->getPosition() + dt * ((RigidBody*)(it.get()))->getLinearVelocity());
			
			
			//Angular :
			Mat<float> av( ((RigidBody*)(it.get()))->getAngularVelocity() );
			Quat q( ((RigidBody*)(it.get()))->getOrientation() );
			
			//----------------------------
			//let us normalize q :
			#ifdef NORMALIZE_QUAT
			if(q.w != 0.0f)
			{
				q.x /= q.w;
				q.y /= q.w;
				q.z /= q.w;
				q.w = 1.0f;
			}
			#endif
			//---------------------------
			
			float roll,pitch,yaw;
			Qt2Euler(q, &roll, &pitch, &yaw);
			
			
			std::cout << " INTEGRATOR : BEFORE : " << it->getName() << std::endl;
			//transpose( ((RigidBody*)(it.get()))->getMatOrientation() ).afficher();
			transpose( Qt2Mat<float>(q) ).afficher();
			Qt2Euler( ((RigidBody*)(it.get()))->getOrientation(), &roll, &pitch, &yaw);
			std::cout << " r p y : " << roll << " ; " << pitch << " ; " << yaw << std::endl;
			std::cout << " VEL : " << std::endl;
			transpose( av ).afficher();
			
			roll += av.get(1,1)*dt;
			pitch += av.get(2,1)*dt;
			yaw += av.get(3,1)*dt;
			
			std::cout << " after vel : r p y : " << roll << " ; " << pitch << " ; " << yaw << std::endl;
			
			
			Quat res( Euler2Qt(roll,pitch,yaw) );
			
			//----------------------------
			//let us normalize res :
			#ifdef NORMALIZE_QUAT
			if(res.w != 0.0f)
			{
				res.x /= res.w;
				res.y /= res.w;
				res.z /= res.w;
				res.w = 1.0f;
			}
			#endif
			//---------------------------		
			
			
			((RigidBody*)(it.get()))->setOrientation( res );
			//----------------------------------------------
			
			std::cout << " INTEGRATOR : " << it->getName() << std::endl;
			transpose( ((RigidBody*)(it.get()))->getMatOrientation() ).afficher();
			Qt2Euler( ((RigidBody*)(it.get()))->getOrientation(), &roll, &pitch, &yaw);
			std::cout << " r p y : " << roll << " ; " << pitch << " ; " << yaw << std::endl;
			//-------------------
			((RigidBody*)(it.get()))->clearUser();
		}
	}
}
*/



