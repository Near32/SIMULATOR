#include "IIntegrator.h"
#include "../Simulation.h"

#define NORMALIZE_QUAT
#define damping 
#define dampingAfter

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
		if( it->getType() == TSORigidBody && ((RigidBody*)(it.get()))->getFixedStatus() == false)
		{
			((RigidBody*)(it.get()))->calculateDerivedData();
			
			
			//Linear :
			Mat<float> resultingLinearAcceleration(  ((RigidBody*)(it.get()))->getIMass() * ((RigidBody*)(it.get()))->getForceAccumulator());
			Mat<float> resultingLinearVelocity(  ((RigidBody*)(it.get()))->getLinearVelocity() + dt*resultingLinearAcceleration);
			
			std::cout << "RigidBody : " << it->getName() << std::endl;
			std::cout << " Linear Velocity :: before update " << std::endl;
			((RigidBody*)(it.get()))->getLinearVelocity().afficher();
			/*
			std::cout << " Resulting Linear Velocity :: before damping " << std::endl;
			resultingLinearVelocity.afficher();
			*/
			((RigidBody*)(it.get()))->setLinearVelocity( resultingLinearVelocity );
			

			#ifdef damping
			// Linear Damping :
			resultingLinearVelocity *= (float) pow( ((RigidBody*)(it.get()))->getLinearDampingCoefficient(), dt);
			((RigidBody*)(it.get()))->setLinearVelocity( resultingLinearVelocity );
			#endif
						
			//--------------------------------------
			//--------------------------------------
			//--------------------------------------
			
										
			//Angular :
			Mat<float> resultingAngularAcceleration(  ((RigidBody*)(it.get()))->getInverseInertialWorld() * ((RigidBody*)(it.get()))->getTorqueAccumulator());
			Mat<float> resultingAngularVelocity( ((RigidBody*)(it.get()))->getAngularVelocity() + dt*resultingAngularAcceleration);
			((RigidBody*)(it.get()))->setAngularVelocity( resultingAngularVelocity );

			#ifdef damping
			// Angular Damping :
			resultingAngularVelocity *= (float) pow( ((RigidBody*)(it.get()))->getAngularDampingCoefficient(), dt);
			((RigidBody*)(it.get()))->setAngularVelocity( resultingAngularVelocity );
			#endif
			
			//#ifdef debuglvl2
			std::cout << " Linear Velocity :: after update :" << std::endl;
			((RigidBody*)(it.get()))->getLinearVelocity().afficher();
			std::cout << " Resulting Linear Velocity :: " << std::endl;
			resultingLinearVelocity.afficher();
			std::cout << " Angular Velocity :: " << std::endl;
			((RigidBody*)(it.get()))->getAngularVelocity().afficher();
			//#endif

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
			std::cout << "RigidBody : " << it->getName() << std::endl;
			std::cout << " Position :: before update " << std::endl;
			((RigidBody*)(it.get()))->getPosition().afficher();
			((RigidBody*)(it.get()))->setPosition( ((RigidBody*)(it.get()))->getPosition() + dt * ((RigidBody*)(it.get()))->getLinearVelocity());
			std::cout << " Linear Velocity :: before update " << std::endl;
			((RigidBody*)(it.get()))->getLinearVelocity().afficher();
			std::cout << " Position :: after update " << std::endl;
			((RigidBody*)(it.get()))->getPosition().afficher();
			
			#ifdef dampingAfter
			// Linear Damping :
			((RigidBody*)(it.get()))->setLinearVelocity( (float) pow( ((RigidBody*)(it.get()))->getLinearDampingCoefficient(), dt) * ((RigidBody*)(it.get()))->getLinearVelocity() );
			#endif
			
			//----------------------------------------------
			//----------------------------------------------
			//----------------------------------------------
			

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
			q.normalize();
			#endif
			//---------------------------

#define method3
// not working : #define method2
#ifdef method1			
			//METHOD 1 :
			
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
			
			
			//let us normalize res :
			#ifdef NORMALIZE_QUAT
			res.normalize();
			#endif
			
			
			//---------------------------
			
			((RigidBody*)(it.get()))->setOrientation( res );
			//----------------------------------------------
#endif			
			
#ifdef method2			
			//METHOD 2 : BARAFF rigid Body 1 
			std::cout << " Orientation :: before update " << std::endl;
			((RigidBody*)(it.get()))->getMatOrientation().afficher();
			std::cout << " Angular Velocity :: before update " << std::endl;
			((RigidBody*)(it.get()))->getAngularVelocity().afficher();
			
			Quat qav;
			Mat<float> vav( ((RigidBody*)(it.get()))->getAngularVelocity() );
			qav.x = vav.get(1,1);
			qav.y = vav.get(2,1);
			qav.z = vav.get(3,1);
			qav.w = 0;
			
			

			Quat res( q+dt*0.5f * Qt_Mul( qav, q) );
			std::cout << " Orientation :: before norm " << std::endl;
			Qt2Mat<float>(res).afficher();
			
			//let us normalize res :
			#ifdef NORMALIZE_QUAT
			res.normalize();
			#endif

			//----------------------------
			
			//std::cout << " INTEGRATOR : BEFORE : " << it->getName() << std::endl;
			//transpose( ((RigidBody*)(it.get()))->getMatOrientation() ).afficher();
			
			//float roll,pitch,yaw;
			//transpose( Qt2Mat<float>(q) ).afficher();
			//Qt2Euler( ((RigidBody*)(it.get()))->getOrientation(), &roll, &pitch, &yaw);
			
			//---------------------------
			
			((RigidBody*)(it.get()))->setOrientation( res );
			
			std::cout << " Orientation :: after update " << std::endl;
			((RigidBody*)(it.get()))->getMatOrientation().afficher();
			//----------------------------------------------
			
#endif			

#ifdef method3		
			//METHOD 3 : se3
			Mat<float> omega( ((RigidBody*)(it.get()))->getPose().getW() ); 
			Mat<float> avv( ((RigidBody*)(it.get()))->getAngularVelocity() );
			omega.set( omega.get(1,1) + dt*avv.get(1,1), 1,1);
			omega.set( omega.get(2,1) + dt*avv.get(2,1), 2,1);
			omega.set( omega.get(3,1) + dt*avv.get(3,1), 3,1);
			
			std::cout << " Orientation :: before update " << std::endl;
			((RigidBody*)(it.get()))->getMatOrientation().afficher();
			std::cout << " Angular Velocity :: before update " << std::endl;
			((RigidBody*)(it.get()))->getAngularVelocity().afficher();
			
			((RigidBody*)(it.get()))->setW(omega); 
			
			std::cout << " Orientation :: after update " << std::endl;
			((RigidBody*)(it.get()))->getMatOrientation().afficher();
			
			
#endif			
			
			#ifdef dampingAfter
			// Angular Damping :
			((RigidBody*)(it.get()))->setAngularVelocity( (float) pow( ((RigidBody*)(it.get()))->getAngularDampingCoefficient(), dt) * ((RigidBody*)(it.get()))->getAngularVelocity() );
			#endif
							
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



