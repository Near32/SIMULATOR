#include "IRobotHandler.h"
#include "../Simulation.h"

#define debug


IRobotHandler::IRobotHandler(Simulation* sim_) : sim(sim_)
{

}

IRobotHandler::~IRobotHandler()
{

}

void IRobotHandler::update(float dt)
{
	for( auto& rb : sim->simulatedObjects)
	{
		if( ((RigidBody*)rb.get())->isFixed == false )
		{
		if( ((RigidBody*)rb.get())->isRobot == false )
		{
			
			bool isComputing = true;
#ifdef debug
			((Robot*)rb.get())->process(dt,isComputing);
#endif
		}
		}	
	}	
}

//-------------------------------

