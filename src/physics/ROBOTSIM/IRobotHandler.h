#ifndef IROBOTHANDLER_H
#define IROBOTHANDLER_H

#include "../Robot.h"

class Simulation;


class IRobotHandler
{
	private :
	
	Simulation* sim;
	
	//------------------------------
	//------------------------------
	
	public :
	
	IRobotHandler(Simulation* sim_);
	
	~IRobotHandler();
	
	virtual void update(float dt = 0.0001f);	
	
	
	//-------------------------------
	
	
};

#endif
