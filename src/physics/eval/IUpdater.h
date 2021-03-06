#ifndef IUPDATER_H
#define IUPDATER_H

#include "IIntegrator.h"
class Simulation;

class IUpdater
{
	protected :
	
	Simulation* sim;	
	std::unique_ptr<IIntegrator> integrator;
	
	
	//----------------------------------------------
	//----------------------------------------------
	public :
	
	IUpdater(Simulation* sim_, IIntegrator* integ);
	
	~IUpdater();
	
	virtual void update(float dt) = 0;
	virtual void updateVelocities(float dt) = 0;
	virtual void updatePositions(float dt) =0;
};


class Updater : public IUpdater
{
	public :
	
	Updater(Simulation* sim_, IIntegrator* integ);
	
	~Updater();
	
	virtual void update(float dt) override;
	virtual void updateVelocities(float dt) override;
	virtual void updatePositions(float dt) override;
	
};



#endif
