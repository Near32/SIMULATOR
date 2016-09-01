#include "IUpdater.h"
#include "../Simulation.h"

IUpdater::IUpdater(Simulation* sim_, IIntegrator* integ) : sim(sim_), integrator( integ )
{

}

IUpdater::~IUpdater()
{

}



//-------------------------------
//-------------------------------
//-------------------------------


//-------------------------------
//-------------------------------
//-------------------------------


Updater::Updater(Simulation* sim_, IIntegrator* integ) : IUpdater(sim_,integ)
{

}

Updater::~Updater()
{

}

void Updater::update(float dt)
{
	this->integrator->integrateVelocities(dt);
}

void Updater::updateVelocities(float dt)
{
	this->integrator->integrateVelocities(dt);
}

void Updater::updatePositions(float dt)
{
	this->integrator->integratePositions(dt);
}
