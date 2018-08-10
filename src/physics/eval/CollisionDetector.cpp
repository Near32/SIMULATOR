#include "CollisionDetector.h"
#include "../Simulation.h"

#define debug


CollisionDetector::CollisionDetector(Simulation* sim_, float eps) : sim(sim_), epsilon(eps), broadPhase(new BroadPhaseStrategyA( sim_)), midNarrowPhase(new MidNarrowPhaseStrategyA( sim_)), contactGenerator( new ContactGeneratorA( sim_))
{

}

CollisionDetector::~CollisionDetector()
{

}

void CollisionDetector::checkForCollision(float dt)
{
#ifdef debug
std::cout << "COLLISION DETECTOR : initialization : ..." << std::endl;
#endif	
	contacts.clear();
#ifdef debug
std::cout << "COLLISION DETECTOR : broadPhase : ..." << std::endl;
#endif	
	
	broadPhase->checkForCollisions(contacts);
	
	
#ifdef debug
std::cout << "COLLISION DETECTOR : nbrContacts : " << contacts.size() << std::endl;
for(auto& c : contacts)
{
	std::cout << "Contact :: " << c.rbA->getName() << " <--> " << c.rbB->getName() << std::endl; 
	
	/*
	Mat<float> origin(0.0f, 3,1);
	Mat<float> originA( c.rbA->getPointInWorld( c.rbA->getPointInLocal(origin) ) );
	originA.afficher();
	c.rbA->getPose().getSE3().afficher();
	Mat<float> originB( c.rbB->getPointInWorld( c.rbB->getPointInLocal(origin) ) );
	originB.afficher();
	c.rbB->getPose().getSE3().afficher();
	*/

}
std::cout << "COLLISION DETECTOR : midNarrowPhase : ..." << std::endl;
#endif

	
	midNarrowPhase->checkForCollisions(contacts);
	
	
#ifdef debug
std::cout << "COLLISION DETECTOR : nbrContacts : " << contacts.size() << std::endl;
std::cout << "COLLISION DETECTOR : constraints generation : ..." << std::endl;
#endif
	
	contactGenerator->generateContactConstraints(contacts);
	
#ifdef debug
std::cout << "COLLISION DETECTOR : nbrContacts : " << contacts.size() << std::endl;
std::cout << "COLLISION DETECTOR : all set : OKAY." << std::endl;
#endif	
}



//-------------------------------

std::vector<Contact> CollisionDetector::getContacts()
{	
	return contacts;	
}
