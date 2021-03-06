#include "IContactGenerator.h"
#include "../Simulation.h"

#define debug


IContactGenerator::IContactGenerator(Simulation* sim_) : sim(sim_)
{

}

IContactGenerator::~IContactGenerator()
{

}

//----------------------------
//----------------------------
//----------------------------

//----------------------------
//----------------------------
//----------------------------


ContactGeneratorA::ContactGeneratorA(Simulation* sim_) : IContactGenerator(sim_)
{

}

ContactGeneratorA::~ContactGeneratorA()
{

}

void ContactGeneratorA::generateContactConstraints(std::vector<Contact>& c)
{
	//let us create a ContactConstraint in the simulation for every contact in c in order to have penetrationDepth :
	
#ifdef debug
std::cout << "COLLISION DETECTOR : CONTACT GENERATOR : initialization : OKAY." << std::endl;
#endif	

#ifdef debug
std::cout << "COLLISION DETECTOR : CONTACT GENERATOR : contact forces creations : " << c.size() << " contact(s)." << std::endl;
#endif	
	//--------------------------------------
	
	
	for(auto& contact : c)
	{
		//let us create the SpringForceEffect required into the simulation with endTime being the next simulation step time :
		//sim->collectionF.insert( sim->collectionF.begin(), std::unique_ptr<IForceEffect>( new SpringForceEffect( /*in LOCAL : c[i].rbA->getPosition()*/c[i].rbA->getPointInLocal(c[i].contactPoint[0]), /*in LOCAL : c[i].rbB->getPosition()*/c[i].rbB->getPointInLocal(c[i].contactPoint[0]), *(c[i].rbA), *(c[i].rbB), pD[i] ) ) );
		//sim->collectionF[0]->setEndTime( sim->getTime()+sim->getTimeStep());
		//this collision force is only here for one timestep and it will be recreated if needed on the next time step.

		for(int i=contact.contactPoint.size();i--;)
		{
			//let us create a contact constraints for every contact point :
			
			//let us find the anchors in local frames : A ...
			Mat<float> anchorAL( contact.rbA->getPointInLocal(contact.contactPoint[i]) );
			Mat<float> anchorALProjected( contact.rbA->getPointInLocal(contact.contactPoint[i]) );
			
			//----------------------------------------------------------------------------------
			// ... then B :
			//----------------------------------------------------------------------------------
			Mat<float> normalAB(0.0f,3,1);
			//innerVoronoiProjectionANDNormal( *(contact.rbA), anchorALProjected, normalAB);
			innerVoronoiProjectionANDNormal( *(contact.rbA), anchorALProjected, normalAB);
			//Mat<float> normalAB( contact.normal[0]);
			
			//----------------------------------------------------------------------------------
			// Beware, penetration depths should be positive, 
			// but they can be negative when the contact is (about to be resting) positively.
			//----------------------------------------------------------------------------------
			float penetrationDepth = ( transpose(normalAB)*(anchorALProjected-anchorAL) ).get(1,1);
			//----------------------------------------------------------------------------------
			//penetrationDepth = fabs(penetrationDepth);
			//----------------------------------------------------------------------------------
			
			//----------------------------------------------------------------------------------
			//----------------------------------------------------------------------------------
			#ifdef debug 
			std::cout << "CONTACT GENERATOR :: Contact  : " << i << " : penetration depth = " << penetrationDepth << std::endl;
			std::cout << "CONTACT GENERATOR :: Contact  : " << i << " : normal :" << std::endl;
			transpose(normalAB).afficher();
			#endif
			//----------------------------------------------------------------------------------
			//----------------------------------------------------------------------------------
			

			//----------------------------------------------------------------------------------
			//Creation of the ContactConstraint :
			//----------------------------------------------------------------------------------
			ContactConstraint* ptrCst = new ContactConstraint( *(contact.rbA), *(contact.rbB), anchorAL, anchorALProjected, normalAB, penetrationDepth, contact.restitutionFactor);
			//----------------------------------------------------------------------------------
			//----------------------------------------------------------------------------------
			
			//----------------------------------------------------------------------------------
			//Computation of the relative velocity :
			//----------------------------------------------------------------------------------
			Mat<float> pointW( contact.rbA->getPointInWorld( contact.contactPoint[i] ) );
			Mat<float> vAPW( contact.rbA->getVelocityPointWInWorld( pointW ) );
			Mat<float> vBPW( contact.rbB->getVelocityPointWInWorld( pointW ) );
			//  relative velocity from A to B :
			ptrCst->setVrel( vBPW-vAPW);
			//----------------------------------------------------------------------------------
			//----------------------------------------------------------------------------------
			
			//----------------------------------------------------------------------------------
			//	Registration of the ContactConstraint :
			//----------------------------------------------------------------------------------
			sim->collectionC.insert( sim->collectionC.end(), std::unique_ptr<IConstraint>( ptrCst ) );
			//----------------------------------------------------------------------------------
			//----------------------------------------------------------------------------------
		}
	}
	

	
}
