#include "Contact.h"

#define defRestFactor 1.0f

/*
Contact::Contact()
{

}
*/

Contact::Contact( Contact& c) : rbA(c.rbA), rbB(c.rbB)
{
	contactPoint = c.contactPoint;
	normal = c.normal;
	restitutionFactor = defRestFactor;
}

Contact::Contact(const Contact& c) : rbA(c.rbA), rbB(c.rbB)
{
	contactPoint = c.contactPoint;
	normal = c.normal;
	restitutionFactor = c.restitutionFactor;	
}

Contact::Contact(RigidBody* rbA_, RigidBody* rbB_) : rbA(rbA_), rbB(rbB_)
{
	restitutionFactor = defRestFactor;
	normal.push_back( rbB->getPosition()-rbA->getPosition());
	float magnitude = norme2(normal[0]);
	if(magnitude > 0.0f)
	{
		normal[0] *= 1.0f/magnitude;
	}
	else
	{
		throw;
	}
}

Contact::~Contact()
{

}

Contact& Contact::operator=(Contact& c)
{
	rbA = c.rbA;
	rbB = c.rbB;
	
	contactPoint = c.contactPoint;
	normal = c.normal;
	
	return *this;
}

Contact Contact::operator=(Contact c)
{
	rbA = c.rbA;
	rbB = c.rbB;
	
	contactPoint = c.contactPoint;
	normal = c.normal;
	
	return *this;
}
