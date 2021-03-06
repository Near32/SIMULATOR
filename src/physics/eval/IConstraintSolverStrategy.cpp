#include "IConstraintSolverStrategy.h"
#include "../Simulation.h"

#include "../../utils/LCPSolver/LCPSolver.h"

//#define debuglvl1
//#define debuglvl2
//#define debuglvl3
//#define debuglvl4


IConstraintSolverStrategy::IConstraintSolverStrategy(Simulation* sim_) : sim(sim_)
{
	C = Mat<float>(0.0f,1,1);
	constraintsJacobian = Mat<float>(0.0f,1,1);
	rs = new RunningStats<float>(std::string("./stats.txt"));
}

IConstraintSolverStrategy::~IConstraintSolverStrategy()
{
	delete rs;
}

Mat<float> IConstraintSolverStrategy::getConstraints() const
{
	return C;
}

Mat<float> IConstraintSolverStrategy::getConstraintsJacobian() const
{
	return constraintsJacobian;
}

Mat<float> IConstraintSolverStrategy::getLambda()	const
{
	return lambda;
}



//-----------------------------------------
//-----------------------------------------
//-----------------------------------------

//-----------------------------------------
//-----------------------------------------
//-----------------------------------------


SimultaneousImpulseBasedConstraintSolverStrategy::SimultaneousImpulseBasedConstraintSolverStrategy(Simulation* sim_) : IConstraintSolverStrategy(sim_)
{

}

SimultaneousImpulseBasedConstraintSolverStrategy::~SimultaneousImpulseBasedConstraintSolverStrategy()
{

}


void SimultaneousImpulseBasedConstraintSolverStrategy::computeConstraintsJacobian(std::vector<std::unique_ptr<IConstraint> >& c)
{
	size_t size = c.size();
	int n = sim->simulatedObjects.size();
	
	c[0]->computeJacobians();
		
	Mat<float> tJA(c[0]->getJacobianA());
	Mat<float> tJB(c[0]->getJacobianB());
	
	//tJA.afficher();
	//tJB.afficher();
		
	int sl = tJA.getLine();
	int idA = 6 * ( c[0]->rbA.getID() );
	int idB = 6 * ( c[0]->rbB.getID() );
	
	Mat<float> temp((float)0,sl, 6*n );
	
	for(int i=1;i<=sl;i++)
	{
		for(int j=1;j<=6;j++)
		{
			temp.set( tJA.get(i,j) , i, idA+j);
			temp.set( tJB.get(i,j), i, idB+j  );
		}
	}
	
	constraintsJacobian = temp;
	
	for(int i=1;i<size;i++)
	{
		c[i]->computeJacobians();
		
		Mat<float> tJA(c[i]->getJacobianA());
		Mat<float> tJB(c[i]->getJacobianB());
		
		size_t sl = tJA.getLine();
		int idA = 6 * ( c[i]->rbA.getID() );
		int idB = 6 * ( c[i]->rbB.getID() );
		
		temp = Mat<float>((float)0,sl, 6*n );
		
		for(int i=1;i<=sl;i++)
		{
			for(int j=1;j<=6;j++)
			{
				temp.set( tJA.get(i,j) , i, idA+j);
				temp.set( tJB.get(i,j), i, idB+j  );
			}
		}
		
		constraintsJacobian = operatorC(constraintsJacobian, temp );
	}
}

void SimultaneousImpulseBasedConstraintSolverStrategy::computeConstraintsJacobian(std::vector<std::unique_ptr<IConstraint> >& c, const Mat<float>& q, const Mat<float>& qdot)
{
	//MAJ qdot :
	int b1 = 0;
	int b2 = 0;
	for( auto& o : sim->simulatedObjects ) 
	{	
		//((RigidBody*)(o.get()))->setPosition( extract(q, b1+1,1, b1+3,1) );	
		//((RigidBody*)(o.get()))->setMatOrientation( extract(q, b1+4,1, b1+7,1) );
		((RigidBody*)(o.get()))->setLinearVelocity( extract( qdot, b2+1,1, b2+3,1) );
		((RigidBody*)(o.get()))->setAngularVelocity( extract( qdot, b1+4,1, b1+6,1) );
		
		b1+=7;
		b2+=6;	
	}
	
	//-------------------------------------
	//-------------------------------------
	//-------------------------------------
	
	size_t size = c.size();
	int n = sim->simulatedObjects.size();
	
	c[0]->computeJacobians();
	
	int idA = 6 * ( c[0]->rbA.getID() );
	int idB = 6 * ( c[0]->rbB.getID() );
	
	Mat<float> tJA(c[0]->getJacobianA());
	Mat<float> tJB(c[0]->getJacobianB());
	int sl = tJA.getLine();
	
	Mat<float> temp((float)0,sl, 6*n );
	
	for(int i=1;i<=sl;i++)
	{
		for(int j=1;j<=6;j++)
		{
			temp.set( tJA.get(i,j) , i, idA+j);
			temp.set( tJB.get(i,j), i, idB+j  );
		}
	}
	
	constraintsJacobian = temp;
	
	for(int i=1;i<size;i++)
	{
		c[i]->computeJacobians();
		
		Mat<float> tJA(c[i]->getJacobianA());
		Mat<float> tJB(c[i]->getJacobianB());
		
		size_t sl = tJA.getLine();
		int idA = 6 * ( c[i]->rbA.getID() );
		int idB = 6 * ( c[i]->rbB.getID() );
		
		temp = Mat<float>((float)0,sl, 6*n );
		
		for(int i=1;i<=sl;i++)
		{
			for(int j=1;j<=6;j++)
			{
				temp.set( tJA.get(i,j) , i, idA+j);
				temp.set( tJB.get(i,j), i, idB+j  );
			}
		}
		
		constraintsJacobian = operatorC(constraintsJacobian, temp );
	}
	
	
	//Let us delete the contact constraints that are ephemerous by essence :
	std::vector<std::unique_ptr<IConstraint> >::iterator itC = c.begin();
	bool erased = false;
	while(itC != c.end())
	{
		if( (itC->get())->getType() == CTContactConstraint )
		{
			erased = true;
			c.erase(itC);
		}
	
		if(!erased)
			itC++;
			
		erased = false;
	}
}

void SimultaneousImpulseBasedConstraintSolverStrategy::computeConstraintsANDJacobian(std::vector<std::unique_ptr<IConstraint> >& c, const Mat<float>& q, const Mat<float>& qdot)
{
	//MAJ qdot :
	int b1 = 0;
	int b2 = 0;
	
	/*
	for( auto& o : sim->simulatedObjects ) 
	{	
		//((RigidBody*)(o.get()))->setPosition( extract(q, b1+1,1, b1+3,1) );	
		//((RigidBody*)(o.get()))->setMatOrientation( extract(q, b1+4,1, b1+7,1) );
		
		((RigidBody*)(o.get()))->setLinearVelocity( extract( qdot, b2+1,1, b2+3,1) );
		((RigidBody*)(o.get()))->setAngularVelocity( extract( qdot, b2+4,1, b2+6,1) );
		
		b1+=7;
		b2+=6;	
	}
	*/
	//-------------------------------------
	//-------------------------------------
	//-------------------------------------
	
	size_t size = c.size();
	int n = sim->simulatedObjects.size();

	c[0]->computeJacobians();
	
	int idA = 6 * ( c[0]->rbA.getID() );
	int idB = 6 * ( c[0]->rbB.getID() );
	
	Mat<float> tJA(c[0]->getJacobianA());
	Mat<float> tJB(c[0]->getJacobianB());
	//Constraint :
	Mat<float> tC(c[0]->getConstraint());
	
	int sl = tJA.getLine();
	
	Mat<float> temp((float)0,sl, 6*n );
	
	for(int i=1;i<=sl;i++)
	{
		for(int j=1;j<=6;j++)
		{
			temp.set( tJA.get(i,j) , i, idA+j);
			temp.set( tJB.get(i,j), i, idB+j  );
		}
	}
	
	constraintsJacobian = temp;
	//Constraint :
	float baumgarteBAS = 1e-1f;
	float baumgarteC = 1e-1f;
	C = tC;
	//----------------------------------------
	//BAUMGARTE STABILIZATION
	//----------------------------------------
	//Contact offset :
	if( c[0]->getType() == CTContactConstraint)
	{
		//Baumgarte stabilization :
		//temp *= 0.1f/this->dt;
		//float slop = -5e-1f;
		//float pdepth = ((ContactConstraint*)(c[0].get()))->penetrationDepth;
		//tC *= baumgarteC/this->dt*(pdepth-slop);
		tC *= baumgarteC/this->dt;
	}
	//BAS JOINT :
	if( c[0]->getType() == CTBallAndSocketJoint)
	{
		tC*= baumgarteBAS/this->dt;
	}
	//----------------------------------------
	//----------------------------------------
	offset = tC;
	
	
#ifdef debuglvl1
std::cout << "CONSTRAINTS : nbr = " << size << std::endl;
std::cout << "CONSTRAINTS : 0 : type = " << c[0]->getType() << " ; ids are : " << c[0]->rbA.getID() << " : " << c[0]->rbB.getID() << std::endl;
offset.afficher();
/*std::cout << "local Anchor A : " << std::endl;
c[0]->AnchorAL.afficher();
std::cout << "global Anchor A : " << std::endl;
c[0]->rbA.getPointInWorld(c[0]->AnchorAL).afficher();
std::cout << "local Anchor B : " << std::endl;
c[0]->AnchorBL.afficher();
std::cout << "global Anchor B : " << std::endl;
c[0]->rbB.getPointInWorld(c[0]->AnchorBL).afficher();*/
#endif	
	
	for(int i=1;i<size;i++)
	{

		c[i]->computeJacobians();
		
		tJA = c[i]->getJacobianA();
		tJB = c[i]->getJacobianB();
		
		//Constraint :
		tC = c[i]->getConstraint();
		
		size_t sl = tJA.getLine();
		int idA = 6 * ( c[i]->rbA.getID() );
		int idB = 6 * ( c[i]->rbB.getID() );
		
		temp = Mat<float>((float)0,sl, 6*n );
		
		//Constraints :
		C = operatorC(C, tC);
		//----------------------------------------
		//BAUMGARTE STABILIZATION
		//----------------------------------------
		//Contact offset :
		if( c[i]->getType() == CTContactConstraint)
		{
			//Baumgarte stabilization :
			//temp *= 0.1f/this->dt;
			tC *= baumgarteC/this->dt;
		}
		//BAS JOINT :
		if( c[i]->getType() == CTBallAndSocketJoint)
		{
			tC*= baumgarteBAS/this->dt;
		}
		//----------------------------------------
		//----------------------------------------
		
						
		for(int i=1;i<=sl;i++)
		{
			for(int j=1;j<=6;j++)
			{
				temp.set( tJA.get(i,j) , i, idA+j);
				temp.set( tJB.get(i,j), i, idB+j  );
			}
		}
		
		constraintsJacobian = operatorC(constraintsJacobian, temp );
		//Constraint :
		offset = operatorC( offset, tC);
		
#ifdef debuglvl1
std::cout << "CONSTRAINTS : " << i << " type = " << c[i]->getType() << " ; ids are : " << c[i]->rbA.getID() << " : " << c[i]->rbB.getID() << std::endl;
offset.afficher();
#endif
		
	}
	
	
	//Let us delete the contact constraints that are ephemerous by essence :
	std::vector<std::unique_ptr<IConstraint> >::iterator itC = c.begin();
	bool erased = false;
	while(itC != c.end())
	{
		if( (itC->get())->getType() == CTContactConstraint )
		{
			erased = true;
			c.erase(itC);
		}
	
		if(!erased)
			itC++;
			
		erased = false;
	}
}


/*
void SimultaneousImpulseBasedConstraintSolverStrategy::Solve(float dt, std::vector<std::unique_ptr<IConstraint> >& c, Mat<float>& q, Mat<float>& qdot, SparseMat<float>& invM, SparseMat<float>& S, const Mat<float>& Fext )
{
	computeConstraintsJacobian(c);
	
	SparseMat<float> tConstraintsJacobian( transpose(constraintsJacobian) );
	
	Mat<float> temp( invGJ( constraintsJacobian * ( invM*tConstraintsJacobian ).SM2mat() ) * constraintsJacobian );//invM*tConstraintsJacobian ) * constraintsJacobian );
	
	Mat<float> tempInvMFext( invM *(dt * Fext) ) ;//SM2Mat<float>( invM * Fext ) );
	//lambda = (-1.0f) * ( temp * tempInvMFext + (1.0f/dt) * ( temp * qdot) ) ;
	lambda = (-1.0f) * ( (temp * tempInvMFext) + ( temp * qdot) ) ;
	
	
	constraintsImpulse =  tConstraintsJacobian * lambda;
	
	Mat<float> tdot( tempInvMFext + invM * constraintsImpulse );
	qdot += tdot;//SM2Mat<float>(  );
	
	
	Mat<float> t( dt*( S*qdot ) );
	q += t;
	
	//S.print();
	//constraintsJacobian.afficher();
	//tempInvMFext.afficher();
	//temp.afficher();
	//lambda.afficher();
	//constraintsImpulse.afficher();
	//tempInvMFext.afficher();
	//tdot.afficher();
	//qdot.afficher();
	//t.afficher();
	//q.afficher();
	
}
*/

/*
void SimultaneousImpulseBasedConstraintSolverStrategy::Solve(float dt, std::vector<std::unique_ptr<IConstraint> >& c, Mat<float>& q, Mat<float>& qdot, SparseMat<float>& invM, SparseMat<float>& S, const Mat<float>& Fext )
{
	Mat<float> tempInvMFext( dt*(invM * Fext) ) ;
	qdot += tempInvMFext;
	
	for(int i=0;i<=1;i++)
	{
	computeConstraintsJacobian(c,q,qdot);
	
	SparseMat<float> tConstraintsJacobian( transpose(constraintsJacobian) );
	Mat<float> invMtJ( ( invM*tConstraintsJacobian ).SM2mat() );
	
	Mat<float> temp( invGJ( constraintsJacobian * ( invM*tConstraintsJacobian ).SM2mat() ) * constraintsJacobian );//invM*tConstraintsJacobian ) * constraintsJacobian );
	
	lambda = (-1.0f) * ( (temp * tempInvMFext) + ( temp * qdot) ) ;
	
	
	constraintsImpulse =  tConstraintsJacobian * lambda;
	
	Mat<float> tdot(invM * constraintsImpulse );
	qdot += tdot;//SM2Mat<float>(  );
	
	
#ifdef debuglvl1
	std::cout << "J = " << std::endl;
	constraintsJacobian.afficher();

	Mat<float> Cdot(constraintsJacobian*qdot);
	std::cout << "Cdot = Jqdot : norme2 =  " << norme2(Cdot) << std::endl;	
	Cdot.afficher();
	tdot.afficher();
	if( norme2(Cdot) <= 1e-20f )
		i=100;		
#endif
	}	
	
	Mat<float> t( dt*( S*qdot ) );
	q += t;
	
	//S.print();
	//constraintsJacobian.afficher();
	//tempInvMFext.afficher();
	//temp.afficher();
	//lambda.afficher();
	//constraintsImpulse.afficher();
	//tempInvMFext.afficher();
	//tdot.afficher();
	//qdot.afficher();
	//t.afficher();
	//q.afficher();
	
}
*/

/*
void SimultaneousImpulseBasedConstraintSolverStrategy::Solve(float dt, std::vector<std::unique_ptr<IConstraint> >& c, Mat<float>& q, Mat<float>& qdot, SparseMat<float>& invM, SparseMat<float>& S, const Mat<float>& Fext )
{
	Mat<float> tempInvMFext( dt*(invM * Fext) ) ;
	qdot += tempInvMFext;
	computeConstraintsJacobian(c,q,qdot);
	
	Mat<float> tConstraintsJacobian( transpose(constraintsJacobian) );
	Mat<float> M( invGJ( invM.SM2mat()) );
	//Mat<float> A( operatorC( operatorL( M, (-1.0f)*tConstraintsJacobian), operatorL( constraintsJacobian, Mat<float>((float)0, constraintsJacobian.getLine(), constraintsJacobian.getLine()) ) ) );
	Mat<float> b( operatorC( (-1.0f)*(M*qdot+dt*Fext), Mat<float>(0.0f, 3,1) ) );
	Mat<float> x( operatorC( qdot, Mat<float>(0.0f,3, 1) ) );
		
	int N = 10;
	LCPSolver instanceLCPSolver( N, A,b,x);
	x = instanceLCPSolver.solve(N);
	

	lambda = extract( x, qdot.getLine()+1, 1, x.getLine(), 1);
	
	constraintsImpulse =  tConstraintsJacobian * lambda;
	
	Mat<float> tdot(invM * constraintsImpulse );
	qdot += tdot;//SM2Mat<float>(  );
	
	A.afficher();
	std::cout << " qdot+ = " << std::endl;
	qdot.afficher();
	std::cout << " computation gave : " << std::endl;
	extract(x, 1,1, qdot.getLine(), 1).afficher();
	
	
	Mat<float> t( dt*( S*qdot ) );
	q += t;
	
	//S.print();
	//constraintsJacobian.afficher();
	//tempInvMFext.afficher();
	//temp.afficher();
	lambda.afficher();
	//constraintsImpulse.afficher();
	//tempInvMFext.afficher();
	//tdot.afficher();
	//qdot.afficher();
	//t.afficher();
	//q.afficher();
	
}
*/

/*
//TIME STEPPING IMPULSE BASED METHOD : with Baumgarte Stabilization....
void SimultaneousImpulseBasedConstraintSolverStrategy::Solve(float dt, std::vector<std::unique_ptr<IConstraint> >& c, Mat<float>& q, Mat<float>& qdot, SparseMat<float>& invM, SparseMat<float>& S, const Mat<float>& Fext )
{
	computeConstraintsJacobian(c);
	Mat<float> tempInvMFext( dt*(invM * Fext) ) ;
	qdot += tempInvMFext;
	//computeConstraintsJacobian(c,q,qdot);
	Mat<float> b((float)10,3,1);
	Mat<float> C( ((RigidBody*)(sim->simulatedObjects[1].get()))->getPointInWorld(c[0]->AnchorAL )-((RigidBody*)(sim->simulatedObjects[2].get()))->getPointInWorld(c[0]->AnchorBL) );
	b = (0.5f/dt)*C;
	//BAUMGARTE STABILIZATION ....
	
	
	std::cout << "Constraints BASJoint : norme  = " << norme2(C) << std::endl;
	C.afficher();
	c[0]->AnchorAL.afficher();
	((RigidBody*)(sim->simulatedObjects[1].get()))->getPointInWorld(c[0]->AnchorAL ).afficher();
	c[0]->AnchorBL.afficher();
	((RigidBody*)(sim->simulatedObjects[2].get()))->getPointInWorld(c[0]->AnchorBL ).afficher();
	
	
	SparseMat<float> tConstraintsJacobian( transpose(constraintsJacobian) );
	Mat<float> invJinvMtJ( invGJ( constraintsJacobian * ( invM*tConstraintsJacobian ).SM2mat() ) );//invM*tConstraintsJacobian ) * constraintsJacobian );
	
	lambda = (-1.0f) * (invJinvMtJ * (constraintsJacobian*qdot+b) );
	
	
	constraintsImpulse =  tConstraintsJacobian * lambda;
	
	//Mat<float> tdot( (1.0f)*tempInvMFext + invM * constraintsImpulse );
	Mat<float> tdot(invM * constraintsImpulse );
	qdot += tdot;//SM2Mat<float>(  );
	
	
	Mat<float> t( dt*( S*qdot ) );
	q += t;
	
	//S.print();
	//constraintsJacobian.afficher();
	//temp.afficher();
	//invJinvMtJ.afficher();
	std::cout << " lambda = : " << std::endl;
	lambda.afficher();
	std::cout << " Pc : " << std::endl;
	constraintsImpulse.afficher();
	//tempInvMFext.afficher();
	//tdot.afficher();
	//qdot.afficher();
	//t.afficher();
	//q.afficher();
	
}
*/


//KKT :
//BACKUP.... the new one is to handle contact constraints... baumgarte watch out...
/*
void SimultaneousImpulseBasedConstraintSolverStrategy::Solve(float dt, std::vector<std::unique_ptr<IConstraint> >& c, Mat<float>& q, Mat<float>& qdot, SparseMat<float>& invM, SparseMat<float>& S, const Mat<float>& Fext )
{
	
	//computeConstraintsJacobian(c);
	Mat<float> tempInvMFext( dt*(invM * Fext) ) ;
	//qdot += tempInvMFext;
	//computeConstraintsJacobian(c,q,qdot);
	computeConstraintsANDJacobian(c,q,qdot);
	
	Mat<float> offset((float)10,3,1);
	Mat<float> C( ((RigidBody*)(sim->simulatedObjects[1].get()))->getPointInWorld(c[0]->AnchorAL )-((RigidBody*)(sim->simulatedObjects[2].get()))->getPointInWorld(c[0]->AnchorBL) );
	offset = (0.5f/dt)*C;
	//BAUMGARTE STABILIZATION ....
	
	std::cout << "Constraints BASJoint : norme  = " << norme2(C) << std::endl;
	C.afficher();
	//c[0]->AnchorAL.afficher();
	//((RigidBody*)(sim->simulatedObjects[1].get()))->getPointInWorld(c[0]->AnchorAL ).afficher();
	//c[0]->AnchorBL.afficher();
	//((RigidBody*)(sim->simulatedObjects[2].get()))->getPointInWorld(c[0]->AnchorBL ).afficher();
	
	Mat<float> tConstraintsJacobian( transpose(constraintsJacobian) );
	Mat<float> A( (-1.0f)*tConstraintsJacobian );
	Mat<float> M( invGJ( invM.SM2mat() ) );
	A = operatorL( M, A);
	A = operatorC( A , operatorL( constraintsJacobian, Mat<float>((float)0,constraintsJacobian.getLine(), constraintsJacobian.getLine()) ) );
	
	Mat<float> invA( invGJ(A) );//invM*tConstraintsJacobian ) * constraintsJacobian );
	
	Mat<float> tempLambda( invA * operatorC( Mat<float>((float)0,invA.getLine()-3,1) , constraintsJacobian*(invM*Fext)+offset ) );
	lambda = extract( &tempLambda, qdot.getLine()+1, 1, tempLambda.getLine(), 1);
	
	qdot = extract(  &tempLambda, 1,1, qdot.getLine(), 1)+tempInvMFext;
	
	
	Mat<float> t( dt*( S*qdot ) );
	q += t;
	
	//S.print();
	std::cout << " computed Pc : " << std::endl;
	(tConstraintsJacobian*lambda).afficher();
	std::cout << "invM*Fext : " << std::endl;
	tempInvMFext.afficher();
	//temp.afficher();
	//(constraintsJacobian*(invM*Fext)).afficher();
	//(invM*Fext).afficher();
	//invA.afficher();
	lambda.afficher();
	qdot.afficher();
	//tempInvMFext.afficher();
	//tdot.afficher();
	//qdot.afficher();
	//t.afficher();
	q.afficher();
	
}
*/

//KKT : with baumgarte handling the whole constraints : contacts are handled too : assumed model to be correct to some extent :
/*
void SimultaneousImpulseBasedConstraintSolverStrategy::Solve(float dt, std::vector<std::unique_ptr<IConstraint> >& c, Mat<float>& q, Mat<float>& qdot, SparseMat<float>& invM, SparseMat<float>& S, const Mat<float>& Fext )
{
	//std::cout << "STATE :" << std::endl;
	//q.afficher();
	
	
	this->dt = dt;
	//computeConstraintsJacobian(c);
	Mat<float> tempInvMFext( dt*(invM * Fext) ) ;
	//qdot += tempInvMFext;
	//computeConstraintsJacobian(c,q,qdot);
	computeConstraintsANDJacobian(c,q,qdot);
	
	//BAUMGARTE STABILIZATION has been handled in the computeConstraintsANDJacobian function....
	//std::cout << "Constraints : norme  = " << norme2(C) << std::endl;
	//C.afficher();
	
	Mat<float> tConstraintsJacobian( transpose(constraintsJacobian) );
	//std::cout << "t Constraints Jacobian :" << std::endl;
	//tConstraintsJacobian.afficher();
	Mat<float> A( (-1.0f)*tConstraintsJacobian );
	Mat<float> M( invGJ( invM.SM2mat() ) );
	A = operatorL( M, A);
	A = operatorC( A , operatorL( constraintsJacobian, Mat<float>((float)0,constraintsJacobian.getLine(), constraintsJacobian.getLine()) ) );
	
	Mat<float> invA( invGJ(A) );//invM*tConstraintsJacobian ) * constraintsJacobian );
	
	Mat<float> tempLambda( invA * operatorC( Mat<float>((float)0,invA.getLine()-constraintsJacobian.getLine(),1) , constraintsJacobian*(invM*Fext) + offset ) );
	lambda = extract( &tempLambda, qdot.getLine()+1, 1, tempLambda.getLine(), 1);
	
	if(isnanM(lambda))
		lambda = Mat<float>(0.0f,lambda.getLine(),lambda.getColumn());
		
	//qdot += dt*extract(  &tempLambda, 1,1, qdot.getLine(), 1);//+tempInvMFext
	
	//Assumed model :
	qdot += tempInvMFext -dt*extract(  &tempLambda, 1,1, qdot.getLine(), 1);
	
	//qdot += tempInvMFext;	//contraints not satisfied.
	
	//qdot += tempInvMFext;	//qdot+ = qdot- + dt*M-1Fext;
	//Mat<float> qdotreal( qdot + dt*extract(  &tempLambda, 1,1, qdot.getLine(), 1) );	//qdotreal = qdot+ + Pc;
	
	Mat<float> t( dt*( S*qdot ) );
	q += t;
	
	//S.print();
	
	std::cout << " computed Pc : " << std::endl;
	(tConstraintsJacobian*lambda).afficher();
	std::cout << " delta state = S * qdotreal : " << std::endl; 
	t.afficher();
	//std::cout << " S & qdotreal : " << std::endl;
	//S.print();
	//qdot.afficher();
	//std::cout << "invM*Fext : " << std::endl;
	//tempInvMFext.afficher();
	
	
	//temp.afficher();
	//(constraintsJacobian*(invM*Fext)).afficher();
	//(invM*Fext).afficher();
	//std::cout << " A : " << std::endl;
	//A.afficher();
    //std::cout << " SVD A*tA :  S : " << std::endl;
    //SVD<float> instanceSVD(A*transpose(A));
    //instanceSVD.getS().afficher();
	//std::cout << " invA : " << std::endl;
	//invA.afficher();
	
	//std::cout << " LAMBDA : " << std::endl;
	//lambda.afficher();
	//std::cout << " qdot+ : " << std::endl;
	//qdot.afficher();
	//std::cout << " q+ : " << std::endl;
	//q.afficher();
	
	std::cout << "Cdot : " << std::endl;
	(constraintsJacobian*qdot).afficher();
	//BAUMGARTE STABILIZATION has been handled in the computeConstraintsANDJacobian function....
	std::cout << "Constraints : norme  = " << norme2(C) << std::endl;
	C.afficher();
	
}
*/

//KKT : with baumgarte handling the whole constraints : contacts are handled too :

void SimultaneousImpulseBasedConstraintSolverStrategy::Solve(float dt, std::vector<std::unique_ptr<IConstraint> >& c, Mat<float>& q, Mat<float>& qdot, SparseMat<float>& invM, SparseMat<float>& S, const Mat<float>& Fext )
{
	//std::cout << "STATE :" << std::endl;
	//q.afficher();
	
	Mat<float> qdotminus(qdot);
	this->dt = dt;
	//computeConstraintsJacobian(c);
	Mat<float> tempInvMFext( dt*(invM * Fext) ) ;
	//qdot += tempInvMFext;
	//computeConstraintsJacobian(c,q,qdot);
	computeConstraintsANDJacobian(c,q,qdot);
	
	//BAUMGARTE STABILIZATION has been handled in the computeConstraintsANDJacobian function....
	//std::cout << "Constraints : norme  = " << norme2(C) << std::endl;
	//C.afficher();
	
	Mat<float> tConstraintsJacobian( transpose(constraintsJacobian) );
	//std::cout << "t Constraints Jacobian :" << std::endl;
	//tConstraintsJacobian.afficher();
	
	
	//PREVIOUS METHOD :
	//--------------------------------
	//David Baraff 96 STAR.pdf Interactive Simulation of Rigid Body Dynamics in Computer Graphics : Lagrange Multipliers Method :
	//Construct A :
	/*
	Mat<float> A( (-1.0f)*tConstraintsJacobian );
	Mat<float> M( invGJ( invM.SM2mat() ) );
	A = operatorL( M, A);
	A = operatorC( A , operatorL( constraintsJacobian, Mat<float>((float)0,constraintsJacobian.getLine(), constraintsJacobian.getLine()) ) );
	*/
	//----------------------------
	Mat<float> A( constraintsJacobian * invM.SM2mat() * tConstraintsJacobian );
	
	//---------------------------
	Mat<float> invA( invGJ(A) );//invM*tConstraintsJacobian ) * constraintsJacobian );
	
	//Construct b and compute the solution.
	//----------------------------------
	//Mat<float> tempLambda( invA * operatorC( Mat<float>((float)0,invA.getLine()-constraintsJacobian.getLine(),1) , (-1.0f)*(constraintsJacobian*(invM*Fext) + offset) ) );
	//-----------------------------------
	Mat<float> tempLambda( invA * ((-1.0f)*(constraintsJacobian*tempInvMFext + offset) ) );
	//-----------------------------------
	
	//Solutions :
	//------------------------------------
	//lambda = extract( &tempLambda, qdot.getLine()+1, 1, tempLambda.getLine(), 1);
	//if(isnanM(lambda))
	//	lambda = Mat<float>(0.0f,lambda.getLine(),lambda.getColumn());
	//Mat<float> udot( extract(  &tempLambda, 1,1, qdot.getLine(), 1) );
	//------------------------------------
	lambda = tempLambda;
	Mat<float> udot(  tConstraintsJacobian * tempLambda);
	//------------------------------------
	
	if(isnanM(udot))
		udot = Mat<float>(0.0f,udot.getLine(),udot.getColumn());
	
	
	float clampingVal = 1e4f;
	for(int i=1;i<=udot.getLine();i++)
	{
		if(udot.get(i,1) > clampingVal)
		{
			udot.set( clampingVal,i,1);
		}
	}
	
#ifdef debuglvl1	
	std::cout << " SOLUTIONS : udot and lambda/Pc : " << std::endl;
	transpose(udot).afficher();
	transpose(lambda).afficher();
	transpose( tConstraintsJacobian*lambda).afficher();
#endif	
	//Assumed model :
	//qdot = tempInvMFext + dt*extract(  &tempLambda, 1,1, qdot.getLine(), 1);
	//qdot = tempInvMFext + udot;
	qdot += tempInvMFext + invM*udot;
	//qdot += invM*udot;
	//qdot += tempInvMFext + udot;
	float clampingValQ = 1e3f;
	for(int i=1;i<=qdot.getLine();i++)
	{
		if( fabs_(qdot.get(i,1)) > clampingValQ)
		{
			qdot.set( clampingValQ * fabs_(qdot.get(i,1))/qdot.get(i,1),i,1);
		}
	}
	//qdot = udot;
	
	//Assumed model if the update of the integration is applied after that constraints solver :
	//qdot += dt*extract(  &tempLambda, 1,1, qdot.getLine(), 1);//+tempInvMFext
	
	Mat<float> t( dt*( S*qdot ) );	
	
	float clampQuat = 1e-1f;
	float idxQuat = 3;
	while(idxQuat < t.getLine())
	{
		for(int i=1;i<4;i++)
		{
			if( fabs_(t.get(idxQuat+i,1)) > clampQuat)
			{
				t.set( clampQuat*(t.get(idxQuat+i,1))/t.get(idxQuat+i,1), idxQuat+i,1);
			}
		}
		
		idxQuat += 7;
	}
	
	//the update is done by the update via an accurate integration and we must construct q and qdot at every step
	//q += t;
	
	//--------------------------------------	
	//let us normalize each quaternion :
	/*
	idxQuat = 3;
	while(idxQuat < q.getLine())
	{
		float scaler = q.get( idxQuat+4,1);
		
		if(scaler != 0.0f)
		{
			for(int i=1;i<=4;i++)
			{
				q.set( q.get(idxQuat+i,1)/scaler, idxQuat+i,1);
			}
		}
		
		idxQuat += 7;
	}
	*/
	
	//--------------------------------------
	
#ifdef debuglvl2	
	//std::cout << " computed Pc : " << std::endl;
	//(tConstraintsJacobian*tempLambda).afficher();
	//std::cout << " q+ : " << std::endl;
	//transpose(q).afficher();
	std::cout << " qdot+ : " << std::endl;
	transpose(qdot).afficher();
	std::cout << " qdotminus : " << std::endl;
	transpose(qdotminus).afficher();
#endif

#ifdef debuglvl3	
	std::cout << "SOME VERIFICATION ON : J*qdot + c = 0 : " << std::endl;
	transpose(constraintsJacobian*qdot+offset).afficher();
	
	
	float normC = (transpose(C)*C).get(1,1);
	Mat<float> Cdot( constraintsJacobian*qdot);
	float normCdot = (transpose(Cdot)*Cdot).get(1,1);
	float normQdot = (transpose(qdot)*qdot).get(1,1);
	
	//rs->ltadd(std::string("normC"),normC);
	//rs->ltadd(std::string("normCdot"),normCdot);
	rs->ltadd(std::string("normQdot"),normQdot);
	char name[5];
	for(int i=1;i<=t.getLine();i++)
	{
		sprintf(name,"dq%d",i);
		rs->ltadd(std::string(name), t.get(i,1));
	}
	rs->tWriteFileTABLE();
#endif	
	//END OF PREVIOUS METHOD :
	//--------------------------------
	
	//--------------------------------
	//--------------------------------
	//Second Method :
	/*
	//According to Tonge Richar's Physucs For Game pdf :
	Mat<float> tempLambda( (-1.0f)*invGJ( constraintsJacobian*invM.SM2mat()*tConstraintsJacobian)*constraintsJacobian*qdot );
	qdot += invM*tConstraintsJacobian*tempLambda;
	
	
	//qdot += tempInvMFext;	//contraints not satisfied.
	
	//qdot += tempInvMFext;	//qdot+ = qdot- + dt*M-1Fext;
	//Mat<float> qdotreal( qdot + dt*extract(  &tempLambda, 1,1, qdot.getLine(), 1) );	//qdotreal = qdot+ + Pc;
	
	Mat<float> t( dt*( S*qdot ) );
	q += t;
	*/
	//--------------------------------
	//--------------------------------
	//End of second method...
	//--------------------------------
	
	
	
	//--------------------------------
	//--------------------------------
	//THIRD METHOD :
	//--------------------------------
	//With reference to A Unified Framework for Rigid Body Dynamics Chap. 4.6.2.Simultaneous Force-based methods :
	//which refers to Bara96 :
	/*
	Mat<float> iM(invM.SM2mat());
	Mat<float> b((-1.0f)*constraintsJacobian*iM*Fext+offset);
	Mat<float> tempLambda( invGJ( constraintsJacobian*iM*tConstraintsJacobian) * b );
	//Mat<float> qdoubledot(iM*(tConstraintsJacobian*tempLambda+Fext)); 
	Mat<float> qdoubledot(iM*(tConstraintsJacobian*tempLambda)); 
	qdot += dt*qdoubledot;
	
	
	//qdot += tempInvMFext;	//contraints not satisfied.
	
	//qdot += tempInvMFext;	//qdot+ = qdot- + dt*M-1Fext;
	//Mat<float> qdotreal( qdot + dt*extract(  &tempLambda, 1,1, qdot.getLine(), 1) );	//qdotreal = qdot+ + Pc;
	
	Mat<float> t( dt*( S*qdot ) );
	q += t;
	
	std::cout << " computed Pc : " << std::endl;
	(tConstraintsJacobian*tempLambda).afficher();
	std::cout << " q+ : " << std::endl;
	q.afficher();
	std::cout << " qdot+ : " << std::endl;
	qdot.afficher();
	*/
	//END OF THIRD METHOD :
	//--------------------------------
	
	
	
	
	//S.print();
	
	//std::cout << " computed Pc : " << std::endl;
	//(tConstraintsJacobian*lambda).afficher();
	//std::cout << " delta state = S * qdotreal : " << std::endl; 
	//t.afficher();
	//std::cout << " S & qdotreal : " << std::endl;
	//S.print();
	//qdot.afficher();
	//std::cout << "invM*Fext : " << std::endl;
	//tempInvMFext.afficher();
	
	
	//temp.afficher();
	//(constraintsJacobian*(invM*Fext)).afficher();
	//(invM*Fext).afficher();
	//std::cout << " A : " << std::endl;
	//A.afficher();
    //std::cout << " SVD A*tA :  S : " << std::endl;
    //SVD<float> instanceSVD(A*transpose(A));
    //instanceSVD.getS().afficher();
	//std::cout << " invA : " << std::endl;
	//invA.afficher();
	
	//std::cout << " LAMBDA : " << std::endl;
	//lambda.afficher();
	
	//std::cout << " qdot+ & qdot- : " << std::endl;
	//qdot.afficher();
	//qdotminus.afficher();
	
	//std::cout << " q+ : " << std::endl;
	//q.afficher();
#ifdef debuglvl4	
	//BAUMGARTE STABILIZATION has been handled in the computeConstraintsANDJacobian function....
	std::cout << "tConstraints : norme  = " << norme2(C) << std::endl;
	transpose(C).afficher();
	std::cout << "Cdot : " << std::endl;
	(constraintsJacobian*qdot).afficher();
	std::cout << " JACOBIAN : " << std::endl;
	//transpose(constraintsJacobian).afficher();
	constraintsJacobian.afficher();
	std::cout << " Qdot+ : " << std::endl;
	transpose(qdot).afficher();
#endif	
	//BAUMGARTE STABILIZATION has been handled in the computeConstraintsANDJacobian function....
	//std::cout << "Constraints : norme  = " << norme2(C) << std::endl;
	//C.afficher();
	
}



/*	FORCE BASED : */
void SimultaneousImpulseBasedConstraintSolverStrategy::SolveForceBased(float dt, std::vector<std::unique_ptr<IConstraint> >& c, Mat<float>& q, Mat<float>& qdot, SparseMat<float>& invM, SparseMat<float>& S, const Mat<float>& Fext )
{	
	Mat<float> qdotminus(qdot);
	this->dt = dt;
	Mat<float> tempInvMFext( dt*(invM * Fext) ) ;


	computeConstraintsANDJacobian(c,q,qdot);
	
	Mat<float> tConstraintsJacobian( transpose(constraintsJacobian) );
	Mat<float> A( constraintsJacobian * invM.SM2mat() * tConstraintsJacobian );
	
	//---------------------------
	Mat<float> invA( invGJ(A) );
	
	//Construct b and compute the solution.
	//-----------------------------------
	Mat<float> tempLambda( invA * ((-1.0f)*(constraintsJacobian*tempInvMFext + offset) ) );
	//-----------------------------------
	
	//Solutions :
	//------------------------------------
	lambda = tempLambda;
	Mat<float> udot(  tConstraintsJacobian * tempLambda);
	//------------------------------------
	
	if(isnanM(udot))
		udot = Mat<float>(0.0f,udot.getLine(),udot.getColumn());
	
	
	float clampingVal = 1e4f;
	for(int i=1;i<=udot.getLine();i++)
	{
		if(udot.get(i,1) > clampingVal)
		{
			udot.set( clampingVal,i,1);
		}
	}
	
#ifdef debuglvl1	
	std::cout << " SOLUTIONS : udot and lambda/Pc : " << std::endl;
	transpose(udot).afficher();
	transpose(lambda).afficher();
	transpose( tConstraintsJacobian*lambda).afficher();
#endif	
	//Assumed model :
	qdot += tempInvMFext + dt*(invM*udot);
	float clampingValQ = 1e3f;
	for(int i=1;i<=qdot.getLine();i++)
	{
		if( fabs_(qdot.get(i,1)) > clampingValQ)
		{
			qdot.set( clampingValQ * fabs_(qdot.get(i,1))/qdot.get(i,1),i,1);
		}
	}
	
	//--------------------------------------
	
#ifdef debuglvl2	
	//std::cout << " computed Pc : " << std::endl;
	//(tConstraintsJacobian*tempLambda).afficher();
	//std::cout << " q+ : " << std::endl;
	//transpose(q).afficher();
	std::cout << " qdot+ : " << std::endl;
	transpose(qdot).afficher();
	std::cout << " qdotminus : " << std::endl;
	transpose(qdotminus).afficher();
#endif

	//END OF PREVIOUS METHOD :
	//--------------------------------
	
	
#ifdef debuglvl4	
	//BAUMGARTE STABILIZATION has been handled in the computeConstraintsANDJacobian function....
	std::cout << "tConstraints : norme  = " << norme2(C) << std::endl;
	transpose(C).afficher();
	std::cout << "Cdot : " << std::endl;
	(constraintsJacobian*qdot).afficher();
	std::cout << " JACOBIAN : " << std::endl;
	//transpose(constraintsJacobian).afficher();
	constraintsJacobian.afficher();
	std::cout << " Qdot+ : " << std::endl;
	transpose(qdot).afficher();
#endif	
	
	
}






















//-----------------------------------------
//-----------------------------------------
//-----------------------------------------

//-----------------------------------------
//-----------------------------------------
//-----------------------------------------


IterativeImpulseBasedConstraintSolverStrategy::IterativeImpulseBasedConstraintSolverStrategy(Simulation* sim_) : IConstraintSolverStrategy(sim_)
{

}

IterativeImpulseBasedConstraintSolverStrategy::~IterativeImpulseBasedConstraintSolverStrategy()
{

}


void IterativeImpulseBasedConstraintSolverStrategy::computeConstraintsANDJacobian(std::vector<std::unique_ptr<IConstraint> >& c, const Mat<float>& q, const Mat<float>& qdot, const SparseMat<float>& invM)
{
	//-------------------------------------
	//-------------------------------------
	//-------------------------------------
	
	size_t size = c.size();
	int n = sim->simulatedObjects.size();
	float baumgarteBAS = 0.0f;//1e-1f;
	float baumgarteC = -2e0f;
	float baumgarteH = 0.0f;//1e-1f;
	
	//---------------
	//	RESETTING :
	constraintsC.clear();
	constraintsJacobians.clear();
	constraintsOffsets.clear();
	constraintsIndexes.clear();
	constraintsInvM.clear();
	constraintsV.clear();
	//----------------------
	
	if( size > 0)
	{
		
		for(int k=0;k<size;k++)
		{
			int idA = ( c[k]->rbA.getID() );
			int idB = ( c[k]->rbB.getID() );
			std::vector<int> indexes(2);
			//indexes are set during the creation of the simulation and they begin at 0.
			indexes[0] = idA;
			indexes[1] = idB;
			
			constraintsIndexes.push_back( indexes );
			
			//---------------------------
			//Constraint :
			c[k]->computeJacobians();
			
			
			Mat<float> tJA(c[k]->getJacobianA());
			Mat<float> tJB(c[k]->getJacobianB());
		
			
			Mat<float> tC(c[k]->getConstraint());
			constraintsC.push_back( tC );
	
			int nbrlineJ = tJA.getLine();
			Mat<float> JacobianAB( operatorL(tJA, tJB)  );
			constraintsJacobians.push_back( JacobianAB );
			
			//----------------------------------------
			//BAUMGARTE STABILIZATION
			//----------------------------------------
			//Contact offset :
			if( c[k]->getType() == CTContactConstraint)
			{
				//----------------------------------------
				//SLOP METHOD :
				/*
				float slop = 1e0f;
				float pdepth = ((ContactConstraint*)(c[k].get()))->penetrationDepth;
				std::cout << " ITERATIVE SOLVER :: CONTACT : pDepth = " << pdepth << std::endl;
				tC *= baumgarteC/this->dt * fabs_(fabs_(pdepth)-slop);			
				*/
				//----------------------------------------
				
				//----------------------------------------
				//DEFAULT METHOD :
				tC *= baumgarteC/this->dt;
				//----------------------------------------
				
				//----------------------------------------
				//METHOD 2 :
				/*
				float restitFactor = ( (ContactConstraint*) (c[k].get()) )->getRestitutionFactor();
				std::cout << " ITERATIVE SOLVER :: CONTACT : restitFactor = " << restitFactor << std::endl;
				Mat<float> Vrel( ( (ContactConstraint*) (c[k].get()) )->getRelativeVelocity() );
				Mat<float> normal( ( (ContactConstraint*) (c[k].get()) )->getNormalVector() );
				std::cout << " ITERATIVE SOLVER :: CONTACT : Normal vector : " << std::endl;
				transpose(normal).afficher();
				tC +=  restitFactor * transpose(Vrel)*normal; 
				*/
				//----------------------------------------
				
				std::cout << " ITERATIVE SOLVER :: CONTACT : Contact Constraint : " << std::endl;
				transpose(tC).afficher();
				std::cout << " ITERATIVE SOLVER :: CONTACT : Relative Velocity vector : " << std::endl;
				transpose(( (ContactConstraint*) (c[k].get()) )->getRelativeVelocity()).afficher();
				//std::cout << " ITERATIVE SOLVER :: CONTACT : First derivative of Contact Constraint : " << std::endl;
				//(transpose(tJA)*).afficher();
					
			}
			//BAS JOINT :
			if( c[k]->getType() == CTBallAndSocketJoint)
			{
				tC *= baumgarteBAS/this->dt;
			}
			
			//HINGE JOINT :
			if( c[k]->getType() == CTHingeJoint)
			{
				tC *= baumgarteH/this->dt;
			}
			
			//BAUMGARTE OFFSET for the moments...
			constraintsOffsets.push_back( tC );
			
			//----------------------------------------
			//----------------------------------------
			
			
			//-------------------
			//	invM matrixes :
			Mat<float> invmij(0.0f,12,12);
			for(int k=0;k<=1;k++)
			{
				for(int i=1;i<=6;i++)
				{
					for(int j=1;j<=6;j++)
					{
						invmij.set( invM.get( indexes[k]*6+i, indexes[k]*6+j), k*6+i,k*6+j);
						
					}
				}
			}
			
			constraintsInvM.push_back( invmij);
			
			
			//-------------------
			//	Vdot matrixes :
			Mat<float> vij(0.0f,12,1);
			for(int k=0;k<=1;k++)
			{
				for(int i=1;i<=6;i++)
				{
					vij.set( qdot.get( indexes[k]*6+i, 1), k*6+i,1);
				}
			}
			
			constraintsV.push_back( vij);
			
			
			
		}
		
	}	
}

void IterativeImpulseBasedConstraintSolverStrategy::wrappingUp(std::vector<std::unique_ptr<IConstraint> >& c, const Mat<float>& q, const Mat<float>& qdot)
{
	//Let us delete the contact constraints that are ephemerous by essence :
	std::vector<std::unique_ptr<IConstraint> >::iterator itC = c.begin();
	bool erased = false;
	while(itC != c.end())
	{
		if( (itC->get())->getType() == CTContactConstraint )
		{
			erased = true;
			c.erase(itC);
		}

		if(!erased)
			itC++;
		
		erased = false;
	}
}


void IterativeImpulseBasedConstraintSolverStrategy::Solve(float dt, std::vector<std::unique_ptr<IConstraint> >& c, Mat<float>& q, Mat<float>& qdot, SparseMat<float>& invM, SparseMat<float>& S, const Mat<float>& Fext )
{
	float clampingVal = 1e20f;
	float clampingValQ = 1e20f;

	Mat<float> qdotminus(qdot);
	this->dt = dt;
	Mat<float> tempInvMFext( dt*(invM * Fext) ) ;
	
	
	std::vector<float> constraintsNormeCdotAfterImpulses(c.size(),1.0f);
	std::vector<Mat<float> > constraintsVAfterImpulses;
	
	
	bool continuer = true;
	int nbrIteration = 0;
	while( continuer)
	{
		
		computeConstraintsANDJacobian(c,q,qdot,invM);	
		constraintsImpulses.clear();	
		constraintsImpulses.resize(constraintsC.size());
		
		if( nbrIteration == 0)
		{
			constraintsVAfterImpulses = constraintsV;
		}
		
		int nbrConstraintsSolved = 0;	
		
		for(int k=0;k<constraintsC.size();k++)
		{
			if(constraintsNormeCdotAfterImpulses[k] >= 0.0f)
			{
				Mat<float> tConstraintsJacobian( transpose( constraintsJacobians[k] ) );
				Mat<float> A( constraintsJacobians[k] * constraintsInvM[k] * tConstraintsJacobian );
	
				//Construct the inverse matrix :
				//---------------------------
				Mat<float> invA( invGJ(A) );
	
				//Construct b and compute the solution.
				//----------------------------------
				Mat<float> tempLambda( invA * ((-1.0f)*(constraintsJacobians[k]*constraintsV[k] + constraintsOffsets[k]) ) );
				//-----------------------------------
	
				//Solution Impulse :
				//------------------------------------
				constraintsImpulses[k] =  tConstraintsJacobian * tempLambda;
				Mat<float> udot( constraintsInvM[k]*constraintsImpulses[k]);
				//------------------------------------
	
				if(isnanM(udot))
				{
					std::cout << " ITERATIVE SOLVER :: udot :: NAN ISSUE : " << std::endl;
					transpose(udot).afficher();
					udot = Mat<float>(0.0f,udot.getLine(),udot.getColumn());
				}
	
	
				/*
				for(int i=1;i<=udot.getLine();i++)
				{
					if(udot.get(i,1) > clampingVal)
					{
						udot.set( clampingVal,i,1);
					}
				}
				*/
			
				//---------------------------------			
				// UPDATE OF THE VELOCITY :
				//---------------------------------
				std::vector<int> indexes = constraintsIndexes[k];
				//constraintsVAfterImpulses[k] = constraintsV[k]-udot;
				constraintsVAfterImpulses[k] = constraintsV[k]+udot;
			
				std::cout << " UDOT : ids : " << indexes[0] << "  and " << indexes[1] << std::endl;
				transpose(udot).afficher();
			
				for(int kk=0;kk<=1;kk++)
				{
					for(int i=1;i<=6;i++)
					{
						qdot.set( constraintsVAfterImpulses[k].get(kk*6+i,1), indexes[kk]*6+i, 1);
					}
				}
			
				//---------------------------------	
	
				/*
				for(int i=1;i<=qdot.getLine();i++)
				{
					if( fabs_(qdot.get(i,1)) > clampingValQ)
					{
						qdot.set( clampingValQ * fabs_(qdot.get(i,1))/qdot.get(i,1),i,1);
					}
				}
				*/
					
			}
			else
			{
				std::cout << "CONSTRAINTS : " << k << "/" << constraintsC.size() << " : has been solved for." << std::endl;
				
			}
					
		}
	
	
		for(int k=0;k<constraintsC.size();k++)
		{
//#ifdef debuglvl3	
				std::cout << "SOME VERIFICATION ON : J*qdot + c = 0 :  k = " << k << std::endl;
				Mat<float> cdot( constraintsJacobians[k]*constraintsVAfterImpulses[k]+constraintsOffsets[k]);
				transpose(cdot).afficher();
					
				constraintsNormeCdotAfterImpulses[k] = (transpose(cdot)*cdot).get(1,1);
	
				std::cout << "NORME : "<< k << " : " << constraintsNormeCdotAfterImpulses[k] << std::endl;
				
				if( constraintsNormeCdotAfterImpulses[k] <= 1e-20f)
				{
					nbrConstraintsSolved++;
				}
//#endif
		}
		
		
		if(nbrConstraintsSolved == constraintsC.size() )
		{
			continuer = false;
			std::cout << " IterativeImpulseBasedConstraintsSolver::Solve :: Constraints solved : " << nbrConstraintsSolved << " / " << constraintsC.size() << " :: ENDED CLEANLY." << std::endl;
		}
		else
		{
			continuer = true;
			nbrIteration++;
		
			if(nbrIteration > this->nbrIterationSolver)
			{
				continuer = false;
				std::cout << " IterativeImpulseBasedConstraintsSolver::Solve :: Constraints solved : " << nbrConstraintsSolved << " / " << constraintsC.size() << " :: ENDED WITH UNRESOLVED CONSTRAINTS." << std::endl;
			}
		}
		//--------------------------------------
		
#ifdef debuglvl4	
		std::cout << " Qdot+ : " << std::endl;
		transpose(qdot).afficher();
#endif

	
	}
	
	wrappingUp(c,q,qdot);
}





/*	FORCE BASED : */
void IterativeImpulseBasedConstraintSolverStrategy::SolveForceBased(float dt, std::vector<std::unique_ptr<IConstraint> >& c, Mat<float>& q, Mat<float>& qdot, SparseMat<float>& invM, SparseMat<float>& S, const Mat<float>& Fext )
{	

	
}






