#ifndef ICONSTRAINTSOLVERSTRATEGY
#define ICONSTRAINTSOLVERSTRATEGY


#include "../RigidBody.h"
#include "../IConstraint.h"
#include "../../utils/math.h"
#include "../../utils/RunningStats/RunningStats.h"
#include <stdlib.h>
#include <stdio.h>

class Simulation;

class IConstraintSolverStrategy
{
	
	protected :
	
	Mat<float> C;
	Mat<float> constraintsJacobian;
	Mat<float> constraintsImpulse;
	Mat<float> offset;
	Mat<float> lambda;
	Mat<float> M;
	
	float dt;
	
	Simulation* sim;
	
	public :
	
	RunningStats<float>* rs;
	int nbrIterationSolver;
	
	//-------------------------------------
	
	IConstraintSolverStrategy(Simulation* sim);
	
	~IConstraintSolverStrategy();
	
	
	virtual void Solve(float dt, std::vector<std::unique_ptr<IConstraint> >& c, Mat<float>& q, Mat<float>& qdot, SparseMat<float>& invM, SparseMat<float>& S, const Mat<float>& Fext) = 0;
	virtual void SolveForceBased(float dt, std::vector<std::unique_ptr<IConstraint> >& c, Mat<float>& q, Mat<float>& qdot, SparseMat<float>& invM, SparseMat<float>& S, const Mat<float>& Fext) = 0;

	//------------------------------------
	
	Mat<float> getConstraints()	const;
	
	Mat<float> getConstraintsJacobian() const;
	
	Mat<float> getLambda()	const;
	
	
		
	
};




//-----------------------------------------
//-----------------------------------------
//-----------------------------------------
//-----------------------------------------



class SimultaneousImpulseBasedConstraintSolverStrategy : public IConstraintSolverStrategy
{
	private :
	
	
	//-----------------------------
	
	public :
	
	SimultaneousImpulseBasedConstraintSolverStrategy(Simulation* sim);
	
	~SimultaneousImpulseBasedConstraintSolverStrategy();
	
	
	void computeConstraintsJacobian(std::vector<std::unique_ptr<IConstraint> >& c);
	void computeConstraintsJacobian(std::vector<std::unique_ptr<IConstraint> >& c, const Mat<float>& q, const Mat<float>& qdot);
	void computeConstraintsANDJacobian(std::vector<std::unique_ptr<IConstraint> >& c, const Mat<float>& q, const Mat<float>& qdot);
		
	virtual void Solve(float dt, std::vector<std::unique_ptr<IConstraint> >& c, Mat<float>& q, Mat<float>& qdot, SparseMat<float>& invM, SparseMat<float>& S, const Mat<float>& Fext ) override;
	virtual void SolveForceBased(float dt, std::vector<std::unique_ptr<IConstraint> >& c, Mat<float>& q, Mat<float>& qdot, SparseMat<float>& invM, SparseMat<float>& S, const Mat<float>& Fext ) override;

};


class IterativeImpulseBasedConstraintSolverStrategy : public IConstraintSolverStrategy
{
	private :
	
	std::vector<Mat<float> > constraintsC;
	std::vector<Mat<float> > constraintsJacobians;
	std::vector<Mat<float> > constraintsOffsets;
	std::vector<std::vector<int> > constraintsIndexes;
	std::vector<Mat<float> > constraintsInvM;
	std::vector<Mat<float> > constraintsV;
	
	std::vector<Mat<float> > constraintsImpulses;
	
	
	//-----------------------------
	
	public :
	
	IterativeImpulseBasedConstraintSolverStrategy(Simulation* sim);
	
	~IterativeImpulseBasedConstraintSolverStrategy();
	
	
	void computeConstraintsANDJacobian(std::vector<std::unique_ptr<IConstraint> >& c, const Mat<float>& q, const Mat<float>& qdot, const SparseMat<float>& invM);
		
	virtual void Solve(float dt, std::vector<std::unique_ptr<IConstraint> >& c, Mat<float>& q, Mat<float>& qdot, SparseMat<float>& invM, SparseMat<float>& S, const Mat<float>& Fext ) override;
	
	virtual void SolveForceBased(float dt, std::vector<std::unique_ptr<IConstraint> >& c, Mat<float>& q, Mat<float>& qdot, SparseMat<float>& invM, SparseMat<float>& S, const Mat<float>& Fext ) override;
	
	protected :
	
	void wrappingUp(std::vector<std::unique_ptr<IConstraint> >& c, const Mat<float>& q, const Mat<float>& qdot);

};


#endif
