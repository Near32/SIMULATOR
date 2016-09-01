#include "EtatEngine.h"
#include "Game.h"

#undef NDEBUG
#define debug

extern std::mutex ressourcesMutex;


EtatEngine::EtatEngine(Game* game_, GameState gameState_) : IEngine(game_,gameState_)
{
	env = new Environnement();
	
	init();
	
}

EtatEngine::~EtatEngine()
{
	delete env;
	delete sim;
}
	
void EtatEngine::loop()
{
	ressourcesMutex.lock();
	bool gameON = game->gameON;
	ressourcesMutex.unlock();
	
	while(gameON)
	{
		if( commandsToHandle.size() > 0)
		{
			//let's verify that it is one of those dedicated commands :
			switch( (commandsToHandle[0].get())->getCommandType())
			{
				//DEBUGGING :
				case TCSimulateStride:
				{
				float timestep = 1e-2f;
				float time = sim->getTime();
#ifdef debug
std::cout << "SIMULATION : run : ..." << std::endl;
#endif					
				sim->run(timestep,time+timestep);
#ifdef debug
std::cout << "SIMULATION : run " << sim->getTime() << " : successfully !!" << std::endl;
#endif				
				
				commandsToHandle.erase(commandsToHandle.begin());
				}
				break;
				
				//DEBUGGING :
				case TCSimulation:
				{
				float timestep = 1e-2f;
				float time = sim->getTime();
#ifdef debug
std::cout << "SIMULATION : run : ..." << std::endl;
#endif					
				sim->run(timestep,time+5.0f);
#ifdef debug
std::cout << "SIMULATION : run " << sim->getTime() << " / " << time+5.0f << " : successfully !!" << std::endl;
#endif				
				
				commandsToHandle.erase(commandsToHandle.begin());
				}
				break;
				
				
				default :
				{
				commandsToHandle.erase(commandsToHandle.begin());
				}
				break;
			}
		}
		
		ressourcesMutex.lock();
		gameON = game->gameON;
		ressourcesMutex.unlock();
	}
}




void EtatEngine::init()
{
	//let's create the Elements that we need.
	Mat<float> hwd(500.0f,3,1);
	Mat<float> t((float)0,3,1);
	
	ConstraintsList cl;
	
	//--------------------------------
	//map : ground :
	//ground :
	t.set( -hwd.get(3,1)/2,3,1);
	env->addElement( env->fabriques->fabriquer(ELFObstacle, std::string("ground"), new se3(t), hwd ) );
	//
	//sa position est bien à l'origine..
	//resetting :
	t *= 0.0f;
	hwd = Mat<float>(1.0f,3,1);
	//--------------------------------
	
	
	//----------------------------------
	// OBS ELEMENT FOR : HINGEJOINT CONTRAINTS
	//float offset = 30.0f;
	//hwd *= 10.0f;
	//hwd.set(20.0f,3,1);
	
	//t.set( hwd.get(2,1)/2+1.0f, 3,1);
	//t.set( hwd.get(1,1),1,1);
	//se3 obs_se3(t);
	//t.set( 0.0f,1,1);
	//t.set( hwd.get(3,1)/2, 3,1);
	
	//float roll = +PI/2-0.1f;
	//float pitch = 0.0f;
	//float yaw = 0.0f;
	//Quat q = Euler2Qt(roll,pitch,yaw);
	//obs_se3.setOrientation( q );
	//env->addElement( new ElementMobile(std::string("OBS"), new se3(obs_se3), hwd) );
	
	
	//constraints :
	
	
	
	
	//TESTING ROBOTS :
	float angle = 0.0f;
	float nbrRobot = 3;
	float step = 2*PI/3;
	float radius = 60.0f;
	
	//TESTING ROBOT 1 :
	
	//resetting :
	t = Mat<float>(0.0f,3,1);
	hwd = Mat<float>(10.0f,3,1);
	hwd.set( 20.0f, 3,1);
	//--------------------------------
	t.set(20.0f,2,1);
	t.set( hwd.get(3,1), 3,1);	
	env->addElement( new ElementRobot(std::string("R0"), new se3(t), hwd) );
	//--------------------------------
	
	//TESTING ROBOT 2 :
	
	//resetting :
	for(int k=1;k<=nbrRobot;k++)
	{
		t = Mat<float>(0.0f,3,1);
		hwd = Mat<float>(10.0f,3,1);
		hwd.set( 20.0f, 3,1);
		//--------------------------------
		angle+= step;
		t.set( radius*cos(angle*PI/180.0f), 1,1); 
		t.set( radius*sin(angle*PI/180.0f),2,1);
		
		t.set( hwd.get(3,1), 3,1);
			
		env->addElement( new ElementRobot(std::string("R")+std::to_string(k), new se3(t), hwd) );
		//--------------------------------
	}

	sim = new Simulation(env,cl);
		
}






void EtatEngine::init1()
{
	//let's create the Elements that we need.
	Mat<float> hwd(500.0f,3,1);
	Mat<float> t((float)0,3,1);
	
	ConstraintsList cl;
	
	//--------------------------------
	//create the elements :
	//map : ground :
	//ground :
	t.set( -hwd.get(3,1)/2,3,1);
	env->addElement( env->fabriques->fabriquer(ELFObstacle, std::string("ground"), new se3(t), hwd ) );
	//
	//sa position est bien à l'origine..
	//resetting :
	t *= 0.0f;
	hwd = Mat<float>(1.0f,3,1);
	//--------------------------------
	
	
	//Gunvarrel :
	float offset = 30.0f;
	hwd *= 10.0f;
	hwd.set(20.0f,3,1);
	
	t.set( hwd.get(2,1)/2+1.0f, 3,1);
	t.set( hwd.get(1,1),1,1);
	se3 obs_se3(t);
	t.set( 0.0f,1,1);
	t.set( hwd.get(3,1)/2, 3,1);
	
	float roll = +PI/2-0.1f;
	float pitch = 0.0f;
	float yaw = 0.0f;
	Quat q = Euler2Qt(roll,pitch,yaw);
	//Quat q;
	//q.x = sin(-PI/4);
	//q.w = cos(-PI/4);
	//std::cout << " Quat : " << q.x << " : " << q.y << " : " << q.z << " : " << q.w << std::endl;
	//Qt2Euler(q, &roll, &pitch, &yaw);
	//std::cout << " Quat2Euler results : " << roll << " : " << pitch << " : " << yaw << std::endl;
	obs_se3.setOrientation( q );
	env->addElement( new ElementMobile(std::string("OBS"), new se3(obs_se3), hwd) );
	
	t.set( t.get(3,1)+2.0f+offset, 3,1);	
	env->addElement( new ElementMobile(std::string("picBAS"), new se3(t), hwd) );
	
	t.set( t.get(3,1)+hwd.get(3,1)+2.0f, 3,1);
	
	t.set( t.get(2,1)+0.0f, 2,1);
		
	env->addElement( new ElementMobile(std::string("picHAUT"), new se3(t), hwd) );
	
	//constraints :
	
	Mat<float> AnchorAL(0.0f,3,1);
	AnchorAL.set( hwd.get(3,1)/2+1.0f, 3,1);
	//Mat<float> HJAxis(0.0f,3,1);
	//HJAxis.set( 1.0f, 1,1);
	Mat<float> AnchorBL(0.0f,3,1);
	AnchorBL.set( -hwd.get(3,1)/2-1.0f, 3,1);
	cl.insert( cl.end(), ConstraintInfo(std::string("picBAS"),std::string("picHAUT"), CTBallAndSocketJoint, operatorL(AnchorAL,AnchorBL) ) ); 
	
	
	
	//TESTING ROBOT 1 :
	
	//resetting :
	t = Mat<float>(0.0f,3,1);
	hwd = Mat<float>(10.0f,3,1);
	hwd.set( 20.0f, 3,1);
	//--------------------------------
	t.set(50.0f,2,1);
	t.set( hwd.get(3,1), 3,1);	
	env->addElement( new ElementRobot(std::string("R1"), new se3(t), hwd) );
	//--------------------------------
	
	
	/*
	//resetting :
	t = Mat<float>(0.0f,3,1);
	hwd = Mat<float>(10.0f,3,1);
	hwd.set( 20.0f, 3,1);
	//--------------------------------
	t.set(50.0f,2,1);
	//HEAD :
	float headZ = 150.0f;
	t.set( headZ, 3,1);	
	env->addElement( new ElementMobile(std::string("Gunvarrel-HEAD"), new se3(t), hwd) );
	//--------------------------------
	//CHEST_HIGH :
	roll = +PI/2-0.001f;
	pitch = 0.0f;
	yaw = 0.0f;
	q = Euler2Qt(roll,pitch,yaw);
	t.set( t.get(3,1)-hwd.get(3,1)/2-hwd.get(2,1)/2-2.0f, 3,1);
	se3 poseCHESTHIGH(t);
	poseCHESTHIGH.setOrientation( q );
	env->addElement( new ElementMobile(std::string("Gunvarrel-CHEST_HIGH"), new se3(poseCHESTHIGH), hwd) );
	//--------------------------------
	//Constraint : HEAD - CHEST_HIGH:
	AnchorAL = Mat<float>(0.0f,3,1);
	AnchorAL.set( -hwd.get(3,1)/2-1.0f, 3,1);
	AnchorBL = Mat<float>(0.0f,3,1);
	AnchorBL.set( -hwd.get(2,1)/2-1.0f, 2,1);
	cl.insert( cl.end(), ConstraintInfo(std::string("Gunvarrel-HEAD"),std::string("Gunvarrel-CHEST_HIGH"), CTBallAndSocketJoint, operatorL(AnchorAL,AnchorBL) ) );
	
	//CHEST_LOW :
	t.set( t.get(3,1)-hwd.get(3,1)/2-hwd.get(2,1)/2-2.0f, 3,1);
	env->addElement( new ElementMobile(std::string("Gunvarrel-CHEST_LOW"), new se3(t), hwd) );
	//--------------------------------
	//Constraint : CHEST_HIGH - CHEST_LOW:
	AnchorAL = Mat<float>(0.0f,3,1);
	AnchorAL.set( hwd.get(2,1)/2+1.0f, 2,1);
	AnchorBL = Mat<float>(0.0f,3,1);
	AnchorBL.set( hwd.get(3,1)/2+1.0f, 3,1);
	cl.insert( cl.end(), ConstraintInfo(std::string("Gunvarrel-CHEST_HIGH"),std::string("Gunvarrel-CHEST_LOW"), CTBallAndSocketJoint, operatorL(AnchorAL,AnchorBL) ) );
	
	//BASSIN :
	t.set( t.get(3,1)-hwd.get(3,1)/2-hwd.get(2,1)/2-6.0f, 3,1);
	se3 poseBASSIN(t);
	poseBASSIN.setOrientation( q );
	env->addElement( new ElementMobile(std::string("Gunvarrel-BASSIN"), new se3(poseBASSIN), hwd) );
	//--------------------------------
	//Constraint : CHEST_LOW - BASSIN:
	AnchorAL = Mat<float>(0.0f,3,1);
	AnchorAL.set( -hwd.get(3,1)/2-0.0f, 3,1);
	AnchorAL.afficher();
	AnchorBL = Mat<float>(0.0f,3,1);
	AnchorBL.set( -hwd.get(2,1)/2-6.0f, 2,1);
	cl.insert( cl.end(), ConstraintInfo(std::string("Gunvarrel-CHEST_LOW"),std::string("Gunvarrel-BASSIN"), CTBallAndSocketJoint, operatorL(AnchorAL,AnchorBL) ) );
	
	//LEGR_HIGH :
	t.set( t.get(3,1)-hwd.get(3,1)/2-hwd.get(2,1)/2-6.0f, 3,1);
	t.set( t.get(2,1)-hwd.get(3,1)/2+hwd.get(2,1)/2-2.0f, 2,1);
	env->addElement( new ElementMobile(std::string("Gunvarrel-LEGR_HIGH"), new se3(t), hwd) );
	//--------------------------------
	//Constraint : BASSIN - LEGR_HIGH:
	AnchorAL = Mat<float>(0.0f,3,1);
	AnchorAL.set( hwd.get(2,1)/2+0.0f, 2,1);
	AnchorAL.set( -hwd.get(3,1)/2+hwd.get(2,1)/2-2.0f, 3,1);
	//the rotation of the bassin has put y in z and z in -y. Thus the right is pointing in the direction of -z.
	AnchorBL = Mat<float>(0.0f,3,1);
	AnchorBL.set( hwd.get(3,1)/2+6.0f, 3,1);
	cl.insert( cl.end(), ConstraintInfo(std::string("Gunvarrel-BASSIN"),std::string("Gunvarrel-LEGR_HIGH"), CTBallAndSocketJoint, operatorL(AnchorAL,AnchorBL) ) );
	
	//LEGL_HIGH :
	//t.set( t.get(3,1)-hwd.get(3,1)/2-hwd.get(2,1)/2-2.0f, 3,1);
	t.set( t.get(2,1)+hwd.get(3,1)-hwd.get(2,1)+4.0f, 2,1);
	env->addElement( new ElementMobile(std::string("Gunvarrel-LEGL_HIGH"), new se3(t), hwd) );
	//--------------------------------
	//Constraint : BASSIN - LEGL_HIGH:
	AnchorAL = Mat<float>(0.0f,3,1);
	AnchorAL.set( hwd.get(2,1)/2+0.0f, 2,1);
	AnchorAL.set( hwd.get(3,1)/2-hwd.get(2,1)/2+2.0f, 3,1);
	//the rotation of the bassin has put y in z and z in -y. Thus the right is pointing in the direction of -z.
	AnchorBL = Mat<float>(0.0f,3,1);
	AnchorBL.set( hwd.get(3,1)/2+6.0f, 3,1);
	cl.insert( cl.end(), ConstraintInfo(std::string("Gunvarrel-BASSIN"),std::string("Gunvarrel-LEGL_HIGH"), CTBallAndSocketJoint, operatorL(AnchorAL,AnchorBL) ) );
	
	//LEGL_LOW :
	t.set( t.get(3,1)-hwd.get(3,1)-8.0f, 3,1);
	env->addElement( new ElementMobile(std::string("Gunvarrel-LEGL_LOW"), new se3(t), hwd) );
	//--------------------------------
	//Constraint : LEGL_HIGH - LEGL_LOW:
	AnchorAL = Mat<float>(0.0f,3,1);
	AnchorAL.set( -hwd.get(3,1)/2-4.0f, 3,1);
	AnchorAL.afficher();
	//the rotation of the bassin has put y in z and z in -y. Thus the right is pointing in the direction of -z.
	//AnchorBL = Mat<float>(0.0f,3,1);
	//AnchorBL.set( hwd.get(3,1)/2+4.0f, 3,1);
	Mat<float> HingeAxisAL(0.0f,3,1);
	HingeAxisAL.set(1.0f,2,1);
	cl.insert( cl.end(), ConstraintInfo(std::string("Gunvarrel-LEGL_HIGH"),std::string("Gunvarrel-LEGL_LOW"), CTHingeJoint, operatorL(HingeAxisAL,AnchorAL) ) );
	//cl.insert( cl.end(), ConstraintInfo(std::string("Gunvarrel-LEGL_HIGH"),std::string("Gunvarrel-LEGL_LOW"), CTBallAndSocketJoint, operatorL(AnchorAL,AnchorBL) ) );
	
	//LEGR_LOW :
	//t.set( t.get(3,1)-hwd.get(3,1)/2-8.0f, 3,1);
	t.set( t.get(2,1)-hwd.get(3,1)+hwd.get(2,1)-4.0f, 2,1);
	env->addElement( new ElementMobile(std::string("Gunvarrel-LEGR_LOW"), new se3(t), hwd) );
	//--------------------------------
	//Constraint : LEGR_HIGH - LEGR_LOW:
	AnchorAL = Mat<float>(0.0f,3,1);
	AnchorAL.set( -hwd.get(3,1)/2-4.0f, 3,1);
	//the rotation of the bassin has put y in z and z in -y. Thus the right is pointing in the direction of -z.
	AnchorBL = Mat<float>(0.0f,3,1);
	AnchorBL.set( hwd.get(3,1)/2+4.0f, 3,1);
	HingeAxisAL = Mat<float>(0.0f,3,1);
	HingeAxisAL.set(-1.0f,2,1);
	//cl.insert( cl.end(), ConstraintInfo(std::string("Gunvarrel-LEGR_HIGH"),std::string("Gunvarrel-LEGR_LOW"), CTHingeJoint, operatorL(HingeAxisAL,AnchorAL) ) );
	cl.insert( cl.end(), ConstraintInfo(std::string("Gunvarrel-LEGR_HIGH"),std::string("Gunvarrel-LEGR_LOW"), CTBallAndSocketJoint, operatorL(AnchorAL,AnchorBL) ) );
	
	
	//FOOTR :
	roll = 0.0f;
	pitch = PI/2-0.001f;
	yaw = 0.0f;
	q = Euler2Qt(roll,pitch,yaw);
	t.set( t.get(3,1)-hwd.get(3,1)/2-hwd.get(2,1)/2-4.0f, 3,1);
	t.set( t.get(1,1)+hwd.get(3,1)/4, 1,1);
	se3 poseFOOT(t);
	poseFOOT.setOrientation( q );
	env->addElement( new ElementMobile(std::string("Gunvarrel-FOOTR"), new se3(poseFOOT), hwd) );
	//--------------------------------
	//Constraint : FOOTR - LEGR_LOW:
	AnchorAL = Mat<float>(0.0f,3,1);
	AnchorAL.set( -hwd.get(3,1)/2-2.0f, 3,1);
	AnchorBL = Mat<float>(0.0f,3,1);
	AnchorBL.set( -hwd.get(2,1)/2-2.0f, 1,1);
	AnchorBL.set( -hwd.get(3,1)/4, 3,1);
	cl.insert( cl.end(), ConstraintInfo(std::string("Gunvarrel-LEGR_LOW"),std::string("Gunvarrel-FOOTR"), CTBallAndSocketJoint, operatorL(AnchorAL,AnchorBL) ) );
	*/
	
	
	/*
	//map : obstacles :
	int nbrObstacle = 0;
	//on veut changer sa position avec une hauteur un peu plus grande :
	t.set(20.0f,3,1);
	t.set(10.0f,1,1);
	t.set(30.0f,2,1); 
	env->addElement( env->fabriques->fabriquer(ELFObstacle, std::string("obstacle")+std::to_string(nbrObstacle), new se3(t), hwd ) );
	nbrObstacle++;
	//--------------------------------
	
	
	//map : orbe bonus :
	//orbe bonus :
	int nbrOrbeBonus = 0;
	//on veut changer sa position avec une hauteur un peu plus grande :
	t.set(20.0f,3,1);
	t.set(30.0f,1,1);
	t.set(30.0f,2,1); 
	env->addElement( env->fabriques->fabriquer(ELFOrbeBonus, std::string("orbebonus")+std::to_string(nbrOrbeBonus), new se3(t), hwd ) );
	nbrOrbeBonus++;
	*/
	//--------------------------------
	
	//-------------------------------
	
	sim = new Simulation(env,cl);
		
}






