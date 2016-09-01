#include "Game.h"

void mainTestQuat(int argc,char* argv[]);
void mainTestQuat1(int argc,char* argv[]);
void mainTestQuat2(int argc,char* argv[]);

int main(int argc, char* argv[])
{
	Game game;
	
	game.loop();
	
	//mainTestQuat(argc,argv);
	//mainTestQuat1(argc,argv);
	//mainTestQuat2(argc,argv);
	
	return 0;
}

void mainTestQuat(int argc,char* argv[])
{
	Quat q;
	q.x = 0;
	q.y = 0;
	q.z = 0;
	q.w = 1;
	
	/*Mat<float> angularVel(0.0f,3,1);
	angularVel.set( 1.0f, 1,1);
	*/
	float avstep = PI/4;
	
	for(int k=0;k<=100;k++)
	{
		float dt = 0.01f;
		Quat w;
		w.x = avstep;
		w.y = 0;
		w.z = 0;
		w.w = 0;
		
		Quat qdot( 0.5f * Qt_Mul(w,q) );
		
		//NO NORMALIZATION REQUIRED !
		//-----------------
		q = q+dt*qdot;
		
		//Normalization :
		q.x /= q.w;
		q.y /= q.w;
		q.z /= q.w;
		q.w = 1;
		//---------------------
		
		float roll,pitch,yaw;
		Qt2Euler(q, &roll, &pitch, &yaw); 
		std::cout << "iteration : " << k*dt << " roll pitch yaw :: " << roll*180.0/PI << " : " << pitch*180.0/PI << " : " << yaw*180.0/PI << std::endl;
		transpose(Qt2Mat<float>(qdot)).afficher();
	}
}

void mainTestQuat1(int argc,char* argv[])
{
	Quat q;
	q.x = 0;
	q.y = 0;
	q.z = -0.38f;
	q.w = 1;
	
	float av = PI/4;
	
	float roll,pitch,yaw;
	Qt2Euler(q, &roll, &pitch, &yaw); 
	
	Mat<float> qmat1( Qt2Mat<float>(q) );
	std::cout << " Q : " << std::endl;
	transpose( qmat1 ).afficher();
	
	std::cout << " roll pitch yaw :: " << roll*180.0/PI << " : " << pitch*180.0/PI << " : " << yaw*180.0/PI << std::endl;
	
	
	//yaw += av;
	
	q = Euler2Qt(roll,pitch,yaw);
	
	Mat<float> qmat( Qt2Mat<float>(q) );
	std::cout << " Q : " << std::endl;
	transpose( qmat ).afficher();
	
	
	Quat qp( q);
	Qt2Euler(qp, &roll, &pitch, &yaw);
	
	std::cout << " roll pitch yaw :: " << roll*180.0/PI << " : " << pitch*180.0/PI << " : " << yaw*180.0/PI << std::endl;
	Qt2Euler(qp, &roll, &pitch, &yaw);
	
	std::cout << " roll pitch yaw :: " << roll*180.0/PI << " : " << pitch*180.0/PI << " : " << yaw*180.0/PI << std::endl;
	
}


void mainTestQuat2(int argc,char* argv[])
{
	Quat q;
	q.x = 0;
	q.y = 0;
	q.z = 0;
	q.w = 1;
	
	/*Mat<float> angularVel(0.0f,3,1);
	angularVel.set( 1.0f, 1,1);
	*/
	float avstep = PI/8;
	float dt = 0.01f;
	
	for(int k=0;k<=100;k++)
	{		
		float roll,pitch,yaw;
		Qt2Euler(q, &roll, &pitch, &yaw); 
		
		std::cout << "iteration : " << k*dt << " roll pitch yaw :: " << roll*180.0/PI << " : " << pitch*180.0/PI << " : " << yaw*180.0/PI << std::endl;
		
		roll += avstep*dt;
		
		q = Euler2Qt(roll,pitch,yaw);
		
	}
}
