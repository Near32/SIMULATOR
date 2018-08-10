#include "math.h"


//#define debuglvl1
//#define debuglvl2

//------------------------------------
//		Identity Matrix 3x3 :
//------------------------------------
float id3_val_array[9] = {1.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 1.0f};
Mat<float> Identity3( id3_val_array,3,3);

float xaxis_val_array[3] = {1.0f, 0.0f, 0.0f};
Mat<float> XAxis( xaxis_val_array,3,1);

float yaxis_val_array[3] = {0.0f, 1.0f, 0.0f};
Mat<float> YAxis( yaxis_val_array,3,1);

float zaxis_val_array[3] = {0.0f, 0.0f, 1.0f};
Mat<float> ZAxis( zaxis_val_array,3,1);
//------------------------------------
//------------------------------------
float EPSILON = (float)1e-100;
float INF = (float)1e100;

//------------------------------------
//------------------------------------


//------------------------------------
//------------------------------------

se3::se3() : hasChanged(true)	//so that SE3 would be computed on the first access...
{
	try
	{
		t = new Mat<float>((float)0,3,1);
		w = new Mat<float>((float)0,3,1);
		SE3 = new Mat<float>((float)0, 4,4);

		transM_L2W = new Mat<float>(0.0f,4,4);
		transM_W2L = new Mat<float>(0.0f,4,4);
	}
	catch( std::exception& e)
	{
		std::cout << e.what() << std::endl;
		throw e;
	}
}

se3::se3(const Mat<float>& w_, const Mat<float>& t_) : se3()	//so that SE3 would be computed on the first access...
{
	(*t) = t_;
	(*w) = w_;

	computeExp();
}

se3::se3(const Mat<float>& t_) : se3()	//so that SE3 would be computed on the first access...
{
	(*t) = t_;
	computeExp();	
}

se3::se3(const float* w_t_array) : se3()
{
	this->w->set( w_t_array[0], 1,1);
	this->w->set( w_t_array[1], 2,1);
	this->w->set( w_t_array[2], 3,1);
	
	this->t->set( w_t_array[3], 1,1);
	this->t->set( w_t_array[4], 2,1);
	this->t->set( w_t_array[5], 3,1);

	computeExp();
}

	
se3::se3(const se3& x) : se3()
{
	(*t) = x.getT();
	(*w) = x.getW();
	
	computeExp();
}

se3::~se3()
{
	delete t;
	delete w;
	delete SE3;
	delete transM_L2W;
	delete transM_W2L;
}

void se3::computeExp()
{	
	*SE3 = expM( operatorC( *w, *t) );
	this->computeTransformations();
	hasChanged = false;
	
}

Mat<float> se3::exp()					//compute the (R | t) matrix.
{
	if(hasChanged)
	{
#ifdef debuglvl1	
		std::cout << " EXP : w et t : " << std::endl;
		w->afficher();
		t->afficher();
#endif		
		this->computeExp();

		hasChanged = false;
	}
	
	return *SE3;
}

void se3::computeTransformations()
{
	Mat<float> R( extract( *SE3, 1,1, 3,3) );
	Mat<float> tR( transpose(R) );
	
	*transM_L2W = operatorL( 
					operatorC( tR, Mat<float>((float)0,1,3) ), 
					operatorC( *t, Mat<float>((float)1,1,1) ) 
					);

	*transM_W2L = operatorL( 
					operatorC( R, Mat<float>((float)0,1,3) ), 
					operatorC( -1.0f*(R*(*t) ), Mat<float>((float)1,1,1) ) 
					);

}

Mat<float> se3::getTransformationL2W() const
{
	return *transM_L2W;
}


Mat<float> se3::getTransformationW2L() const
{
	return *transM_W2L;
}


void se3::setT(const Mat<float>& t_)
{
	if(t_.getLine() == 3 && t_.getColumn() == 1)
	{
		*t = t_;
		hasChanged = true;
	}
}

void se3::setW(const Mat<float>& w_)
{
	if(w_.getLine() == 3 && w_.getColumn() == 1)
	{
		*(this->w) = w_;
		hasChanged = true;
	}
}

/**/
void se3::setOrientation( const Quat& q)
{
	Mat<float> so3( q.mat() );

	Mat<float> wX( logMEuler(so3));
	w->set( -wX.get(2,3), 1,1);
	w->set( wX.get(1,3), 2,1);
	w->set( -wX.get(1,2), 3,1);
	
	
	hasChanged = true;
}
/**/

/*
void se3::setOrientation( const Quat& q)
{
	float roll,pitch,yaw;
	Qt2Euler(q,&roll,&pitch,&yaw);
	
	Mat<float> wX(3,1);
	w->set( -roll, 1,1);
	w->set( -pitch, 2,1);
	w->set( -yaw, 3,1);
	
#ifdef debuglvl2
	std::cout << " SE3 : TEST APRES MODIF : " << std::endl;
	transpose(*w).afficher();
	transpose( Qt2Mat<float>(q) ).afficher();
#endif	
	
	hasChanged = true;
}
/**/

se3& se3::operator=(const se3& x)
{
	this->~se3();
	
	try
	{
		this->t = new Mat<float>(x.getT());
		this->w = new Mat<float>(x.getW());
		this->SE3 = new Mat<float>(x.getSE3());

		this->transM_W2L = new Mat<float>(x.getTransformationW2L());
		this->transM_L2W = new Mat<float>(x.getTransformationL2W());
	}
	catch(std::exception& e)
	{
		std::cout << e.what() << std::endl;
		throw e;
	}
	
	this->hasChanged = true;
	
	return *this;
}

Mat<float> se3::getT()	const
{	
	return *t;	
}

Mat<float> se3::getW()	const
{	
	return *w;
}

Mat<float> se3::getSE3()	const
{	
	return *SE3;
}

	//---------------------------------------------------------
	//---------------------------------------------------------
	
	

Mat<float> crossproductV( const Mat<float>& p1, const Mat<float>& p2)
{
	return crossProduct(p1)*p2;
}
