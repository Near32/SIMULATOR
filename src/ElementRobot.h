#ifndef ELEMENTROBOT_H
#define ELEMENTROBOT_H

#include"ElementMobile.h"

class ElementRobot : public ElementMobile
{
	public:
		ElementRobot();
		ElementRobot(std::string name_, std::unique_ptr<se3> pose_);
		ElementRobot(std::string name_, std::unique_ptr<se3> pose_, const Mat<float>& hwd_);
		ElementRobot(std::string name_, se3* pose_, const Mat<float>& hwd_);
		
		~ElementRobot();
		
		virtual bool isFixe() override;
		
		virtual bool isComposit()	override;
};


#endif
