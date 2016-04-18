#include "ElementRobot.h"

ElementRobot::ElementRobot() : ElementMobile()
{

}

ElementRobot::ElementRobot(std::string name_, std::unique_ptr<se3> pose_) : ElementMobile(name_, std::move(pose_) )
{

}

ElementRobot::ElementRobot(std::string name_, std::unique_ptr <se3> pose_, const Mat<float>& hwd_) : ElementMobile(name_,std::move(pose_),hwd_)
{

}

ElementRobot::ElementRobot(std::string name_, se3* pose_, const Mat<float>& hwd_)  : ElementMobile(name_,pose_,hwd_)
{

}

ElementRobot::~ElementRobot()
{

}

bool ElementRobot::isFixe()
{
  return false;
}

bool ElementRobot::isComposit()
{
 	return false;
}

bool ElementRobot::isRobot()
{
 	return true;
}

