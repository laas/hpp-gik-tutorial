// Copyright (C) 2011, Sebastien Dalibard, CNRS
//
// This file is part of hpp-gik-tutorial.
// hpp-gik-tutorial is free software: you can redistribute it and/or
// modify it under the terms of the GNU Lesser General Public License
// as published by the Free Software Foundation, either version 3 of
// the License, or (at your option) any later version.
//
// hpp-gik-tutorial is distributed in the hope that it will be useful, but
// WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
// General Lesser Public License for more details.  You should have
// received a copy of the GNU Lesser General Public License along with
// hpp-gik-tutorial. If not, see <http://www.gnu.org/licenses/>.

# include <hpp/gik/tutorial/robot-builder.hh>
# include <jrl/dynamics/robotdynamicsimpl.hh>

# include  "romeo-joints.hh"


namespace hpp
{
  namespace gik
  {
    namespace tutorial
    {
      Robotbuilder::Robotbuilder()
      {
      }
 
      Robotbuilder::~Robotbuilder()
      {
      }

      CjrlHumanoidDynamicRobot *
      Robotbuilder::makeRobot()
      {
	CimplObjectFactory objFactory;
	robot_ = objFactory.createHumanoidDynamicRobot();

	#include "romeo.cc"
	
	setActuatedJoints();

	vectorN halfSittingCfg(ROMEO_NB_DOF,0.);

	halfSittingCfg(Trunk_TZ) =1.083992009;

	halfSittingCfg(RShoulderPitch) = M_PI/2;
	halfSittingCfg(RShoulderYaw) = -M_PI/12;
	halfSittingCfg(RElbowYaw) = M_PI/12;
	halfSittingCfg(RWristRoll) = M_PI/2;

	halfSittingCfg(LShoulderPitch) = M_PI/2;
	halfSittingCfg(LShoulderYaw) = M_PI/12;
	halfSittingCfg(LElbowYaw) = -M_PI/12;
	halfSittingCfg(LWristRoll) = -M_PI/2;

	halfSittingCfg(LHipPitch) = -0.3;
	halfSittingCfg(LKneePitch) = 0.6;
	halfSittingCfg(LAnklePitch) = -0.3;

	halfSittingCfg(RHipPitch) = -0.3;
	halfSittingCfg(RKneePitch) = 0.6;
	halfSittingCfg(RAnklePitch) = -0.3;
	
	robot_->currentConfiguration(halfSittingCfg);

	vectorN zeros(ROMEO_NB_DOF,0.);
	robot_->currentVelocity(zeros);
	robot_->currentAcceleration(zeros);

	return robot_;
      }

      void
      Robotbuilder::setActuatedJoints()
      {
	std::vector<CjrlJoint*> actuatedJoints;
	std::map<const std::string, CjrlJoint*>::iterator it;
	for(it=jointMap_.begin() ; it != jointMap_.end(); it++ ) {
	  if ( (*it).second != robot_->rootJoint() )
	    actuatedJoints.push_back( (*it).second);
	}
	robot_->setActuatedJoints(actuatedJoints);
      }

      matrix4d
      Robotbuilder::buildMatrix4d(double x0,double x1,double x2,double x3,
				  double x4,double x5,double x6,double x7,
				  double x8,double x9,double x10,double x11,
				  double x12,double x13,double x14,double x15)
      {
	matrix4d ret;
	ret[0]=x0; ret[1]=x1; ret[2]=x2; ret[3]=x3;	
	ret[4]=x4; ret[5]=x5; ret[6]=x6; ret[7]=x7;
	ret[8]=x8; ret[9]=x9; ret[10]=x10; ret[11]=x11;
	ret[12]=x12; ret[13]=x13; ret[14]=x14; ret[15]=x15;
	return ret;
      }

      void
      Robotbuilder::createJoint(const std::string& inName,JointType type,const matrix4d& inInitialPos)
      {
	CimplObjectFactory objFactory;
	CjrlJoint* jrlJoint;
	switch (type) {
	case FREEFLYER:
	  jrlJoint = objFactory.createJointFreeflyer(inInitialPos);
	  break;
	case ROTATION:
	  jrlJoint = objFactory.createJointRotation(inInitialPos);
	  break;
	case TRANSLATION:
	  jrlJoint = objFactory.createJointTranslation(inInitialPos);
	  break;
	case ANCHOR:
	  jrlJoint = objFactory.createJointAnchor(inInitialPos);
	}
	CjrlBody* jrlBody = objFactory.createBody();
	jointMap_[inName] = jrlJoint;
	bodyMap_[inName] = jrlBody;
	jrlJoint->setLinkedBody(*jrlBody);
      }

      void
      Robotbuilder::setRootJoint(const std::string& inName)
      {
	robot_->rootJoint(*(jointMap_[inName]));
      }

      void
      Robotbuilder::addChildJoint(const std::string& inParentName, const std::string& inChildName)
      {
	jointMap_[inParentName]->addChildJoint(*(jointMap_[inChildName]));
      }

      void
      Robotbuilder::setDofBounds(const std::string& inName,unsigned int inDof, double minValue, double maxValue)
      {
	CjrlJoint * joint = jointMap_[inName];
	joint->lowerBound(inDof, minValue);
	joint->upperBound(inDof, maxValue);
      }

      void
      Robotbuilder::setMass(const std::string& inName, double mass)
      {
	bodyMap_[inName]->mass(mass);
      }
      
      void
      Robotbuilder::setLocalCenterOfMass(const std::string& inName, const vector3d& inCoM)
      {
	bodyMap_[inName]->localCenterOfMass(inCoM);
      }

      void
      Robotbuilder::setInertiaMatrix(const std::string& inName, const matrix3d& inInertiaMatrix)
      {
	bodyMap_[inName]->inertiaMatrix(inInertiaMatrix);
      }

      void
      Robotbuilder::setWaist(const std::string& inName)
      {
	robot_->waist(jointMap_[inName]);
      }

      void
      Robotbuilder::setChest(const std::string& inName)
      {
	robot_->chest(jointMap_[inName]);
      }

      void
      Robotbuilder::setLeftWrist(const std::string& inName)
      {
	robot_->leftWrist(jointMap_[inName]);
      }

      void
      Robotbuilder::setRightWrist(const std::string& inName)
      {
	robot_->rightWrist(jointMap_[inName]);
      }

      void
      Robotbuilder::setLeftAnkle(const std::string& inName)
      {
	robot_->leftAnkle(jointMap_[inName]);
      }

      void
      Robotbuilder::setRightAnkle(const std::string& inName)
      {
	robot_->rightAnkle(jointMap_[inName]);
      }

      void
      Robotbuilder::setGazeJoint(const std::string& inName)
      {
	robot_->gazeJoint(jointMap_[inName]);
      }

      CjrlHand*
      Robotbuilder::createHand(CjrlJoint* wrist,
			       const vector3d& handCenter,
			       const vector3d& thumbAxis,
			       const vector3d& forefingerAxis,
			       const vector3d& palmNormal)
      {
	CimplObjectFactory objFactory;
	CjrlHand* hand=objFactory.createHand(wrist);
	hand->setAssociatedWrist(wrist);
	hand->setCenter(handCenter);
	hand->setThumbAxis(thumbAxis);
	hand->setForeFingerAxis(forefingerAxis);
	hand->setPalmNormal(palmNormal);
	return hand;
      }
      
      void
      Robotbuilder::setLeftHandParameters(const vector3d& handCenter,
					  const vector3d& thumbAxis,
					  const vector3d& forefingerAxis,
					  const vector3d& palmNormal)
      {
	CjrlHand* hand=createHand(robot_->leftWrist(),handCenter, thumbAxis, forefingerAxis, palmNormal);
	robot_->leftHand(hand);
      }
      
      void
      Robotbuilder::setRightHandParameters(const vector3d& handCenter,
					   const vector3d& thumbAxis,
					   const vector3d& forefingerAxis,
					   const vector3d& palmNormal)
      {
	CjrlHand* hand=createHand(robot_->rightWrist(),handCenter, thumbAxis, forefingerAxis, palmNormal);
	robot_->rightHand(hand);
      }
      
      CjrlFoot*
      Robotbuilder::createFoot(CjrlJoint* ankle,
			       double soleLength,
			       double soleWidth,
			       const vector3d& anklePosition)
      {
	CimplObjectFactory objFactory;
	CjrlFoot* foot = objFactory.createFoot(ankle);
	foot->setAssociatedAnkle(ankle);
	foot->setSoleSize(soleLength,soleWidth);
	foot->setAnklePositionInLocalFrame(anklePosition);
	return foot;
      }

      void
      Robotbuilder::setLeftFootParameters(double soleLength,
					  double soleWidth,
					  const vector3d& anklePosition)
      {
	CjrlFoot* foot = createFoot(robot_->leftAnkle(),soleLength,soleWidth,anklePosition);
	robot_->leftFoot(foot);
      }

      void
      Robotbuilder::setRightFootParameters(double soleLength,
					   double soleWidth,
					   const vector3d& anklePosition)
      {
	CjrlFoot* foot = createFoot(robot_->rightAnkle(),soleLength,soleWidth,anklePosition);
	robot_->rightFoot(foot);
      }

      void
      Robotbuilder::setGazeParameters(const vector3d& gazeOrigin,
				      const vector3d& gazeDirection)
      {
	robot_->gaze(gazeDirection,gazeOrigin);
      }

      void
      Robotbuilder::initializeRobot()
      {
	robot_->initialize();
      }

      void
      Robotbuilder::printBodyCoMs()
      {
	std::map<const std::string, CjrlBody*>::iterator it ;
	for(it=bodyMap_.begin(); it != bodyMap_.end();it++) {
	  vector3d localCom = (*it).second->localCenterOfMass();
	  matrix4d jointT = (*it).second->joint()->currentTransformation();
	  vector3d globCom = jointT*localCom;
	  std::cout << (*it).first << " : " << globCom << std::endl;
	}
      }

    }
  }
}
