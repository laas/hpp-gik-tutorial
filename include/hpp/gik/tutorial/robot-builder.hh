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

#ifndef HPP_GIK_TUTORIAL_ROBOTBUILDER_HH
#define HPP_GIK_TUTORIAL_ROBOTBUILDER_HH

# include <string>
# include <map>

# include <jrl/mal/matrixabstractlayer.hh>
# include <abstract-robot-dynamics/humanoid-dynamic-robot.hh>

namespace hpp
{
  namespace gik
  {
    namespace tutorial
    {
      enum JointType {FREEFLYER,ROTATION,TRANSLATION,ANCHOR};
      /**
	 \brief Builds a jrl-dynamics model of the Romeo humanoid robot.
      */
      class Robotbuilder
      {
      public:
	 /**
	 \brief Constructor
	 */
	Robotbuilder ();
	/**
	 \brief Destructor
	 */
	~Robotbuilder ();

	/**
	 \brief Builds the model and returns a pointer to a newly allocated
	 jrl humanoid dynamic robot
	 */
	CjrlHumanoidDynamicRobot * 
	makeRobot();

      private:
	void setActuatedJoints();

	matrix4d buildMatrix4d(double x0,double x1,double x2,double x3,
			       double x4,double x5,double x6,double x7,
			       double x8,double x9,double x10,double x11,
			       double x12,double x13,double x14,double x15);

	void createJoint(const std::string& inName,JointType type,const matrix4d& inInitialPos);
	void setRootJoint(const std::string& inName);
	void addChildJoint(const std::string& inParentName, const std::string& inChildName);
	void setDofBounds(const std::string& inName,unsigned int inDof, double minValue, double maxValue);
	void setMass(const std::string& inName, double mass);
	void setLocalCenterOfMass(const std::string& inName, const vector3d& inCoM);
	void setInertiaMatrix(const std::string& inName, const matrix3d& inInertiaMatrix);

	void setWaist(const std::string& inName);
	void setChest(const std::string& inName);
	void setLeftWrist(const std::string& inName);
	void setRightWrist(const std::string& inName);
	void setLeftAnkle(const std::string& inName);
	void setRightAnkle(const std::string& inName);
	void setGazeJoint(const std::string& inName);

	CjrlHand* createHand(CjrlJoint* wrist,
			     const vector3d& handCenter,
			     const vector3d& thumbAxis,
			     const vector3d& forefingerAxis,
			     const vector3d& palmNormal);

	void setLeftHandParameters(const vector3d& handCenter,
				   const vector3d& thumbAxis,
				   const vector3d& forefingerAxis,
				   const vector3d& palmNormal);

	void setRightHandParameters(const vector3d& handCenter,
				    const vector3d& thumbAxis,
				    const vector3d& forefingerAxis,
				    const vector3d& palmNormal);

	CjrlFoot* createFoot(CjrlJoint* ankle,
			     double soleLength,
			     double soleWidth,
			     const vector3d& anklePosition);

	void setLeftFootParameters(double soleLength,
				   double soleWidth,
				   const vector3d& anklePosition);

	void setRightFootParameters(double soleLength,
				    double soleWidth,
				    const vector3d& anklePosition);

	void setGazeParameters(const vector3d& gazeOrigin,
			       const vector3d& gazeDirection);

	void initializeRobot();

	std::map<const std::string, CjrlJoint*> jointMap_;
	std::map<const std::string, CjrlBody*> bodyMap_;

	CjrlHumanoidDynamicRobot * robot_;
      };
    }
  }
}





#endif
