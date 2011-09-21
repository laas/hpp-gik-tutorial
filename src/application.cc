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

# include <stdexcept>
# include <iostream>
# include <vector>
# include <unistd.h>

# include <boost/date_time/posix_time/posix_time.hpp>

# include "application.hh"
# include <hpp/gik/tutorial/robot-builder.hh>

# include <hpp/gik/motionplanner/element/step-element.hh>
# include <hpp/gik/constraint/parallel-constraint.hh>
# include <hpp/gik/constraint/plane-constraint.hh>
# include <hpp/gik/constraint/position-constraint.hh>
# include <hpp/gik/motionplanner/element/interpolated-element.hh>

namespace hpp
{
  namespace gik
  {
    namespace tutorial
    {
      Application::Application(int argc, char* argv[]):
	samplingPeriod_(0.005),
	robot_(NULL),
	genericTask_(NULL),
	corba_(argv[0]),
	serverPtr_()
      {
	CORBA::Object_ptr corba_obj =
	  corba_.connectToServant ();

	 try
	   {
	     serverPtr_ = robotviewer_corba::RobotViewer::_narrow(corba_obj);
	     if (CORBA::is_nil (serverPtr_))
	       throw std::runtime_error ("failed to connect to the server.");
	   }
	 catch (CORBA::TRANSIENT& exception)
	   {
	     std::cerr << "Failed to connect to robotviewer." << std::endl;
	     throw;
	   }
      }
      
      Application::~Application()
      {
	if(genericTask_)
	  delete genericTask_;
	
	if(robot_)
	  delete robot_;
      }

      void
      Application::process()
      {
	std::cout << "Creating robot." << std::endl;
	createRobot();
	std::cout << "Creating task." << std::endl;
	createTask();
	std::cout << "Solving task." << std::endl;
	solveTask();
	std::cout << "Displaying motion." << std::endl;
	outputMotion();
      }

      void 
      Application::createRobot()
      {
	Robotbuilder robotbuilder;
	CjrlHumanoidDynamicRobot * jrlRobot = robotbuilder.makeRobot();

	/* Initialize jrl dynamics properties */
	std::string property,value;
	property="ComputeZMP"; value="true";jrlRobot->setProperty ( property,value );
	property="TimeStep"; value="0.005";jrlRobot->setProperty ( property,value );
	property="ComputeAccelerationCoM"; value="true";jrlRobot->setProperty ( property,value );
	property="ComputeBackwardDynamics"; value="false";jrlRobot->setProperty ( property,value );
	property="ComputeMomentum"; value="true";jrlRobot->setProperty ( property,value );
	property="ComputeAcceleration"; value="true";jrlRobot->setProperty ( property,value );
	property="ComputeVelocity"; value="true";jrlRobot->setProperty ( property,value );
	property="ComputeSkewCom"; value="false";jrlRobot->setProperty ( property,value );
	property="ComputeCoM"; value="true";jrlRobot->setProperty ( property,value );

	jrlRobot->computeForwardKinematics();

	robot_ = new ChppGikStandingRobot(*jrlRobot);
	robot_->staticHalfsitting();
      }

      void 
      Application::createTask()
      {
	if (!robot_) {
	  throw std::runtime_error ("GikStandingRobot object is not initialized");
	}
	genericTask_ = new ChppGikGenericTask(robot_,samplingPeriod_);
	
	double startTime = 0;
	/* Time before starting locomotion */
	double time = 1.6; 

	/* Step parameters */
	double finalZMPCoeff = 0.5;
	double endShiftTime = 0.1;
	double startShiftTime = 0.1;
	double footMotionDuration = 0.8;
	double stepHeight = 0.05;

	/* Generate two steps */
	ChppGikFootprint * rightFP = new ChppGikFootprint(0.2,-0.096,0);
	ChppGikFootprint * leftFP = new ChppGikFootprint(0.2,0.096,0);

	ChppGikStepElement* firstStepElement =
	  new ChppGikStepElement(robot_,
				 time,
				 rightFP,
				 true,
				 samplingPeriod_,
				 finalZMPCoeff,
				 endShiftTime,
				 startShiftTime,
				 footMotionDuration,
				 stepHeight);

	time += firstStepElement->duration();
	
	ChppGikStepElement* secondStepElement =
	  new ChppGikStepElement(robot_,
				 time,
				 leftFP,
				 false,
				 samplingPeriod_,
				 finalZMPCoeff,
				 endShiftTime,
				 startShiftTime,
				 footMotionDuration,
				 stepHeight);
	time += secondStepElement->duration();

	/* Extra time after finishing locomotion */
	time += 2.;
	std::cout << "Total time: " << time << std::endl;

	genericTask_->addElement(firstStepElement);
        genericTask_->addElement(secondStepElement);

	/* Constraint on the waist height */
	ChppGikPlaneConstraint * waistPlaneConstraint = 
	  new ChppGikPlaneConstraint( *(robot_->robot()),
				      *(robot_->robot()->waist()),
				      vector3d ( 0,0,0 ),
				      vector3d ( 0, 0,robot_->halfsittingWaistHeight()),
				      vector3d (0,0,1) ) ;
	ChppGikInterpolatedElement * heightElem = 
	  new  ChppGikInterpolatedElement( robot_->robot(),
					   waistPlaneConstraint,
					   1,
					   startTime,
					   time,
					   samplingPeriod_);
	genericTask_->addElement(heightElem);

	/* Constraint on the waist orientation */
	ChppGikParallelConstraint * waistParallelConstraint =
	  new ChppGikParallelConstraint( *(robot_->robot()),
					 *(robot_->robot()->waist()),
					 vector3d (1,0,0),
					 vector3d (0,0,1)) ;
	ChppGikInterpolatedElement * orientationElem =
	  new  ChppGikInterpolatedElement( robot_->robot(),
					   waistParallelConstraint,
					   2,
					   startTime,
					   time,
					   samplingPeriod_);
	genericTask_->addElement(orientationElem);

	/* Hand reaching task */
	ChppGikPositionConstraint * handPosConstraint =
	  new ChppGikPositionConstraint( *(robot_->robot()),
					 *(robot_->robot()->rightWrist()),
					 vector3d ( 0,0,0 ),
					 vector3d ( 0.5,-0.15,1.0) );
	ChppGikInterpolatedElement * positionElem = 
	  new  ChppGikInterpolatedElement( robot_->robot(),
					   handPosConstraint,
					   3,
					   time-2.5,
					   2.5,
					   samplingPeriod_,
					   0.2);
	
	genericTask_->addElement(positionElem);

	genericTask_->showResolutionTime(true);
 
      }

      void
      Application::solveTask()
      {
	if (!genericTask_) {
	  throw std::runtime_error ("GikGenericTask object is not initialized");
	}
	bool solved = genericTask_->solve();
	if (!solved) {
	  std::cout << "Failed to solve gik task" << std::endl;
	}
      }

      void
      Application::outputMotion()
      {
	if (!genericTask_) {
	  throw std::runtime_error ("GikGenericTask object is not initialized");
	}

	ChppRobotMotion solutionMotion = genericTask_->solutionMotion();
	if(solutionMotion.empty()) {
	  std::cout << "Solution motion is empty" << std::endl;
	  return ;
	}

	boost::posix_time::ptime start, end;
	

	const ChppRobotMotionSample * motionSample = solutionMotion.firstSample();
	while (motionSample) {

	  start = boost::posix_time::microsec_clock::local_time();

	  vectorN config = motionSample->configuration;
	  motionSample = solutionMotion.nextSample();
	  robotviewer_corba::DoubleSeq outputConfig;
	  outputConfig.length(config.size());
	  for(unsigned int i=0;i<config.size();i++) {
	    outputConfig[i] = config(i);
	  }
	  elementName_ = std::string("romeo");
	  serverPtr_->updateElementConfig(elementName_.c_str(), outputConfig);

	  end = boost::posix_time::microsec_clock::local_time();
	  boost::posix_time::time_duration duration = end - start;
	  long uDuration = duration.total_microseconds();
	  long timeToSleep = (samplingPeriod_*1e6) - uDuration;
	  if (timeToSleep > 0) 
	    usleep((long) (samplingPeriod_*1e6) - uDuration);
	}
      }

    }
  }
}
