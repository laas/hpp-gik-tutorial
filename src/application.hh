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

#ifndef HPP_GIK_TUTORIAL_APPLI_HH
#define HPP_GIK_TUTORIAL_APPLI_HH

# include <string>

# include <jrl/mal/matrixabstractlayer.hh>
# include <hpp/gik/robot/standing-robot.hh>
# include <hpp/gik/task/generic-task.hh>

# include <hpp/gik/tutorial/corba-client.hh>

# include "RobotViewer.hh"

namespace hpp
{
  namespace gik
  {
    namespace tutorial
    {
      class Application {
      public: 
	Application(int argc, char* argv[]);
	~Application();

	void process();

      protected:
	void createRobot();
	void createTask();
	void solveTask();
	void outputMotion();


      private:
	double samplingPeriod_;
	ChppGikStandingRobot * robot_;
	ChppGikGenericTask * genericTask_;
	CorbaClient  corba_;
	
	robotviewer_corba::RobotViewer_var  serverPtr_;
	std::string elementName_;
      };
    }
  }
}

#endif
