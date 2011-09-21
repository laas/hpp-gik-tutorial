# Copyright 2011, Sebastien Dalibard, CNRS
#
# This file is part of hpp-gik-tutorial.
# hpp-gik-tutorial is free software: you can redistribute it and/or
# modify it under the terms of the GNU Lesser General Public License
# as published by the Free Software Foundation, either version 3 of
# the License, or (at your option) any later version.
#
# hpp-gik-tutorial is distributed in the hope that it will be useful, but
# WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
# General Lesser Public License for more details.  You should have
# received a copy of the GNU Lesser General Public License along with
# hpp-gik-tutorial. If not, see <http://www.gnu.org/licenses/>.

include(../cmake/idl.cmake)

INCLUDE_DIRECTORIES("${Boost_INCLUDE_DIRS}")
INCLUDE_DIRECTORIES("${CMAKE_CURRENT_SOURCE_DIR}")
INCLUDE_DIRECTORIES("${CMAKE_CURRENT_BINARY_DIR}")

GENERATE_IDL_FILE(RobotViewer
  "${ROBOT_VIEWER_DATAROOTDIR}/idl/robot-viewer")

ADD_EXECUTABLE(hpp-gik-tutorial
  ${PROJECT_NAME_HEADERS}
  application.hh
  application.cc
  robot-builder.cc 
  hpp-gik-tutorial.cc
  corba-client.cc
  ${ALL_IDL_STUBS}
  )
ADD_DEPENDENCIES(hpp-gik-tutorial nao.cc)

PKG_CONFIG_USE_DEPENDENCY(hpp-gik-tutorial hpp-gik)
PKG_CONFIG_USE_DEPENDENCY(hpp-gik-tutorial jrl-dynamics)
PKG_CONFIG_USE_DEPENDENCY(hpp-gik-tutorial omniORB4)

INSTALL(TARGETS hpp-gik-tutorial DESTINATION bin)