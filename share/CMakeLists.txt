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

CONFIGURE_FILE (
  "config.romeo.in"
  "config.romeo"
  )

INSTALL(FILES romeo-hpp.kxml ${CMAKE_CURRENT_BINARY_DIR}/config.romeo
  DESTINATION share/hpp/gik/tutorial)

FILE(GLOB vrml "vrml/*.wrl")
INSTALL(FILES ${vrml} 
  DESTINATION share/hpp/gik/tutorial/vrml)
