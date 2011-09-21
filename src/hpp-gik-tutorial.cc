// Copyright (C) 2011 by Sebastien Dalibard.
//
// This file is part of the hpp-gik-tutorial.
//
// hpp-gik-tutorial is free software: you can redistribute it and/or modify
// it under the terms of the GNU Lesser General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// hpp-gik-tutorial is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU Lesser General Public License for more details.
//
// You should have received a copy of the GNU Lesser General Public License
// along with hpp-gik-tutorial.  If not, see <http://www.gnu.org/licenses/>.


# include <iostream>
# include <boost/format.hpp>
# include <boost/make_shared.hpp>

# include "application.hh"


int main(int argc, char* argv[])
{
  using namespace hpp::gik::tutorial;
  try
    {
      Application app(argc,argv);
      app.process();
    }
  catch(CORBA::Exception& exception)
    {
      boost::format fmt
	("A CORBA exception has been raised (exception name: ``%1%'').");
      fmt % exception._name ();
      std::cerr << fmt.str () << std::endl;
      return 1;
    }
  return 0;
}
