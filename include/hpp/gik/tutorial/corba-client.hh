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

#ifndef CORBA_CLIENT_HH
#define CORBA_CLIENT_HH

# include <string>
# include <omniORB4/CORBA.h>


class CorbaClient
{
public:
  static const int DEFAULT_OMNIORB_NAMESERVICE_PORT = 2809;
  static const std::string DEFAULT_OMNIORB_NAMESERVICE_HOST;
  
protected:
  std::string progname_;
  CORBA::ORB_ptr m_orb;
  PortableServer::POA_var poa;
  CosNaming::NamingContext_var rootContext;
  CosNaming::Name name;

public:
  CorbaClient  (const std::string& progname)
    : progname_ (progname)
  {}

  CORBA::Object_ptr connectToServant
  (const std::string& m_HostNameServer = DEFAULT_OMNIORB_NAMESERVICE_HOST,
   int nameServicePort = DEFAULT_OMNIORB_NAMESERVICE_PORT);

  CORBA::Object_ptr reconnectToServant( void ) { return resolve(); }
  
protected:
  CORBA::Object_ptr resolve( void );

};  


#endif
