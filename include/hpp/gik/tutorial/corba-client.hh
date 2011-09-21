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

/// Connection to a servant in the <RobotViewer.context> context.
class CorbaClient
{
public:
  //Default setup for omniorb nameservice
  static const int DEFAULT_OMNIORB_NAMESERVICE_PORT = 2809;
  static const std::string DEFAULT_OMNIORB_NAMESERVICE_HOST;
  
protected:
  std::string progname_;
  CORBA::ORB_ptr m_orb;
  PortableServer::POA_var poa;
  CosNaming::NamingContext_var rootContext;
  CosNaming::Name name;

public:
  /// \brief Main constructor.
  ///
  /// The argument specifies the current program name (i.e. argv[0]).
  /// It is used for human-display only and can be initialized to any string.
  CorbaClient  (const std::string& progname)
    : progname_ (progname)
  {}

  /// \brief Connect to CORBA service.
  ///
  /// Connects to CORBA service.
  ///
  /// This supports two differents mode:
  /// - either no host/port is given to resolve the name service
  ///   and it is done using the local default omniORB configuration
  ///   (usually through the omniORB.cfg file).
  /// - or if a host or port is given, the default configuration is overrided
  ///   and the specified server hosts/ports are used.
  ///   Default values are provided when specifying only one value
  ///  (either port or host).
  ///   * Host: localhost
  ///   * Port: 2809
  ///
  /// See in the omniORB manual
  /// (http://omniorb.sourceforge.net/omni41/omniORB/omniORB004.html#toc22)
  /// the InitRef setting to configure the location of the NameService.
  CORBA::Object_ptr connectToServant
  (const std::string& m_HostNameServer = DEFAULT_OMNIORB_NAMESERVICE_HOST,
   int nameServicePort = DEFAULT_OMNIORB_NAMESERVICE_PORT);

  CORBA::Object_ptr reconnectToServant( void ) { return resolve(); }
  
protected:
  CORBA::Object_ptr resolve( void );

};  

#endif
