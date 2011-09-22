HPP-GIK Tutorial
================

This package contains a small tutorial for hpp-gik. It relies on hpp-gik
for motion planning and robot-viewer for visualization. The robot
configurations generated by hpp-gik are sent to the robot-viewer server
through Corba.

Prerequites
-----------

We recommend to install the software on a Linux computer running Ubuntu 10.04.

Installing dependencies
-----------------------

To install dependencies, use LAAS package management system robotpkg
   git clone http://softs.laas.fr/git/robots/robotpkg.git
   cd robotpkg
   ./bootstrap/bootstrap --prefix=$PREFIX
where PREFIX is /usr/local by default.

Complete the configuration file of robotpkg
    cd ../hpp-gik-tutorial
    cat robotpkg.conf.append >> $PREFIX/etc/robotpkg.conf

Install hpp-gik and dependencies
    cd ../robotpkg/path/hpp-gik
    make install

Install robotviewer
    cd ../../..
    git clone --recursive git://github.com/laas/robot-viewer.git
    cd robot-viewer
    git checkout -b romeo 84dbb30140c41858042781444f50c4b8040eb578
    git submodule update
    python setup.py install --prefix $PREFIX

Setup
-----

To compile this package, it is recommended to create a separate build
directory:
    cd hpp-gik-tutorial
    mkdir _build
    cd _build
    cmake [OPTIONS] ..
    make install

Running the package
-------------------

Before running the tutorial script, make sure you have a Corba naming service
running
    ps -ef | grep omni
If not run
    omniNames -start 2809

Then launch robot-viewer, passing as argument the
romeo configuration file distributed with this package:

    robotviewer -c $PREFIX/share/hpp/gik/tutorial/config.romeo

While parsing the robot files, the robot-viewer server outputs some
log messages, such as:

    robotviewer.kinematic_server:INFO:parsed_config <robotviewer.kinematic_server.CustomConfigParser instance at 0x93d12cc>
    robotviewer.display_server:INFO:Creating window
    OpenGL.GLUT.special:INFO:Cleaning up context data for window 1: True
    use_shader= True
    robotviewer.kinematic_server:INFO:parsed_config <robotviewer.kinematic_server.CustomConfigParser instance at 0xe3580ec>
    robotviewer.display_server:ERROR:romeo Element with that name exists already
    None

Once the parsing is finished and the server is up, you should see a 3D view of
Romeo in an empty environment, launch the hpp-gik-tutorial executable.

      hpp-gik-tutorial

This will start the planning phase. hpp-gik outputs some log messages. It is
normal that during whole-body motion execution, the trajectory of the ZMP
slightly differs from the planned trajectory. You might observe messages
such as:

     Eccentered ZMP: 0.0418092 , 0.0580008 at time 2.305

When the planning is finished, the Romeo model will start moving. If you want
to change the stack of tasks solved by the robot, edit the file:

     src/application.cc

The tasks are defined in function `createTask()`. For more
information on how to create and solve tasks, have a look at
the `hpp-gik` package documentation (installed in $PREFIX/share/doc/hpp-gik/doxygen-html/index.html).
