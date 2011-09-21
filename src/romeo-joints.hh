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

#ifndef HPP_GIK_TUTORIAL_ROMEO_MODEL_HH
#define HPP_GIK_TUTORIAL_ROMEO_MODEL_HH

enum RomeoJointId {
Trunk_TX = 0,
Trunk_TY,
Trunk_TZ,
Trunk_RX,
Trunk_RY,
Trunk_RZ,
TrunkYaw,
LHipYaw,
LHipRoll,
LHipPitch,
LKneePitch,
LAnklePitch,
LAnkleRoll,
LToePitch,
RHipYaw,
RHipRoll,
RHipPitch,
RKneePitch,
RAnklePitch,
RAnkleRoll,
RToePitch,
RShoulderPitch,
RShoulderYaw,
RElbowRoll,
RElbowYaw,
RWristRoll,
RWristYaw,
RWristPitch,
LShoulderPitch,
LShoulderYaw,
LElbowRoll,
LElbowYaw,
LWristRoll,
LWristYaw,
LWristPitch,
NeckYaw,
NeckPitch,
HeadPitch,
HeadRoll
};

static unsigned int ROMEO_NB_DOF = 39;


#endif
