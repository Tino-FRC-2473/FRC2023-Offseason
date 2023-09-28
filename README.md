# FSMBotTemplate

Finite-state-machine based template project for WPILib based robot code.

To provide a more structured framework for FIRST Robotics Competition robot development, this project defines subsystem behaviors strictly in terms of multiple separate finte state machines updated in a round-robin fashion. This will make scheduling behavior explicitly visible instead of hidden behind the command scheduler, and avoid ambiguous shared state between command and subsystems under the WPILib command based programming model.

## FSMSystem
The primary base class for robot systems. Each robot system is defined in terms of a Mealy-style finte state machine with control over a well-defined set of robot hardware. 

## TeleopInput
Utility class with ownership of teleop input handling. The single global instance of this class mediates access to inputs during the teleoperated mode and abstracts control mappings.

## MAXSwerveModule
Code provided by rev swerve template: https://github.com/REVrobotics/MAXSwerve-Java-Template

Copyright (c) 2009-2021 FIRST and other WPILib contributors
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:
   * Redistributions of source code must retain the above copyright
     notice, this list of conditions and the following disclaimer.
   * Redistributions in binary form must reproduce the above copyright
     notice, this list of conditions and the following disclaimer in the
     documentation and/or other materials provided with the distribution.
   * Neither the name of FIRST, WPILib, nor the names of other WPILib
     contributors may be used to endorse or promote products derived from
     this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY FIRST AND OTHER WPILIB CONTRIBUTORS "AS IS" AND
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY NONINFRINGEMENT AND FITNESS FOR A PARTICULAR
PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL FIRST OR CONTRIBUTORS BE LIABLE FOR
ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
