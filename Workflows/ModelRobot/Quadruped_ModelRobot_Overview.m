%% Quadruped Robot Modeling with Simscape(TM)
%
% This repository contains a Simscape model of a quadruped robot able to
% walk on flat and inclined terrain, with a environment that presents
% different obstacles, and the posibility of open-loop or interactive
% control system.
%
% *Main files*
%  
% # Complete Robot Model: <matlab:cd(fullfile(prjPath));open_system('complete_robot') Model>
% # Point cloud generation file: <matlab:cd(fullfile(prjPath));edit('Parameters_pntcld.m') Script>
% 
% *Get up to speed*
%
% - Tutorial 1: <matlab:cd(fullfile(prjPath,'Tutorials\01_Create_gait_depending_Bezier_curve'));open_system('t01_one_leg') Create gait>
%
% The focus of this tutorial is set on implementing the gait function for 
% one leg, that it moves realistically and is able to perform a step forward.
% 
% - Tutorial 2: <matlab:cd(fullfile(prjPath,'Tutorials\02_Create_sideways_gait'));open_system('t02_one_leg_sideways_movement') Create sideways gait>
%
% The focus of this tutorial is set on implementing an additional sideways 
% joint in a leg, that it is able to perform a lateral step which is later 
% needed for turning and walking sideways.
%
% - Tutorial 3: <matlab:cd(fullfile(prjPath,'Tutorials\03_Put_legs_together_to_torso_and_walk'));open_system('t03_cobine_legs_to_torso') Combine legs and torso>
%
% This tutorial consists of building the first state of a walking four 
% legged robot, based on the leg elements and the gait functions from 
% tutorial 1 and 2.
%
% - Tutorial 4: <matlab:cd(fullfile(prjPath,'Tutorials\04_Combine_Gaits_with_if_statements'));open_system('t04_combine_gaits') Combine different gaits>
%
% In this tutorial several gaits need to be combined that the robot is able 
% to perform different movements, depending on the control value which acts 
% as an input.
%
% - Tutorial 5: <matlab:cd(fullfile(prjPath,'Tutorials\05_Write_A_Code_that_balances_out'));open_system('t05_balancing') Balancing>
%
% In this tutorial a balancing function has to be implemented to make the
% center of gravity of the robot remain in the center of the support feet
% so the robot is stable even in inclined terrain.
%
% - Tutorial 6: <matlab:cd(fullfile(prjPath,'Tutorials\06_Manual_Control'));open_system('t06_manual_control') Manual control>
%
% This tutorial addresses how to connect an external controller (e.g. from 
% an Xbox) and use it to manually guide the robot.
