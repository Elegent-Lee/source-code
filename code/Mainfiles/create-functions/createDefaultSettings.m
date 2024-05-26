function [ settings ] = createDefaultSettings(  )
% Function to produce a default setting for execution


%% Species settings
settings.NSpecies = 1;
settings.NAgents = [100];


%% Agent settings

settings.mass = [1]; %mass of each agent
settings.HeightAgents = 2; %Height of agents represented as triangles

load('h-100.mat');load('Xi-100.mat');
settings.h=h; %Coefficient of communication barrier between agent and leader
settings.Xi=Xi;%Coefficient of Agents' self-communication barriers

settings.PosMeanAgentsIni = [[-100,-100]]; 
settings.PosVarAgentsIni = [[100,100]];
settings.VelIntervAgentsIni = [[0,0.01]];

settings.d = [5]; %agent safe distance
settings.r = [7]; %perception radius


settings.Gamma_c1 = [0.01,0.01,0.01]; %c1 
settings.Gamma_c2 = [0.6,0.6,0.6]; %c2 
settings.Gamma_c3 = [0.25,0.25,0.25]; % c3 for Coefficient of local feedback mechanism
settings.Gamma_c4 = [0,0,0]; 
settings.LFM = 1;% 0 is without local feedback mechanism; 1 is with local feedback mechanism


% settings.randomBool = 0; %

settings.agentPositionStyle = 'random'; %options: 'random','filename','fixed_circle'
settings.agentPositionFilename = ''; % full filename to file containing at least the variable agents
settings.agentPositionFixedDist = [30,30,30];


%% Gamma agent settings
settings.GammaTraj = 'line'; %Options: point, line, circle
settings.qd = [0,0]; %initial position for fixed point and line. Center for circle
settings.pd = [0.5,0.5]; %initial velocity

%Circular Traj
% settings.GammaRad = 20; %Radius of Gamma agent circular trajectory
% settings.GammaPhase = 0; %Initial phase angle for starting point for cicular traj


%% Obstacles Settings
%Only circular obstacles included for now
% settings.ObstacleCenter = [0,0;30,3;80,0];

%% Graph settings
settings.AxisMode = 'fixed'; %Options: 'auto', 'fixed'
settings.axis = [-80,80,-80,80];

%% Timer Settings                                    
settings.dtPlot = 0.5; %[s] frame shown in these time steps
settings.period = 0.001; %Time between calls for timerfunc. Min is 0.001
settings.iteration = 500; %Number of iterations for timerfunc


%% Simulation Settings
settings.SimMode = 'algorithm2'; %Options:  algorithm for this paper 



end