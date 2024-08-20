function set_up_project()
%% Store user created variables for cleanup upon exit
clc
clear
close all
evalin('base', 'userVars = who;');

%% Initialize Project variables 
evalin('base', 'Init_VSDDL_Packages;');

%% Set up work folders
project = simulinkproject;
projectRoot = project.RootFolder;

%% Store project created variables for cleanup upon exit
evalin('base', 'setupAndUserVars = who;');

myCacheFolder = fullfile(projectRoot, 'cache','cache');
myCodeFolder = fullfile(projectRoot,'cache','code');

Simulink.fileGenControl('set',...
                        'CacheFolder', myCacheFolder,...
                        'CodeGenFolder', myCodeFolder,...
                        'createDir', true)
end
