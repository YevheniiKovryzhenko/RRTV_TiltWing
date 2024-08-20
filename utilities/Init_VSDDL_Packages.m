if isfolder("VSDDL_Packages")
    addpath("VSDDL_Packages")
    cd 'VSDDL_Packages'
    addpath("ControlSystemModel")
    addpath("MADCASPCore")
    addpath("PropellerData")
    addpath("Waypoints")
    evalin('base', 'SIMULATION_INIT;');
    cd ../
end