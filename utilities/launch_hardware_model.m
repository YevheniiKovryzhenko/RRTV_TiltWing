function launch_hardware_model
    project = simulinkproject;
    projectRoot = project.RootFolder;
    open_system(fullfile(projectRoot,'IO Layer','hardware_model.slx'));
end
