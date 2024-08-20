function launchController
    project = simulinkproject;
    projectRoot = project.RootFolder;
    open_system(fullfile(projectRoot,'IO Layer','ControlSubsystem_with_IO.slx'));
end
