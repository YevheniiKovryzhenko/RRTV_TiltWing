function launchGCS
    project = simulinkproject;
    projectRoot = project.RootFolder;
    open_system(fullfile(projectRoot,'GCS','GCS.slx'));
end
