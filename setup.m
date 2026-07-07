function setup
% Add project folders to the MATLAB path.
    root = fileparts(mfilename('fullpath'));

    folders = { ...
        fullfile(root, 'plant'), ...
        fullfile(root, 'plots')};
    
    addpath(folders{:});
end
