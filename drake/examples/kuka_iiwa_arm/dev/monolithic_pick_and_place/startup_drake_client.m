addpath((fullfile(getenv('DRAKE'), 'drake/matlab/util')));
assert(~isempty(which('call_matlab_client')));

fprintf('Use ''call_matlab_client'' to gather data\n');
