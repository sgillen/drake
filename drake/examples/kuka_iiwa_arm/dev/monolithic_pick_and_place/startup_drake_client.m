addpath((fullfile(getenv('DRAKE'), 'drake/matlab/util')));
assert(~isempty(which('call_matlab_client')));

%%
do_reset = true;
if do_reset
    system(['test -f /tmp/matlab_rpc && rm /tmp/matlab_rpc ', ...
        '&& mkfifo /tmp/matlab_rpc']);
    fprintf('Reset FIFO buffer\n');
end

%%
clear call_matlab_client;
call_matlab_client;
