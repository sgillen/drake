% %%
% addpath((fullfile(getenv('DRAKE'), 'drake/matlab/util')));
% assert(~isempty(which('call_matlab_client')));
% 
% %%
% log = drake.matlab.util.loadLCMLog('/home/eacousineau/proj/tri/proj/dart_impl/_data/sim-logs/okr_resize_rev4.lcmlog');

addpath('/home/eacousineau/devel/matlab_utilities/matlab');
addpath_matlab_utilities('general', 'validation');

%%
data = load('data.mat');

fmt = @(s) struct(...
    'ts', s.ts, ...
    'ps', cat_cell(2, s.xs(:, 1))', ...
    'qs', cat_cell(2, s.xs(:, 2))');

act = fmt(data.actual);
est = fmt(data.est);

%%
figure(1); clf();
hold('all');
plot(act.ts, act.ps);
plot(est.ts, est.ps, '--', 'LineWidth' ,2);

%%
figure(2); clf();
[tis, acti, esti] = resample_series(act.ts, act.ps, est.ts, est.ps);
plot_compare('pos', tis, 'act', acti, 'est', esti);