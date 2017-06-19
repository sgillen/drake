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
dt = 0.03;
tds = 0:dt:est.ts(end);
[tis, esti, acti] = resample_series(est.ts, est.ps, act.ts, act.ps, tds);

logic = tis > 4.7;
tsis = tis(logic);
actsi = acti(logic, :);
estsi = esti(logic, :);

%%
figure(1); clf();
e = plot_compare('Cartesian Position', tsis, 'Actual', actsi, 'Estimated', estsi);
legend({'x', 'y', 'z'});

%%
figure(2); clf();
dts = diff(tsis)';
% dts(end + 1) = dts(end);
nd = 3;
zpad = zeros(1, nd);
da = diff(actsi) ./ repmat(dts, 1, nd);
db = diff(estsi) ./ repmat(dts, 1, nd);
e = plot_compare('Cartesian Velocitiy F. Diff', tsis(2:end), 'Actual', da, 'Estimated', db);
legend({'x', 'y', 'z'});
