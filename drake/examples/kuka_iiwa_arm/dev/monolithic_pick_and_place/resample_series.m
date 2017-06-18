function [tis, x1is, x2is] = resample_series(t1s, x1s, t2s, x2s)
% Resample data that varies along rows, such that both timestamps match.

[t1s, x1s] = clean(t1s, x1s);
[t2s, x2s] = clean(t2s, x2s);

t_min = max(t1s(1), t2s(1));
t_max = min(t1s(end), t2s(end));

tis = t1s(t1s >= t_min & t1s <= t_max);
x1is = interp1(t1s, x1s, tis');
x2is = interp1(t2s, x2s, tis');

% sz = size(x1s(:, 2));
% 
% res = @(x) reshape(x, [], sz(2));
% 
% t1 = timeseries(t1s, x1s);
% t2 = timeseries(t2s, x2s);
% 
% [ti1, ti2] = synchronize(t1, t2, 'Intersection', 'InterpMethod', 'zoh');
% 
% tis = ti1.Time;
% maxabs = @(x) max(abs(x(:)));
% assert(maxabs(tis - ti2.Time) < eps);
% 
% 
% x1is = res(ti1.Data);
% x2is = res(ti2.Data);

end


function [t, x] = clean(t, x)
% Delete nonunique points
dt = diff(t);
bad = [false; dt < eps];

t(bad) = [];
x(bad, :) = [];
end
