function [y] = reformat_depth(u)
% Make sure a depth image can be displayed easily within MATLAB
y = u;
y(~isfinite(u)) = 0;
assert(min(y(:)) >= 0);
y = y / max(y(:));
end
