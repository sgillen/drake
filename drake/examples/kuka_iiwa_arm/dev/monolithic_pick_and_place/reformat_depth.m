function [y] = reformat_depth(u)
% Make sure a depth image can be displayed easily within MATLAB
% y = repmat(u, [1, 1, 3]);
% y(isinf(u), :) = 1; % Better way to do this? Colorize?
y = u;
y(isinf(u)) = 1;
y(isnan(u)) = 0.5; % ...
assert(min(y(:)) >= 0);
y = y / max(y(:));
end
% 
% function cs = set_rgb(logic, value)
% assert(all(size(value) == [1, 3]));
% n = nnz(is);
% end
