function [x] = cat_cell(dim, C)
% Expand cell array for CAT.
x = cat(dim, C{:});

end
