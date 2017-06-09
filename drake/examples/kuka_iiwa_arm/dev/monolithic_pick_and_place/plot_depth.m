function plot_depth(depth_image, fig_index)
if nargin < 2
    fig_index = 1;
end

sfigure(fig_index);
imshow(reformat_depth(depth_image));
% drawnow();
disp('Updated depth image');

end
