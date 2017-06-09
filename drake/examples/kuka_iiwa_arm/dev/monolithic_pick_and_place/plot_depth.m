function plot_depth(depth_image, fig_index)
if nargin < 2
    fig_index = 1;
end

sfigure(fig_index);
im_args = {'InitialMagnification','fit'};
imshow(reformat_depth(depth_image), im_args{:});
% drawnow();
disp('Updated depth image');

end
