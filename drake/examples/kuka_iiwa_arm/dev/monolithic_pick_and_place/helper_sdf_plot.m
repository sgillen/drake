figure(1); clf();

startup_drake_client;

% @ref https://stackoverflow.com/questions/19989565/how-can-i-keep-matlab-figure-window-maximized-when-showing-a-new-image
im_args = {'InitialMagnification','fit'};

%%
figure(1); clf();
imshow(reformat_depth(depth_image), im_args{:});

figure(2); clf();
imshow(observation_sdf, im_args{:});

%%
figure(3); clf();
% imshow(reformat_depth(depth_image));
imshow(reformat_depth(latest_depth_image), im_args{:});
% imshow(reformat_depth(depth_image_mat));

%%
figure(4); clf();
ncloud = size(full_cloud, 2);
is = 1:100:ncloud;
plot(is, full_cloud(:, is));
legend({'x', 'y', 'z'});
