figure(1); clf();

startup_drake_client;

%%
figure(1); clf();
imshow(reformat_depth(depth_image));

figure(2); clf();
imshow(observation_sdf);

%%
figure(3); clf();
% imshow(reformat_depth(depth_image));
imshow(reformat_depth(latest_depth_image));
% imshow(reformat_depth(depth_image_mat));

%%
figure(4); clf();
ncloud = size(full_cloud, 2);
is = 1:100:ncloud;
plot(is, full_cloud(:, is));
legend({'x', 'y', 'z'});
