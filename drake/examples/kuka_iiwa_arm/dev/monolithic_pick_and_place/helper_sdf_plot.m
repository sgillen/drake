startup_drake_client;

do_reset = true;
if do_reset
    system(['test -f /tmp/matlab_rpc && rm /tmp/matlab_rpc ', ...
        '&& mkfifo /tmp/matlab_rpc']);
    fprintf('Reset FIFO buffer\n');
end

%%

% This is truncated???
figure(1); clf();
imshow(depth_image);

figure(2); clf();
imshow(observation_sdf);

%
figure(3); clf();
imshow(full_depth_image);

figure(4); clf();
ncloud = size(full_cloud, 2);
is = 1:100:ncloud;
plot(is, full_cloud(:, is));
legend({'x', 'y', 'z'});
