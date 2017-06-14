Q = rand(5, 5);
Q(3, :) = nan;
f = rand(5, 1);
K = 0;

show_sparsity(1, Q, f, K);

Qf = [Q, nan(5, 1), f];

% How to hide a band of pixels?
show_sparsity(1, Qf, f, K);
