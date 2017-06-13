function [] = show_sparsity(index, Q, f, K)

sfigure(index + 1);
subplot(1, 2, 1);
spy(Q);
subplot(1, 2, 2);
spy(f);
fprintf('K[%d] = %g\n', index, K);
drawnow();

end
