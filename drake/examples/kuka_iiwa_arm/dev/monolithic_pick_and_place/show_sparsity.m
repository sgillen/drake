function [] = show_sparsity(index, Q, f, K)
sfigure(index + 1);
subplot(1, 4, [1 3]);
plot_mat(Q);
subplot(1, 4, 4);
plot_mat(f);
fprintf('K[%d] = %g\n', index, K);
drawnow();
end

function [] = plot_mat(A)
colormap('gray');
image(A, 'CDataMapping','scaled');
colorbar();
axis('equal');
set(gca, 'Visible', 'off');
end
