function [h] = plot_matrix(M)
%PLOT_MATRIX 2-D Plot of matrix entries
% like a table, but filled with colormap instead of numbers

h = image(M, 'CDataMapping', 'scaled');
colormap(gray(256))
colorbar
axis ij
axis square

end

