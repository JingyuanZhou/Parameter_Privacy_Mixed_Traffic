load("data\ovm_para_ngsim.mat")
use_index = est_ls(:,3)>1 & est_ls(:,3)<15 & est_ls(:,4)>25 & est_ls(:,4)<39;
est_ls = est_ls(use_index,:);
alpha = est_ls(est_ls(:,1)<4.9 & est_ls(:,2)<4.9,1);
beta = est_ls(est_ls(:,1)<4.9 & est_ls(:,2)<4.9,2);

% 可视化原始数据点
figure;
scatter(data(:, 1), data(:, 2), 'filled');
title('Original Data Points');
xlabel('\alpha')
ylabel('\beta')
grid on;

% 可视化PDF
figure;
contourf(X1, X2, pdf_grid);
title('Joint Probability Density Function (PDF)');
xlabel('\alpha')
ylabel('\beta')
colorbar;
grid on;

% 可视化CDF
figure;
contourf(X1, X2, cdf_grid);
title('Cumulative Distribution Function (CDF)');
xlabel('\alpha')
ylabel('\beta')
colorbar;
grid on;

figure;
contourf(X1, X2, cdf_grid_normalized);
title('Normalized Cumulative Distribution Function (CDF)');
xlabel('\alpha')
ylabel('\beta')
colorbar;
grid on;