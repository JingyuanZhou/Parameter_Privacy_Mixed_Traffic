I0 = 0.4;

if_privacy_bound = 0;
if_lips = 0;

if if_privacy_bound
    path = ['model/model_',num2str(I0),'_low_privacy'];
else
    path = ['model/model_',num2str(I0),'_low_privacy'];
end

if if_lips
    path = [path, '_lips', '.pth'];
else
    path = [path, '.pth'];
end

Py = Py_calculation(path, I0, if_privacy_bound, if_lips);
Ey = -sum(sum(Py.*log2(Py+eps)));
disp(Ey)
disp(Ey-I0)

N = 60;
a = linspace(0.1, 1, N); % discreted a
b = linspace(0.1, 1, N); % discreted b
para_ls = [];
Ey_x_mat = zeros(N,N);
Nx = 1;
for alpha = a
    Ny = 1;
    for beta = b
        para = [beta, alpha];
        para_ls = [para_ls;para];
        Py_x = NN_test(path,para,I0,if_privacy_bound,if_lips);
        Ey_x = -sum(sum(Py_x.*log2(Py_x)));
        Ey_x_mat(Nx, Ny) = Ey_x;
        Ny = Ny + 1;
    end
    Nx = Nx + 1;
end

if if_privacy_bound

    privacy_level = ones(N,N)*(Ey-I0);
    xy = 0:0.2:1;
    surf(b, a, Ey_x_mat,'EdgeColor','none');
    shading interp
    hold on;
    %surf(a, b, privacy_level,'EdgeColor','none')
    xticks(xy);
    yticks(xy);
    xlabel('\beta', 'Fontname', 'Times New Roman','FontSize',18)
    ylabel('\alpha', 'Fontname', 'Times New Roman','FontSize',18)
    zlabel('bit', 'Fontname', 'Times New Roman','FontSize',18)
    
    %legend({'marginal probability entropy','privacy leve related entropy'}, 'Fontname', 'Times New Roman')
    
    ax = gca;
    exportgraphics(ax,['../figs/continuous_3D_I0_',num2str(I0),'.eps'])

else
    privacy_level = ones(N,N)*(Ey-I0);
    xy = 0:0.2:1;
    surf(b, a, Ey_x_mat,'EdgeColor','none');
    shading interp
    hold on;
    surf(b, a, privacy_level,'EdgeColor','none')
    xticks(xy);
    yticks(xy);
    xlabel('\beta', 'Fontname', 'Times New Roman','FontSize',18)
    ylabel('\alpha', 'Fontname', 'Times New Roman','FontSize',18)
    zlabel('bit', 'Fontname', 'Times New Roman','FontSize',18)
    
    %legend({'marginal probability entropy','privacy leve related entropy'}, 'Fontname', 'Times New Roman')
    
    ax = gca;
    exportgraphics(ax,['../figs/continuous_3D_I0_',num2str(I0),'_low_privacy','.eps'])
end


