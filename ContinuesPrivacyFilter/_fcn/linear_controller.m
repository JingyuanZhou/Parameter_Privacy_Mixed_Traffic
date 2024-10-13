function [u,PF,y] = linear_controller(k,K,y,y_last,PF,pos_cav)

n_cav = length(K(:,1));
u = zeros(1,n_cav);
for i = 1:n_cav
    index = pos_cav(i);
    if index == 5 || k == 1
        u(i) = K(i,:) * y;
    else
        x_cav = [y(index),y(index+length(y)/2)];
        [PF{index},x_tilde] = PF{index}.NonlinearTransformation(x_cav,y_last,x_cav,[0.1,0.1]);

        u(i) = K(i,:) * y;
        y(index) = x_tilde(2);
        y(index+length(y)/2) = x_tilde(1);
    end

    if u(i)>2
        u(i) = 2;
    elseif u(i)<-5
        u(i) = -5;
    end
end
u = u';
end

