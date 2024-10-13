x_cur = [20,15.7];
x_last = [19.7,16.2];
mu=[19.8,15.8];
sigma = [0.1,0.1];
v_head = 20;
results = [];
cnt = 1;

PF = PrivacyFilter(1,1);
PF = PF.MonteCarlo(1000);

for I0 = 0:0.1:0.8
    result = 0;
    PF = PF.Randomizer(I0);
    for i=1:3000
        PF = PF.NonlinearTransformation(x_cur,x_last,mu,sigma,v_head);
        result = result + PF.MSE_distoration();
    end
    results(cnt) = result/3000;
    cnt=cnt+1;
end

plot(0:0.1:0.8,results)
xlabel('I_0')
ylabel('normalized mse')