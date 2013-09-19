int_d = 3;

pitch2 = pitch(1:int_d:end)*pi/180;
vx2 = vx(1:int_d:end);
time2 = time(1:int_d:end);

n = length(vx2);

vx_d = zeros(n,1);
dvx_d = zeros(n,1);
pitch_d = zeros(n,1);

nwin = 20;
p_order= 5;
dt = 0.2;

for i=nwin+1:n
   
    
    ts = time2(i-nwin:i) - time2(i);
    
    s = vx2(i-nwin:i);
    coeffs_poly = polyfit(ts,s,p_order);
    coeffs_der_poly = polyder(coeffs_poly);
    vx_d(i) = polyval(coeffs_poly,-dt);
    dvx_d(i) = polyval(coeffs_der_poly,-dt);
    
    s = pitch2(i-nwin:i);
    coeffs_poly = polyfit(ts,s,p_order);
    pitch_d(i) = polyval(coeffs_poly,-dt);
%     ts_v = -0.3:0.01:0.01;
%     s_v = polyval(coeffs_poly,ts_v);
%     figure,
%     plot(ts,s,'xr');
%     hold on;
%     plot(ts_v,s_v,'-b');
%     grid;
%        
%     pause
    
end
dec = round(dt/mean(diff(time2(1:20))));

figure,
subplot(1,3,1);
plot(time2(dec:end),[vx2(1:end- dec+1),vx_d(dec:end)]);

subplot(1,3,2);
plot(time2(dec:end),[pitch2(1:end- dec+1),pitch_d(dec:end)]);

subplot(1,3,3);
plot(time2(dec:end),[dvx_d(dec:end)]);

%%
nls = length(vx_d);
win_id = 400:3000;
X = [vx_d(win_id) pitch_d(win_id) ones(length((win_id)),1)];
Y = dvx_d(win_id);

param = pinv(X)*Y

dvx_d_est = X*param;

figure,
plot(time2(win_id),[dvx_d(win_id) dvx_d_est])

%%
npar = 3;
p_hat = zeros(npar,length(win_id));
param  =zeros(npar,1);
Dcov = ones(npar,1)*1e5;
Ucov = eye(npar);
sig_eps  =1;
Qeps =eye(npar)*1e-4;
for id = win_id
 m_vec=[vx_d(id) pitch_d(id) 1];
    
    y_e = dvx_d(id);
 %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%   
 %%UD Filter for Recursive Least Square Problem 
 %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    x = m_vec;
    
    yhat = m_vec*param;
    
    
    f = zeros(npar,1);
    v = zeros(npar,1);
    
    delta = (y_e - yhat);
    

    for j=npar-1:-1:0
        f(j+1) = x(j+1);
        
        for i=0:1:j-1
            
            f(j+1) = f(j+1) + Ucov(i+1,j+1)*x(i+1);
        end
        
        v(j+1) = Dcov(j+1)*f(j+1);
        
    end
    
    alpha = sig_eps^2+ f(0+1)*v(0+1);
    gamma= 1/alpha;
    Dcov(0+1) = sig_eps^2*gamma*(Dcov(0+1)+Qeps(0+1,0+1));
    
    for j=1:1:npar-1
        
        
        beta = alpha;
        alpha = alpha + f(j+1)*v(j+1);
        lambda = - f(j+1)/beta;
        gamma = 1/alpha;
        
        Dcov(j+1) = beta*gamma*(Dcov(j+1)+Qeps(j+1,j+1));
        
        for i=0:1:j-1
            beta = Ucov(i+1,j+1);
            Ucov(i+1,j+1) = beta+v(i+1)*lambda;
            
            v(i+1) = v(i+1)+v(j+1)*beta;
            
            % v is now K_bar
            
        end
    end
    
    
    epsilon =delta*gamma;
    
    for i=0:1:npar-1
        param(i+1) = param(i+1)+v(i+1)*epsilon;  %v[i]*gamma = K_gain[i][l]
    end
    
    p_hat(:,id) = param;

end
figure,plot(p_hat')