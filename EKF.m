%% Lab 3 - Extended Kalman Filter
close all;
clear;
clc;

rng(1);

N = 100;

delta = 1;
sig_u_sqr = 0.0001;
sig_r_sqr = 0.1;
sig_b_sqr = 0.01;

A = eye(4) + diag([delta delta],2);
auxvar = [0 0 sig_u_sqr sig_u_sqr];
Q = diag(auxvar);
C = diag([sig_r_sqr sig_b_sqr]);

Un = sqrt(sig_u_sqr)*randn(2,N);
U = [zeros(2,N); Un];
R_ideal = zeros(2,N);

vn =[-0.2; 0.2]; % vn[-1]
rn = [10; -5];  % rn[-1]

rn_ideal = rn;
R_ideal(:,1) = rn;
S(:,1)= [rn; vn];
h_sn(:,1) = [ sqrt(S(1,1)^2 + S(2,1)^2); atan( S(2,1)/S(1,1) ) ];
for n=2:N
    rn_ideal = rn_ideal + [-0.2; 0.2]*delta; 
    R_ideal(:,n) = rn_ideal;
    S(:,n) = A*S(:,n-1) + U(:,n);
    h_sn(:,n) = [ sqrt(S(1,n)^2 + S(2,n)^2); atan2( S(2,n),S(1,n) ) ];
end

w = [sqrt(sig_r_sqr)*randn(1,N); sqrt(sig_b_sqr)*randn(1,N)];
xn = h_sn + w;

R_obs = [xn(1,:).*cos(xn(2,:)); xn(1,:).*sin(xn(2,:))];

figure
plot(R_ideal(1,:),R_ideal(2,:),'--k',S(1,:),S(2,:),'-k',R_obs(1,:),R_obs(2,:),'-b');
xlabel('$r_x$','interpreter','latex')
ylabel('$r_y$','interpreter','latex')
legend('Ideal track', 'True track', 'Observed track')
grid on;

figure
plot(0:N-1,h_sn(1,:)); 
xlabel('n','interpreter','latex')
ylabel('$R[n]$','interpreter','latex')
title('Range','interpreter','latex')
grid on;

figure
plot(0:N-1,h_sn(2,:)*180/pi); 
xlabel('n','interpreter','latex')
ylabel('$\beta [n]$ (degrees)','interpreter','latex')
title('Bearing','interpreter','latex')
grid on;
%% Extended Kalman filter

% Initial EKF state
s_hat = [5; 5; 0; 0];                    % s_hat[-1|-1]
M = 100*eye(4);                          % a reasonable initial uncertainty

%initializing to save EKF estimate
S_hat = zeros(4,N);

I4 = eye(4);

M11 = zeros(1,N);
M22 = zeros(1,N);

for n = 1:N
    %prediction
    s_pred = A * s_hat;
    M_pred = A * M * A' + Q;
    
    % Linearization h(.) at s_pred
    rx = s_pred(1); 
    ry = s_pred(2);
    rho = sqrt(rx^2 + ry^2);

    h_pred = [rho; atan2(ry, rx)];

    H = [ rx/rho,        ry/rho,        0, 0;
         -ry/(rho^2),    rx/(rho^2),    0, 0];

    % updating
    z = xn(:,n);                        % measurement [range; bearing]
    innov = z - h_pred;

    % Wrap bearing innovation to [-pi, pi]
    innov(2) = atan2(sin(innov(2)), cos(innov(2)));

    S_innov = H * M_pred * H' + C;  % innovation covariance
    K = M_pred * H' / S_innov;          % Kalman gain
    s_hat = s_pred + K * innov;

    % Covariance update (more stable numerically)
    M = (I4 - K*H) * M_pred;
    M11(n) = M(1,1);   % minimum MSE for r_x[n]
    M22(n) = M(2,2);   % minimum MSE for r_y[n]

    % Save
    S_hat(:,n) = s_hat;
end

% plotting
figure
plot(S(1,:),S(2,:),'-k', S_hat(1,:),S_hat(2,:),'-r');
xlabel('$r_x$','interpreter','latex')
ylabel('$r_y$','interpreter','latex')
legend('True track','EKF estimate')
grid on;

figure
plot(0:N-1, M11, '-k','LineWidth',1.2);
xlabel('Sample number, n','interpreter','latex')
ylabel('Minimum MSE for $r_x[n]$','interpreter','latex')
grid on;

figure
plot(0:N-1, M22, '-k','LineWidth',1.2);
xlabel('Sample number, n','interpreter','latex')
ylabel('Minimum MSE for $r_y[n]$','interpreter','latex')
grid on;