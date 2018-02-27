clear all
close all
clc

stepsize = 1e-4;

c = cos(0:stepsize:2*pi);
s = sin(0:stepsize:2*pi);

% Plant parameters
n = 2;
m = 1;

A = [0.2 0.7;
    0.7  0.2];
B = [0.1 0.5]';

%%

% Optimal Control parameters
T = 3;              %prediction horizon

poles = [-1 -2];
k = place(A,B,poles);
Abar = A-B*k;

Q = eye(2); %state cost matrix
R = eye(1); %input cost matrix

M = lyap(Abar',Q+k'*R*k)

clear poles

%%

close all

sim('riccati.slx')  %simulate Riccati equation

% Flip vectors obtained via backward propagation
P = (P_t.data);
K = (K_t.data);

for i = 1:n
    for j = 1:max(m,n)
        if j<=n
        P(i,j,:) = wrev(P(i,j,:));
        end
        if j<=m
        K(j,i,:) = wrev(K(j,i,:));
        end
    end
end

P_t.data = P;
K_t.data = K;

clear P
clear K
%%
% Simulate LQR to se that unconstrained LQR verifies the constraints 
xm = rand(n,1);

xm = [1 0]';
stoptime = T;
sim('LQR.slx')
figure(1)
subplot(2,2,1)
plot(x_nom)
hold on
plot(x_nom_norm,'k-.')
xlabel('t')
legend('$\chi_1$','$\chi_2$','$|\chi|$')
set(legend(gca),'Interpreter','latex')
subplot(2,2,2)
plot(c,s,'r--')
xlabel('x_1')
ylabel('x_2')
hold on
plot(x_nom.Data(:,1),x_nom.Data(:,2))
legend('$|x|=1$','$\chi$')
set(legend(gca),'Interpreter','latex')
clear x_nom
clear x_nom_norm
%

xm = [0 1]';
stoptime = T;
sim('LQR.slx')
figure(1)
subplot(2,2,3)
plot(x_nom)
hold on
plot(x_nom_norm,'k-.')
xlabel('t')
legend('$\chi_1$','$\chi_2$','$|\chi|$')
set(legend(gca),'Interpreter','latex')
subplot(2,2,4)
plot(c,s,'r--')
hold on
plot(x_nom.Data(:,1),x_nom.Data(:,2))
xlabel('x_1')
ylabel('x_2')
legend('$|x|=1$','$\chi$')
set(legend(gca),'Interpreter','latex')
clear x_nom
clear x_nom_norm

%% Monte-Carlo like LQR simulation to se that unconstrained LQR verifies the constraints
th = linspace(0,2*pi,6);   % increase last number for more simulations

figure(2), clf
plot(c,s,'r--')
xlabel('x_1')
ylabel('x_2')
legend('$|x|=1$')
set(legend(gca),'Interpreter','latex')
title('LQR simulations from the unit disk, with level sets of the terminal cost')
hold on

figure(3), clf
title('Maximum norm of inputs corresponding to each LQR simulation')
hold on
for i = 1:length(th)
    xm = [cos(th(i)) sin(th(i))]';
    stoptime = T;
    sim('LQR.slx')
    figure(2)
    plot(x_nom.Data(:,1),x_nom.Data(:,2))
    plot(x_nom.Data(end,1),x_nom.Data(end,2),'ko')
    figure(3)
    plot(i,max(abs(u_opt_nom.Data)),'k.')
end
figure(3), grid on

xax = linspace(-1,1,20);
yax = linspace(-1,1,20);
[Xax,Yax] = meshgrid(xax,yax);
Z = M(1,1)*Xax.^2 + M(2,2)*Yax.^2 - 2*M(2,1)*Xax.*Yax;

figure(2)
contour(Xax,Yax,Z,'ShowText','on')

clear th

clear xax
clear yax
clear Xax
clear Yax
clear Z