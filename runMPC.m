clc
close all

input_limit = 5;

LW = 1.25;                  %Linewidth
FS = 8;                     %fontsize

stop = 50;
periods = zeros(1,stop+1);

% Set measurement parameters
t1 = 0.5;
t2 = 2.0;

p = 1;    % perturbation level, 0 is nominal

% Set perturbation parameters
d = p*0.1;                             %input disturbance
Delta = p*0.05;                        %unmodeled dynamics
eta = p*0.01;                          %noise
theta_c = ceil(p*25e-3/stepsize);      %timer initialization error; 1st number is in milisec
omega_c = ceil(p*5e-3/stepsize);       %timer rate error; 1st number is in milisec

drop = ceil(p)*5;                      % # of data dropouts
if(drop>0)
    loss = randi(stop,drop,1);         % data dropouts at iterations loss
else
	loss = -1;
end

%% 1st iteration

i = 0;
tf = 0;

% random initialization for unmodeled dynamics integrator
angle = 2*pi*rand(1);
Delta0 = Delta*rand(1)*[cos(angle) sin(angle)]';

% initialize initial condition
angle = 2*pi*rand(1);
x0 = 0.8*[cos(angle) sin(angle)]';
x0 = x0-Delta0;      %ensures the actual initial condition is the above line

% initialize measured initial condition
angle = 2*pi*rand(1);
dist = eta*rand(1)*[cos(angle) sin(angle)]';
xm = x0+dist;

stoptime = (t2-t1)*rand(1)+t1;  % randomly select sampling period
sim('LQR.slx')                  % generate input using noisy initial condition xm

% introduce timer errors
[u_data,init] = timererror(u_opt_nom.Data,theta_c,omega_c);
u_time = u_opt_nom.Time(init:end);
u_opt = timeseries(u_data,u_time);

% run
sim('controlledplant.slx')

% adjust timestamps
x.Time = x.Time+tf;
x_norm.Time = x_norm.Time+tf;
u_opt.Time = u_opt.Time+tf;
tf = x.Time(end);
periods(i+1) = tf;

% create storage vectors
y = x;
v = u_opt;
y_norm = x_norm;

% record terminal conditions, set as initial conditions
% for next iteration
Delta0 = [do.Data(end,:)]';        %unmodeled dynamics integrator
x0 = x.Data(end,:)'-Delta0;        %plant
angle = 2*pi*rand(1);
dist = eta*rand(1)*[cos(angle) sin(angle)]';
xm = x0+dist;               %sample, corrupted

%% rest of the iterations

for i = 1:stop
    stoptime = (t2-t1)*rand(1)+t1;  % randomly select sampling period
    sim('LQR.slx')                  % generate input using noisy initial condition xm

    % introduce timer errors
    [u_data,init] = timererror(u_opt_nom.Data,theta_c,omega_c);
    u_time = u_opt_nom.Time(init:end);
    u_opt = timeseries(u_data,u_time);
    
    if(sum(i==loss))
        u_opt.Data = zeros(size(u_opt.Data));
    end
    
    if(max(abs(u_opt.Data))>input_limit)
        error('INPUT LIMIT EXCEEDED!!!!')
    end
    
    % run
    sim('controlledplant.slx')

    % adjust timestamps
    x.Time = x.Time+tf;
    x_norm.Time = x_norm.Time+tf;
    u_opt.Time = u_opt.Time+tf;
    tf = x.Time(end);
    periods(i+1) = tf;

    % update storage vectors
    y = append(y,x);
    v = append(v,u_opt);
    y_norm = append(y_norm, x_norm);
    
    % record terminal conditions, set as initial conditions
    % for next iteration
    Delta0 = [do.Data(end,:)]';     %unmodeled dynamics integrator
    x0 = x.Data(end,:)'-Delta0;     %plant
    angle = 2*pi*rand(1);
    dist = eta*rand(1)*[cos(angle) sin(angle)]';
    xm = x0+dist;                   %sample, corrupted

end

%%
figure(10), clf
plot(y,'LineWidth',LW)
hold on
h = gcf;
xlabel('')
ylabel('')
title('')
xlabel('Time (s)','FontName','Times','FontSize',FS)
ylabel('Plant State','FontName','Times','FontSize',FS)
set(gca,'FontName','Times','FontSize',FS)
legend('$x_1$','$x_2$')
set(h,'Units','inches','Position',[2 2 3.4 2])
set(legend(gca),'FontName','Times','FontSize',FS,'Interpreter','latex')

for i = 1:(length(periods))
    if(sum((i)==loss))
        plot([periods(i) periods(i)],get(gca,'ylim'),'Color','k', 'LineWidth',1)                    %dropouts
    else
        plot([periods(i) periods(i)],get(gca,'ylim'),'Color', [0.31, 0.31, 0.31], 'LineStyle', '-.') %sample times
    end
end

%%
figure(11), clf
plot(v,'LineWidth',LW)
hold on
h = gcf;
xlabel('')
ylabel('')
title('')
xlabel('Time (s)','FontName','Times','FontSize',FS)
ylabel('Input','FontName','Times','FontSize',FS)
set(gca,'FontName','Times','FontSize',FS)
set(h,'Units','inches','Position',[2 2 3.4 2])

for i = 1:(length(periods))
    if(sum((i)==loss))
        plot([periods(i) periods(i)],get(gca,'ylim'),'Color','k', 'LineWidth',1)                        %dropouts
    else
        plot([periods(i) periods(i)],get(gca,'ylim'),'Color', [0.31, 0.31, 0.31], 'LineStyle', '-.')    %sample times
    end
end