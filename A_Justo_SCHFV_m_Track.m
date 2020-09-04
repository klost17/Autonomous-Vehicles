clear all
%close all
clc

% For reproducibility
rng default;

% Parameters for STATE FEEDBACK WITH INTEGRAL ACTION
tfinal=500;
d = 230;      % Circuit length [m]
v_r_0 = 10/3.6;
T_s_0 = 30;

% Parameters for STATE FEEDBACK WITH INTEGRAL ACTION
tfinal=500;
d = 300;      % Circuit length [m]
v_r_0 = 16.7/3.6;
T_s_0 = 50;

% Parameters for STATE FEEDBACK WITH INTEGRAL ACTION
tfinal=500;
d = 500;      % Circuit length [m]
v_r_0 = 16.7/3.6;
T_s_0 = 15;

% Parameters for STATE FEEDBACK WITH INTEGRAL ACTION
tfinal=800;
d = 320;      % Circuit length [m]
v_r_0 = 20/3.6;
T_s_0 = 60;

% % Parameters for STATE FEEDBACK WITH INTEGRAL ACTION
% tfinal=800;
% d = 400;      % Circuit length [m]
% v_r_0 = 23/3.6;
% T_s_0 = 70;
% 
% tfinal=200;
% d = 230;      % Circuit length [m] for PROPORTIONAL CONTROL
% v_r_0 = 16.7/3.6;
 kgain = 0.02;

tfinal=400;
d = 200;      % 230 - Circuit length [m] for PROPORTIONAL CONTROL
v_r_0 = 20/3.6;
kgain = 0.02;

% tfinal=300;
% d = 280;      % Circuit length [m] for PROPORTIONAL CONTROL
% v_r_0 = 25/3.6;
% kgain = 0.01;

R=d/(2*pi);	% Radius [m]
% Number of vehicles in the first fleet of human driven vehicles
n1=10;
%n1=21;
% Number of vehicles in the second fleet of human driven vehicles
%n2=10;
% Number of vehicles
N=n1+1;
%N=n1+n2+2;

% Vehicle types
isACC = [1 zeros(1,n1)]; %1 zeros(1,n2)];
isIDM = [0 ones(1,n1)];%  0 ones(1,n2)];
isMIT = [0 zeros(1,n1)];% 0 zeros(1,n2)];
% isACC = [1 zeros(1,n1) 1 zeros(1,n2)];
% isIDM = [0 ones(1,n1) 0 ones(1,n2)];
% isMIT = [0 zeros(1,n1) 0 zeros(1,n2)];

accind = find(isACC==1);
if numel(accind)==1
    acc1 = accind(1); acc2 = 0;
elseif numel(accind)==2
    acc1 = accind(1); acc2 = accind(2);
else
    acc1 = 0; acc2 = 0;
end

Smit = diag(isMIT);
Sidm = diag(isIDM);
Sacc = diag(isACC);

isALL = isIDM+isMIT+isACC;
if numel(isIDM)~=N || numel(isMIT)~=N || numel(isACC)~=N || numel(find(isALL==1))~=N
    error('Something is wrong. Please check number and types of vehicles.')
end

% Vehicle parameter: Length of vehicles (mean value; standard deviation)
l=4.5;
ls=0.2;
% Vehicle parameter: Time constant
tau=0.5;
taus=0.1;
% IDM parameter: Safety time headway [s]
IDM_t=0.7;
IDM_ts=0.2;

% Generation of vectors and matrices
sep_h_H_1 = zeros(1,N); sep_h_H_1(acc1+1) = 1;
sep_v_H_1 = zeros(1,N); sep_v_H_1(acc1+n1) = 1;
sep_a_H_1 = zeros(1,N); sep_a_H_1(acc1+n1) = 1;
sep_v_A_1 = zeros(1,N); sep_v_A_1(acc1) = 1;
sep_a_A_1 = zeros(1,N); sep_a_A_1(acc1) = 1;
sep_h_H_2 = zeros(1,N); %sep_h_H_2(acc2+1) = 1;
sep_v_H_2 = zeros(1,N); %sep_v_H_2(N) = 1;
sep_a_H_2 = zeros(1,N); %sep_a_H_2(N) = 1;
sep_v_A_2 = zeros(1,N); %sep_v_A_2(acc2) = 1;
sep_a_A_2 = zeros(1,N); %sep_a_A_2(acc2) = 1;
add_u_1 = zeros(N,1); add_u_1(acc1) = 1;
add_u_2 = zeros(N,1); %add_u_2(acc2) = 1;
% ----------------------------------------------
% sep_h_H_1 = zeros(1,N); sep_h_H_1(acc1+1) = 1;
% sep_v_H_1 = zeros(1,N); sep_v_H_1(acc1+n1) = 1;
% sep_a_H_1 = zeros(1,N); sep_a_H_1(acc1+n1) = 1;
% sep_v_A_1 = zeros(1,N); sep_v_A_1(acc1) = 1;
% sep_a_A_1 = zeros(1,N); sep_a_A_1(acc1) = 1;
% sep_h_H_2 = zeros(1,N); sep_h_H_2(acc2+1) = 1;
% sep_v_H_2 = zeros(1,N); sep_v_H_2(N) = 1;
% sep_a_H_2 = zeros(1,N); sep_a_H_2(N) = 1;
% sep_v_A_2 = zeros(1,N); sep_v_A_2(acc2) = 1;
% sep_a_A_2 = zeros(1,N); sep_a_A_2(acc2) = 1;
% add_u_1 = zeros(N,1); add_u_1(acc1) = 1;
% add_u_2 = zeros(N,1); add_u_2(acc2) = 1;
for i=1:N
    Lind(i)=normrnd(l,ls);
    IDM_Tind(i,i)=normrnd(IDM_t,IDM_ts);
end
% Lini(1) = Lind(1);
% Lini(2) = sum(Lind(2:(n1+1)));
% Lini(3) = Lind(n1+2);
% Lini(4) = sum(Lind((n1+3):(n1+n2+2)));
Lini = Lind;
L = Lini;
% IDM_T(1,1) = IDM_Tind(1);
% IDM_T(2,2) = sum(IDM_Tind(2:(n1+1)));
% IDM_T(3,3) = IDM_Tind(n1+2);
% IDM_T(4,4) = sum(IDM_Tind((n1+3):(n1+n2+2)));
IDM_T = IDM_Tind;
% Vehicle parameters
% Tau(1,1)=normrnd(tau_A,tau_As); iTau(1,1)=1/Tau(1,1);
% Tau(2,2)=normrnd(tau_H,tau_Hs); iTau(2,2)=1/Tau(2,2);
% Tau(3,3)=normrnd(tau_A,tau_As); iTau(3,3)=1/Tau(3,3);
% Tau(4,4)=normrnd(tau_H,tau_Hs); iTau(4,4)=1/Tau(4,4);
for i=1:N
    Tau(i,i) = normrnd(tau,taus);
    iTau(i,i)=1/Tau(i,i);
    thini(i)=(l+0.5)/R*(N-i);
    vini(i)=0;
    aini(i)=0;
	B(i,i)=-1;
    if i==1
        B(i,N)=1;
        Bv(i,N)=1;
    else
    	B(i,i-1)=1;
        Bv(i,i-1)=1;
    end
end
[MITSIM_a_Free_thr_low,MITSIM_a_Free_thr_mid,...
    MITSIM_a_Free_thr_high,MITSIM_a_Free_bra_low,...
    MITSIM_a_Free_bra_midlow,MITSIM_a_Free_bra_mid,...
    MITSIM_a_Free_bra_midhigh,MITSIM_a_Free_bra_high,MITSIM_Alpha_thr,...
    MITSIM_Alpha_bra,MITSIM_Beta_thr,MITSIM_Beta_bra,MITSIM_Gamma_thr,...
    MITSIM_Gamma_bra,MITSIM_T_upper,MITSIM_T_lower,IDM_Amax,IDM_Bmax,...
    IDM_H0,IDM_V0,IDM_Delta,IDM_iSqab] = A_Justo_SCHFV_f_Parameters(N);
% Initial angular positions
for i=1:N
    if i==N
        thini(i)=0;
    else
        thini(i)=sum(IDM_H0(N:-1:(i+1))+L(N:-1:(i+1)))/R;
    end
end
% thini(1:(acc2-1)) = thini(acc2:N) + pi;

% Maximum possible number of vehicles given the size of the track
lengthMin = sum(Lini+IDM_H0);
if lengthMin<d
    disp(['Minimum needed circuit length: ' num2str(lengthMin) ...
        ' m. Actual circuit length: ' num2str(d) ...
        ' m. Proceeding with the simulation...'])
else
    disp(['Minimum needed circuit length: ' num2str(lengthMin) ...
        ' m. Actual circuit length: ' num2str(d) ...
        ' m. Stopping simulation.'])
    error('There are too many vehicles given the size of the track.')
end

%% Simulations (by Simulink)
maxstepsize=1e-2;
sim('A_Justo_SCHFV_slx_Track')

%% Plots

% Colors
ColorHDV=[1 1 1]*0.5;
ColorAV1=[0 0.4470 0.7410];
ColorAV2=[0.8500 0.3250 0.0980];
ColorVH1=[1 1 1]*0.3;
ColorVA1=ColorAV1;%[1 0 1];

figure
subplot(2,1,1)
for i=1:N
    if i==acc1 && acc2~=0
        col=ColorAV1; lw=2;
    elseif i==acc2
        col=ColorAV2; lw=2;
    elseif i==acc1+n1 && acc2==0
        col=ColorVH1; lw=2;
    elseif i==acc1 && acc2==0
        col=ColorVA1; lw=2;
    else
        col=ColorHDV; lw=1;
    end
    [aux,loc]=findpeaks(sims_th(:,i));  % sims_th is the angle theta of the positions
    if length(loc)>1
        ind2=loc(1)-1;
        plot(time(1:ind2),sims_th(1:ind2,i)*R,'Color',col,'Linewidth',lw)
        hold on
        for j=1:length(loc)-1
            ind1=loc(j)+1;
            ind2=loc(j+1)-1;
            plot(time(ind1:ind2),sims_th(ind1:ind2,i)*R,'Color',col,'Linewidth',lw)
        end
        ind1=loc(end)+1;
        plot(time(ind1:end),sims_th(ind1:end,i)*R,'Color',col,'Linewidth',lw)
    end
end
ax = gca; ax.FontSize = 20; grid on
title(['Circular track of ' num2str(d) ' m with ' num2str(N) ' vehicles'],'FontSize',20)
xlabel('Time [s]','FontSize',20);
ylabel('Position on road [m]','FontSize',20);

% if acc1~=0 && acc2~=0
%     title(['Simulation results for a fleet of ' num2str(N) ' vehicles. AUT vehicles are #' num2str(acc1) ' and #' num2str(acc2)],'FontSize',20)
% elseif acc1~=0 && acc2==0
%     title(['Simulation results for a fleet of ' num2str(N) ' vehicles. AUT vehicle is #' num2str(acc1)],'FontSize',20)
% else 
%     title(['Simulation results for a fleet of ' num2str(N) ' vehicles, without any AUT vehicle'],'FontSize',20)
% end
figure(10)
subplot(2,1,2)
for i=1:N
    if i==acc1 && acc2~=0
        col=ColorAV1; lw=2;
        va1 = plot(time,sims_v(:,i)*3.6,'Color',col,'Linewidth',lw);
        hold on
    elseif i==acc2
        col=ColorAV2; lw=2;
        va2 = plot(time,sims_v(:,i)*3.6,'Color',col,'Linewidth',lw);
        hold on
    elseif i==acc1+n1 && acc2==0
        col=ColorVH1; lw=2;
        vh = plot(time,sims_v(:,i)*3.6,'Color',col,'Linewidth',lw);
        hold on
    elseif i==acc1 && acc2==0
        col=ColorVA1; lw=2;
        va = plot(time,sims_v(:,i)*3.6,'Color',col,'Linewidth',lw);
        hold on
    else
        col=ColorHDV; lw=1;
        plot(time,sims_v(:,i)*3.6,'Color',col,'Linewidth',lw);
        hold on
    end
    plot(time,sims_v(:,i)*3.6,'Color',col,'Linewidth',lw)
    hold on
end
av = plot(time,sims_vav*3.6,':k','Linewidth',1);
hold on
vr = plot(time,sims_v_r*3.6,'r');
legend([vr,va1,va2,av],{'Reference speed','Autonomous vehicle #1',...
    'Autonomous vehicle #2','Average speed'},'Location','SouthEast','Fontsize',14);
% legend([vr,va,vh,av],{'Reference speed','Autonomous vehicle',...
%     'Last fleet vehicle','Average speed'},'Location','SouthEast','Fontsize',14);
ax = gca; ax.FontSize = 20; grid on
title(['Circular track of ' num2str(d) ' m with ' num2str(N) ' vehicles'],'FontSize',20)
xlabel('Time [s]','FontSize',20);
ylabel('Speed [km/h]','FontSize',20);
ylim([0 30])

%% Comparison
x = sims_h + L;
L1 = sum(x(:,2:(acc1+n1)),2)-sims_h(:,2);

figure(7)
subplot(2,1,1)
plot(time,sims_v_r*3.6,'r'); hold on
plot(time,sims_vav*3.6,':k'); hold on
plot(time,sims_v(:,acc1+n1)*3.6,'Color',ColorVH1,'Linewidth',1)
ax = gca; ax.FontSize = 20; grid on
title(['Circular track of ' num2str(d) ' m with ' num2str(N) ' vehicles'],'FontSize',20)
xlabel('Time [s]','FontSize',20); ylabel('Speed [km/h]','FontSize',20);
ylim([0 25]); hold on;

subplot(2,1,2)
plot(time,L1,'Color',ColorVH1)
ax = gca; ax.FontSize = 20; grid on
ylim([0 d])
title('Comparison between lengths','FontSize',20)
xlabel('Time [s]','FontSize',20);
ylabel('Length of the fleet [m]','FontSize',20);
hold on