clear all
%close all
clc

% Parameters
tfinal=400;
d=200;      % Circuit length [m]
R=d/(2*pi);	% Radius [m]
N=11;       % Number of vehicles

% For reproducibility
rng default;

% Vehicle types
isMIT = [0 0 0 0 0 0 0 0 0 0 0];
isIDM = [1 1 1 1 1 1 1 1 1 1 1];

acc1 = 0; acc2 = 0; n1 = 0;
ColorAV1=[0 0.4470 0.7410];
ColorAV2=[0.8500 0.3250 0.0980];
ColorVH1=[1 1 1]*0.3;
ColorVA1=ColorAV1;

% isMIT = [0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0];
% isIDM = [1 1 1 1 1 1 1 1 1 1 1 1 1 1 1 1 1 1 1 1 1 1];
% %isIDM = [1 1 0 0 0 1 0 0 1 1 1 1 0 1 0 0 0 1 1 1 0 1];
% %isMIT = [0 0 1 1 1 0 1 1 0 0 0 0 1 0 1 1 1 0 0 0 1 0];

Smit = diag(isMIT); Sidm = diag(isIDM);

isALL = isMIT + isIDM;
if numel(isMIT)~=N || numel(isIDM)~=N || numel(find(isALL==1))~=N
    error('Something is wrong. Please check number and types of vehicles.')
end

% Vehicle parameter: Length of vehicles (mean value; standard deviation)
l=4.5;
ls=0.2;
% Vehicle parameter: Time constant of vehicles
tau=0.5;
taus=0.1;
% IDM parameter: Safety time headway [s]
IDM_t=0.7;
IDM_ts=0.2;

% Generation of vectors and matrices
for i=1:N
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
    L(i)=normrnd(l,ls);
    Tau(i,i)=normrnd(tau,taus);
    iTau(i,i)=1/Tau(i,i);
    % IDM
    IDM_T(i,i)=normrnd(IDM_t,IDM_ts);
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
%thini(1:(N/2)) = thini((N/2+1):N) + pi;

% Maximum possible number of vehicles given the size of the track
lengthMin = sum(L+IDM_H0);
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
sim('A_Justo_SCHFV_slx_IV')

ColorHDV=[1 1 1]*0.5;
figure(1)
subplot(2,1,2)
for i=1:N
    plot(time,sims_v(:,i)*3.6,'Color',ColorHDV,'Linewidth',1)
    hold on
end
plot(time,sims_vav*3.6,'-k','Linewidth',2)
title(['Circular track of ' num2str(d) ' m with ' num2str(N) ' vehicles governed by Intelligent Driver Model'],'FontSize',20)
%title(['Circular track of ' num2str(d) ' m with ' num2str(N) ' vehicles governed by MITSIM Model'],'FontSize',20)
ax = gca; ax.FontSize = 20;
xlabel('Time [s]','FontSize',20); ylim([0 60])
ylabel('Speed [km/h]','FontSize',20); grid on; hold on

%% Plots

% Colors
ColorHDV=[1 1 1]*0.5;

figure
% Positions
subplot(2,1,1)
for i=1:N
    [aux,loc]=findpeaks(sims_th(:,i));
    if length(loc)>1
        ind2=loc(1)-1;
        plot(time(1:ind2),sims_th(1:ind2,i)*R,'Color',ColorHDV,'Linewidth',1)
        hold on
        for j=1:length(loc)-1
            ind1=loc(j)+1;
            ind2=loc(j+1)-1;
            plot(time(ind1:ind2),sims_th(ind1:ind2,i)*R,'Color',ColorHDV,'Linewidth',1)
        end
        ind1=loc(end)+1;
        plot(time(ind1:end),sims_th(ind1:end,i)*R,'Color',ColorHDV,'Linewidth',1)
    end
end
ax = gca; ax.FontSize = 20; grid on
xlabel('Time [s]','FontSize',20);
ylabel('Position on road [m]','FontSize',20);
title(['Circular track of ' num2str(d) ' m with ' num2str(N) ' vehicles governed by a human driving model'],'FontSize',20)
% Speeds
figure(10)
subplot(2,1,1)
for i=1:N
    plot(time,sims_v(:,i)*3.6,'Color',ColorHDV,'Linewidth',1)
    hold on
end
av = plot(time,sims_vav*3.6,'-k','Linewidth',2);
legend(av,{'Average speed'},'Location','NorthEast','Fontsize',14);
ax = gca; ax.FontSize = 20; grid on
title(['Circular track of ' num2str(d) ' m with ' num2str(N) ' vehicles governed by Intelligent Driver Model'],'FontSize',20)
xlabel('Time [s]','FontSize',20);
ylabel('Speed [km/h]','FontSize',20);
ylim([0 30])