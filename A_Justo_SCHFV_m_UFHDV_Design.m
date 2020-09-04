%clear all
%close all
clc

% For reproducibility
rng default;

tfinal=800;
v_r_0 = 10/3.6;
kgain = 0.02;

% Number of vehicles in the fleet of human driven vehicles
n=10;
% Vehicle parameter: Length of vehicles (mean value; standard deviation)
l=4.5;
ls=0.2;
% Vehicle parameter: Time constant
tau=0.5;
taus=0.1;
% IDM parameter: Safety time headway [s]
IDM_t=0.7;
IDM_ts=0.2;

% Parameters
Tau_H=normrnd(tau,taus);
Tau_A=normrnd(tau,taus);
IDM_T=normrnd(IDM_t,IDM_ts);

eta =sum(normrnd(IDM_t,IDM_ts,1,n));
Lini=sum(normrnd(l,ls,1,n));
hini=1.01*IDM_H0;
vini=0;
aini=0;
zini=0;
xini = [hini; vini; aini; vini; aini];

[MITSIM_a_Free_thr_low,MITSIM_a_Free_thr_mid,...
    MITSIM_a_Free_thr_high,MITSIM_a_Free_bra_low,...
    MITSIM_a_Free_bra_midlow,MITSIM_a_Free_bra_mid,...
    MITSIM_a_Free_bra_midhigh,MITSIM_a_Free_bra_high,MITSIM_Alpha_thr,...
    MITSIM_Alpha_bra,MITSIM_Beta_thr,MITSIM_Beta_bra,MITSIM_Gamma_thr,...
    MITSIM_Gamma_bra,MITSIM_T_upper,MITSIM_T_lower,IDM_Amax,IDM_Bmax,...
    IDM_H0,IDM_V0,IDM_Delta,IDM_iSqab] = A_Justo_SCHFV_f_Parameters(1);

% Design of the control
T_s = 70; %settling time
M_p = 1e-10; %overshoot
sigma = 4/T_s; %real part of dominant pair
omega = -pi*sigma/log(M_p); %imaginary part of dominant pair
xi = sqrt(1/(1+(omega/sigma)^2)); %damping coefficient or damping ratio
omega_0 = sigma/xi; %natural frequency
psi = atan(sqrt(1-xi^2)/xi);
T_r = 1/omega_0*exp(psi/tan(psi)); %rise time
if xi<1
    disp(['Underdamped 2nd order system: xi = ' num2str(xi) ' < 1'])
elseif xi>1
    disp(['Overdamped 2nd order system: xi = ' num2str(xi) ' > 1'])
else
    disp(['Critically damped (xi=1) or Oscillator (xi=0): xi = ' num2str(xi)])
end
pDom = -sigma + 1i*omega;
pNeg1 = -21*sigma; pNeg2 = -22*sigma; pNeg3 = -23*sigma; pNeg4 = -24*sigma;
P = [pDom; conj(pDom); pNeg1; pNeg2; pNeg3; pNeg4];

% A_Justo_SCHFV_Control_Sensitivity - Regime 1 in order to design
A31 = 2*IDM_Amax/(Tau_H*(IDM_H0+v_r_0*IDM_T))*(1-(v_r_0/IDM_V0)^IDM_Delta)^(3/2);
A32 = -IDM_Amax/Tau_H*(IDM_Delta*v_r_0^(IDM_Delta-1)/IDM_V0^IDM_Delta+(v_r_0+2*IDM_T*sqrt(IDM_Amax*IDM_Bmax))/((IDM_H0+v_r_0*IDM_T)*sqrt(IDM_Amax*IDM_Bmax))*(1-(v_r_0/IDM_V0)^IDM_Delta));
A34 = IDM_Amax*v_r_0/(Tau_H*((IDM_H0+v_r_0*IDM_T)*sqrt(IDM_Amax*IDM_Bmax)))*(1-(v_r_0/IDM_V0)^IDM_Delta);

% A_Justo_SCHFV_Control_Sensitivity - Regime 0 and 1
A = [ 0    -1     -eta      1       0   ;...
      0     0       1       0       0   ;...
     A31   A32   -1/Tau_H  A34      0   ;...
      0     0       0       0       1   ;...
      0     0       0       0    -1/Tau_A];
B = [0; 0; 0; 0; 1/Tau_A];
C = [0 1 0 0 0];

% Bigger matrix to also account for the kz gain
A_K = [A zeros(5,1); C 0];
B_K = [B; 0];

K = place(A_K,B_K,P);
k1 = K(1); k2 = K(2); k3 = K(3); k4 = K(4); k5 = K(5); kz = K(6);
K = [k1 k2 k3 k4 k5];

% Check controllability
W_C = ctrb(A_K,B_K);
if rank(W_C)==6
    disp('The closed loop system is controllable')
else
    error('The closed loop system is not controllable')
end

% A_Justo_SCHFV_Control_Sensitivity - Regime 0 in order to check poles here
A31 = 2*IDM_Amax/(Tau_H*IDM_H0)*(1-(v_r_0/IDM_V0)^IDM_Delta)^(3/2);
A32 = -IDM_Amax/Tau_H*(IDM_Delta*v_r_0^(IDM_Delta-1)/IDM_V0^IDM_Delta);
A34 = 0;
%[num,den] = ss2tf([A-B*K -kz*B; C 0],[0; 0; 0; 0; 0; -1],[C 0],0);
%H = ss2tf([A-B*K -kz*B; C 0],[0; 0; 0; 0; 0; -1],[C 0],0);
%rlocus(H)
p5 = 1/Tau_H + (k5+1)/Tau_A;
p4 = (k4*Tau_H+k5+1)/(Tau_H*Tau_A) + A31*eta - A32;
p3 = (k1*Tau_H + k4)/(Tau_H*Tau_A) + A31*(1+eta*(k5+1)/Tau_A) - A32*(k5+1)/Tau_A + A34*k3/Tau_A;
p2 = k1/(Tau_H*Tau_A) + A31*(k3+eta*k4+k5+1)/Tau_A - A32*k4/Tau_A + A34*(k2-eta*k1)/Tau_A;
p1 = A31*(k2+k4)/Tau_A - A32*k1/Tau_A + A34*(kz-k1)/Tau_A;
p0 = A31*kz/Tau_A;
num2 = [0 0 0 0 0 A34 A31]*kz/Tau_A;
den2 = [1 p5 p4 p3 p2 p1 p0];
P_LOW = sort(roots(den2),'descend','ComparisonMethod','real');
P_HIGH = sort(P,'descend','ComparisonMethod','real');
disp('Regime 1 poles')
disp(P_HIGH')
disp('Regime 0 poles')
disp(P_LOW')
if real(P_LOW(1))>=0 || real(P_HIGH(1))>=0
    error('Please check the design. Not all poles (from the linearized system) have negative real part.')
end

%% Simulations (by Simulink)
maxstepsize=1e-2;
%sim('A_Justo_SCHFV_slx_UFHDV_Design_LinNoGS')
%sim('A_Justo_SCHFV_slx_UFHDV_Design_NoLinNoGS')
sim('A_Justo_SCHFV_slx_UFHDV_Design_NoLinGS')

%% Plots
time=sims_time; sims_v=[sims_x(:,2) sims_x(:,4)];
figure(2)
subplot(2,1,1)
plot(sims_time,sims_v_r*3.6,'r')
hold on
plot(sims_time,sims_x(:,2)*3.6,'Color',ColorVH1,'Linewidth',2)
hold on
plot(sims_time,sims_x(:,4)*3.6,'Color',ColorVA1,'Linewidth',2)
hold off
legend('v_r (Reference)','v_H (Fleet of human driven vehicles)','v_A (Autonomous vehicle)','Threshold for v_A to switch to R0','Location','SouthEast','FontSize',14)
ax = gca; ax.FontSize = 20; grid on
title('One autonomous vehicle and one fleet of human driven vehicles','FontSize',20)
xlabel('Time [s]','FontSize',20);
ylabel('Speed [km/h]','FontSize',20);
ylim([0 30])

figure
subplot(4,1,[1 2])
plot(sims_time,sims_v_r*3.6,'r')
hold on
plot(sims_time,sims_x(:,2)*3.6,'Color',ColorVH1,'Linewidth',2)
hold on
plot(sims_time,sims_x(:,4)*3.6,'Color',ColorVA1,'Linewidth',2)
hold on
plot(sims_time,(sims_x(:,2)+2*IDM_T*sqrt(IDM_Amax*IDM_Bmax))*3.6,':k')
% hold on
% plot(sims_time,sims_v_r*(1-0.02)*3.6,'--r',sims_time,sims_v_r*(1+0.02)*3.6,'--r')
hold off
xlabel('Time [s]','FontSize',20); ylabel('Speed [km/h]','FontSize',20); grid on;
title(['Speed. Settling time: T_s=' num2str(T_s) 's. Maximum overshoot: M_p=' num2str(M_p) '. Rise time: T_r=' num2str(T_r) 's.'],'FontSize',20)
legend('v_r (Reference)','v_H (Fleet of human driven vehicles)','v_A (Autonomous vehicle)','Threshold for v_A to switch to R0','Location','NorthEast','FontSize',20)
ylim([0 250])
subplot(4,1,3)
plot(sims_time,regime)
xlabel('Time [s]','FontSize',20); ylabel('R0 |||| R1','FontSize',20); grid on; %title('Switching of IDM Regime','FontSize',16)
subplot(4,1,4)
plot(sims_time,sims_L_H)
xlabel('Time [s]','FontSize',20); ylabel('Length [m]','FontSize',20); grid on;
title('Length of the fleet of human driven vehicles','FontSize',20)
ylim([0 200])

%% Comparison

figure(7)
subplot(2,1,1)
plot(sims_time,sims_x(:,2)*3.6,'b')
hold off

subplot(2,1,2)
plot(time,sims_L_H,'b')
hold off;