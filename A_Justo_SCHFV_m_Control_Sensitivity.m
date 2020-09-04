clear all
close all
clc

% For reproducibility
rng default;

% Number of vehicles in the human fleet of vehicles
n=10;
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
[MITSIM_a_Free_thr_low,MITSIM_a_Free_thr_mid,...
    MITSIM_a_Free_thr_high,MITSIM_a_Free_bra_low,...
    MITSIM_a_Free_bra_midlow,MITSIM_a_Free_bra_mid,...
    MITSIM_a_Free_bra_midhigh,MITSIM_a_Free_bra_high,MITSIM_Alpha_thr,...
    MITSIM_Alpha_bra,MITSIM_Beta_thr,MITSIM_Beta_bra,MITSIM_Gamma_thr,...
    MITSIM_Gamma_bra,MITSIM_T_upper,MITSIM_T_lower,IDM_Amax,IDM_Bmax,...
    IDM_H0,IDM_V0,IDM_Delta,IDM_iSqab] = A_Justo_SCHFV_f_Parameters(1);

eta =sum(normrnd(IDM_t,IDM_ts,1,n)); %eta=2.5*eta;

% Control Test
v_r_0 = 30/3.6;

% Design of the control
T_s = 10; %settling time
M_p = 1e-10; %overshoot
sigma = 4/T_s;
omega = -pi*sigma/log(M_p);
pDom = -sigma + 1i*omega;
pNeg1 = -21*sigma; pNeg2 = -22*sigma; pNeg3 = -23*sigma; pNeg4 = -24*sigma;

% Second order system
P = [pDom; conj(pDom); pNeg1; pNeg2; pNeg3; pNeg4];

%% Sensitivity to parameters

% % Variation of eta
% % Regime 0
% A31 = 2*IDM_Amax/Tau_H/(IDM_H0)*(1-(v_r_0/IDM_V0)^IDM_Delta)^(3/2);
% A32 = -IDM_Amax/Tau_H*(IDM_Delta*v_r_0^(IDM_Delta-1)/IDM_V0^IDM_Delta);
% A34 = 0;
% A = [ 0    -1     -eta      1       0    0 ;...
%       0     0       1       0       0    0 ;...
%      A31   A32  -1/Tau_H   A34      0    0 ;...
%       0     0       0       0       1    0 ;...
%       0     0       0       0   -1/Tau_A 0 ;...
%       0     1       0       0       0    0 ];
% B = [0; 0; 0; 0; 1/Tau_A; 0];
% K = place(A,B,P);
% eta_0 = 0.5*eta;
% for i=1:100
%     eta_0 = eta_0 + eta/100;
%     A31 = 2*IDM_Amax/Tau_H/(IDM_H0)*(1-(v_r_0/IDM_V0)^IDM_Delta)^(3/2);
%     A32 = -IDM_Amax/Tau_H*(IDM_Delta*v_r_0^(IDM_Delta-1)/IDM_V0^IDM_Delta);
%     A34 = 0;
%     A = [ 0    -1    -eta_0     1       0    0 ;...
%           0     0       1       0       0    0 ;...
%          A31   A32  -1/Tau_H   A34      0    0 ;...
%           0     0       0       0       1    0 ;...
%           0     0       0       0   -1/Tau_A 0 ;...
%           0     1       0       0       0    0 ];
%       B = [0; 0; 0; 0; 1/Tau_A; 0];
%       Regime0_6(:,i) = eig(A-B*K);
% end
% % Regime 1
% A31 = 2*IDM_Amax/Tau_H/(IDM_H0+v_r_0*IDM_T)*(1-(v_r_0/IDM_V0)^IDM_Delta)^(3/2);
% A32 = -IDM_Amax/Tau_H*(IDM_Delta*v_r_0^(IDM_Delta-1)/IDM_V0^IDM_Delta+(v_r_0+2*IDM_T*sqrt(IDM_Amax*IDM_Bmax))/((IDM_H0+v_r_0*IDM_T)*sqrt(IDM_Amax*IDM_Bmax))*(1-(v_r_0/IDM_V0)^IDM_Delta));
% A34 = IDM_Amax/Tau_H*v_r_0/((IDM_H0+v_r_0*IDM_T)*sqrt(IDM_Amax*IDM_Bmax))*(1-(v_r_0/IDM_V0)^IDM_Delta);
% A = [ 0    -1     -eta      1       0    0 ;...
%       0     0       1       0       0    0 ;...
%      A31   A32  -1/Tau_H   A34      0    0 ;...
%       0     0       0       0       1    0 ;...
%       0     0       0       0   -1/Tau_A 0 ;...
%       0     1       0       0       0    0 ];
% B = [0; 0; 0; 0; 1/Tau_A; 0];
% K = place(A,B,P);
% eta_0 = 0.5*eta;
% for i=1:100
%     eta_0 = eta_0 + eta/100;
%     A31 = 2*IDM_Amax/Tau_H/(IDM_H0+v_r_0*IDM_T)*(1-(v_r_0/IDM_V0)^IDM_Delta)^(3/2);
%     A32 = -IDM_Amax/Tau_H*(IDM_Delta*v_r_0^(IDM_Delta-1)/IDM_V0^IDM_Delta+(v_r_0+2*IDM_T*sqrt(IDM_Amax*IDM_Bmax))/((IDM_H0+v_r_0*IDM_T)*sqrt(IDM_Amax*IDM_Bmax))*(1-(v_r_0/IDM_V0)^IDM_Delta));
%     A34 = IDM_Amax/Tau_H*v_r_0/((IDM_H0+v_r_0*IDM_T)*sqrt(IDM_Amax*IDM_Bmax))*(1-(v_r_0/IDM_V0)^IDM_Delta);
%     A = [ 0    -1    -eta_0     1       0    0 ;...
%           0     0       1       0       0    0 ;...
%          A31   A32  -1/Tau_H   A34      0    0 ;...
%           0     0       0       0       1    0 ;...
%           0     0       0       0   -1/Tau_A 0 ;...
%           0     1       0       0       0    0 ];
%       B = [0; 0; 0; 0; 1/Tau_A; 0];
%       Regime1_6(:,i) = eig(A-B*K);
% end
% figure(1)
% subplot(2,2,1)
% plot(real(Regime0_6(1,:)),imag(Regime0_6(1,:)),'.b',real(Regime0_6(2,:)),imag(Regime0_6(2,:)),'.r',...
%     real(Regime0_6(3,:)),imag(Regime0_6(3,:)),'.g',real(Regime0_6(4,:)),imag(Regime0_6(4,:)),'.y',...
%     real(Regime0_6(5,:)),imag(Regime0_6(5,:)),'.c',real(Regime0_6(6,:)),imag(Regime0_6(6,:)),'.m',...
%     real(Regime0_6(:,length(Regime0_6)/2)),imag(Regime0_6(:,length(Regime0_6)/2)),'xk',...
%     zeros(1,1000),linspace(-6,6,1000),'k')
% axis([-5 1 -4 4])
% ax = gca;
% ax.FontSize = 16;
% title('Sensitivity to \eta in Regime 0','FontSize',16)
% subplot(2,2,2)
% plot(real(Regime1_6(1,:)),imag(Regime1_6(1,:)),'.b',real(Regime1_6(2,:)),imag(Regime1_6(2,:)),'.r',...
%     real(Regime1_6(3,:)),imag(Regime1_6(3,:)),'.g',real(Regime1_6(4,:)),imag(Regime1_6(4,:)),'.y',...
%     real(Regime1_6(5,:)),imag(Regime1_6(5,:)),'.c',real(Regime1_6(6,:)),imag(Regime1_6(6,:)),'.m',...
%     real(Regime1_6(:,length(Regime1_6)/2)),imag(Regime1_6(:,length(Regime1_6)/2)),'xk',...
%     zeros(1,1000),linspace(-6,6,1000),'k')
% axis([-5 1 -4 4])
% ax = gca; ax.FontSize = 20;
% title('Sensitivity to \eta in Regime 1','FontSize',20)
% 
% % Variation of v_r
% % Regime 0
% v_r_0 = IDM_v0/3;
% A31 = 2*IDM_Amax/Tau_H/(IDM_H0)*(1-(v_r_0/IDM_V0)^IDM_Delta)^(3/2);
% A32 = -IDM_Amax/Tau_H*(IDM_Delta*v_r_0^(IDM_Delta-1)/IDM_V0^IDM_Delta);
% A34 = 0;
% A = [ 0    -1     -eta      1       0    0 ;...
%       0     0       1       0       0    0 ;...
%      A31   A32  -1/Tau_H   A34      0    0 ;...
%       0     0       0       0       1    0 ;...
%       0     0       0       0   -1/Tau_A 0 ;...
%       0     1       0       0       0    0 ];
% B = [0; 0; 0; 0; 1/Tau_A; 0];
% K = place(A,B,P);
% v_r_0 = IDM_v0/3 - IDM_v0/3/2;
% for i=1:100
%     v_r_0 = v_r_0 + IDM_v0/3/100;
%     A31 = 2*IDM_Amax/Tau_H/(IDM_H0)*(1-(v_r_0/IDM_V0)^IDM_Delta)^(3/2);
%     A32 = -IDM_Amax/Tau_H*(IDM_Delta*v_r_0^(IDM_Delta-1)/IDM_V0^IDM_Delta);
%     A34 = 0;
%     A = [ 0    -1     -eta      1       0    0 ;...
%           0     0       1       0       0    0 ;...
%          A31   A32  -1/Tau_H   A34      0    0 ;...
%           0     0       0       0       1    0 ;...
%           0     0       0       0   -1/Tau_A 0 ;...
%           0     1       0       0       0    0 ];
%       B = [0; 0; 0; 0; 1/Tau_A; 0];
%       Regime0_6(:,i) = eig(A-B*K);
% end
% % Regime 1
% v_r_0 = IDM_v0/3;
% A31 = 2*IDM_Amax/Tau_H/(IDM_H0+v_r_0*IDM_T)*(1-(v_r_0/IDM_V0)^IDM_Delta)^(3/2);
% A32 = -IDM_Amax/Tau_H*(IDM_Delta*v_r_0^(IDM_Delta-1)/IDM_V0^IDM_Delta+(v_r_0+2*IDM_T*sqrt(IDM_Amax*IDM_Bmax))/((IDM_H0+v_r_0*IDM_T)*sqrt(IDM_Amax*IDM_Bmax))*(1-(v_r_0/IDM_V0)^IDM_Delta));
% A34 = IDM_Amax/Tau_H*v_r_0/((IDM_H0+v_r_0*IDM_T)*sqrt(IDM_Amax*IDM_Bmax))*(1-(v_r_0/IDM_V0)^IDM_Delta);
% A = [ 0    -1     -eta      1       0    0 ;...
%       0     0       1       0       0    0 ;...
%      A31   A32  -1/Tau_H   A34      0    0 ;...
%       0     0       0       0       1    0 ;...
%       0     0       0       0   -1/Tau_A 0 ;...
%       0     1       0       0       0    0 ];
% B = [0; 0; 0; 0; 1/Tau_A; 0];
% K = place(A,B,P);
% v_r_0 = IDM_v0/3 - IDM_v0/3/2;
% for i=1:100
%     v_r_0 = v_r_0 + IDM_v0/3/100;
%     A31 = 2*IDM_Amax/Tau_H/(IDM_H0+v_r_0*IDM_T)*(1-(v_r_0/IDM_V0)^IDM_Delta)^(3/2);
%     A32 = -IDM_Amax/Tau_H*(IDM_Delta*v_r_0^(IDM_Delta-1)/IDM_V0^IDM_Delta+(v_r_0+2*IDM_T*sqrt(IDM_Amax*IDM_Bmax))/((IDM_H0+v_r_0*IDM_T)*sqrt(IDM_Amax*IDM_Bmax))*(1-(v_r_0/IDM_V0)^IDM_Delta));
%     A34 = IDM_Amax/Tau_H*v_r_0/((IDM_H0+v_r_0*IDM_T)*sqrt(IDM_Amax*IDM_Bmax))*(1-(v_r_0/IDM_V0)^IDM_Delta);
%     A = [ 0    -1     -eta      1       0    0 ;...
%           0     0       1       0       0    0 ;...
%          A31   A32  -1/Tau_H   A34      0    0 ;...
%           0     0       0       0       1    0 ;...
%           0     0       0       0   -1/Tau_A 0 ;...
%           0     1       0       0       0    0 ];
%       B = [0; 0; 0; 0; 1/Tau_A; 0];
%       Regime1_6(:,i) = eig(A-B*K);
% end
% figure(1)
% subplot(2,2,3)
% plot(real(Regime0_6(1,:)),imag(Regime0_6(1,:)),'.b',real(Regime0_6(2,:)),imag(Regime0_6(2,:)),'.r',...
%     real(Regime0_6(3,:)),imag(Regime0_6(3,:)),'.g',real(Regime0_6(4,:)),imag(Regime0_6(4,:)),'.y',...
%     real(Regime0_6(5,:)),imag(Regime0_6(5,:)),'.c',real(Regime0_6(6,:)),imag(Regime0_6(6,:)),'.m',...
%     real(Regime0_6(:,length(Regime0_6)/2)),imag(Regime0_6(:,length(Regime0_6)/2)),'xk',...
%     zeros(1,1000),linspace(-6,6,1000),'k')
% axis([-5 1 -4 4])
% ax = gca; ax.FontSize = 20;
% title('Sensitivity to v_r in Regime 0','FontSize',20)
% subplot(2,2,4)
% plot(real(Regime1_6(1,:)),imag(Regime1_6(1,:)),'.b',real(Regime1_6(2,:)),imag(Regime1_6(2,:)),'.r',...
%     real(Regime1_6(3,:)),imag(Regime1_6(3,:)),'.g',real(Regime1_6(4,:)),imag(Regime1_6(4,:)),'.y',...
%     real(Regime1_6(5,:)),imag(Regime1_6(5,:)),'.c',real(Regime1_6(6,:)),imag(Regime1_6(6,:)),'.m',...
%     real(Regime1_6(:,length(Regime1_6)/2)),imag(Regime1_6(:,length(Regime1_6)/2)),'xk',...
%     zeros(1,1000),linspace(-6,6,1000),'k')
% axis([-5 1 -4 4])
% ax = gca; ax.FontSize = 20;
% title('Sensitivity to v_r in Regime 1','FontSize',20)

%% Contour of stability of Regime 1-designed poles in Regime 0

i=0;
for v_r_0 = linspace(1/3.6,IDM_V0-1/3.6,100)
    i=i+1; j=0;
    for T_s = linspace(5,200,100)
        j=j+1;
        
        % Design of the control
        sigma = 4/T_s;
        omega = -pi*sigma/log(M_p);
        pDom = -sigma + 1i*omega;
        %pNeg1 = -5.1*sigma; pNeg2 = -5.2*sigma; pNeg3 = -5.3*sigma; pNeg4 = -5.4*sigma;
        %pNeg1 = -41*sigma; pNeg2 = -52*sigma; pNeg3 = -53*sigma; pNeg4 = -64*sigma;
        %pNeg1 = -5.1*sigma; pNeg2 = -22*sigma; pNeg3 = -33*sigma; pNeg4 = -44*sigma; %Mejor v_r máxima
        %pNeg1 = -5.1*sigma; pNeg2 = -12*sigma; pNeg3 = -33*sigma; pNeg4 = -50*sigma;
        %pNeg1 = -15*sigma; pNeg2 = -25*sigma; pNeg3 = -35*sigma; pNeg4 = -50*sigma;
        %pNeg1 = -11*sigma; pNeg2 = -12*sigma; pNeg3 = -13*sigma; pNeg4 = -14*sigma;
        pNeg1 = -21*sigma; pNeg2 = -22*sigma; pNeg3 = -23*sigma; pNeg4 = -24*sigma; %ANTERIOR
        %pNeg1 = -31*sigma; pNeg2 = -32*sigma; pNeg3 = -33*sigma;  pNeg4=-34*sigma; %Badly scaled
        %pNeg1 = -21*sigma; pNeg2 = -32*sigma; pNeg3 = -43*sigma; pNeg4 = -54*sigma;
        
        % Second order system
        P = [pDom; conj(pDom); pNeg1; pNeg2; pNeg3; pNeg4];
        % First order system
        %P = [-sigma; pNeg1; pNeg2; pNeg3; pNeg4; -25*sigma];
        
        % A_Justo_SCHFV_Control_Sensitivity - Regime 1 in order to design
        A31 = 2*IDM_Amax/(Tau_H*(IDM_H0+v_r_0*IDM_T))*(1-(v_r_0/IDM_V0)^IDM_Delta)^(3/2);
        A32 = -IDM_Amax/Tau_H*(IDM_Delta*v_r_0^(IDM_Delta-1)/IDM_V0^IDM_Delta+(v_r_0+2*IDM_T*sqrt(IDM_Amax*IDM_Bmax))/((IDM_H0+v_r_0*IDM_T)*sqrt(IDM_Amax*IDM_Bmax))*(1-(v_r_0/IDM_V0)^IDM_Delta));
        A34 = IDM_Amax*v_r_0/(Tau_H*((IDM_H0+v_r_0*IDM_T)*sqrt(IDM_Amax*IDM_Bmax)))*(1-(v_r_0/IDM_V0)^IDM_Delta);
        
        % A_Justo_SCHFV_Control_Sensitivity - Regime 0 and Regime 1
        A = [ 0    -1     -eta      1       0   ;...
              0     0       1       0       0   ;...
             A31   A32   -1/Tau_H  A34      0   ;...
              0     0       0       0       1   ;...
              0     0       0       0   -1/Tau_A];
        B = [0; 0; 0; 0; 1/Tau_A];
        C = [0 1 0 0 0];
        
        % Bigger matrix to also account for the kz gain
        A_6 = [A zeros(5,1); C 0];
        B_6 = [B; 0];
        
        K = place(A_6,B_6,P);
        %K = place(A,B,P);
        k1 = K(1); k2 = K(2); k3 = K(3); k4 = K(4); k5 = K(5); kz = K(6);
        %kz = -1/(C*((A-B*K)\B));
        K = [k1 k2 k3 k4 k5];
        
        % A_Justo_SCHFV_Control_Sensitivity - Regime 0 in order to check poles here
        A31 = 2*IDM_Amax/(Tau_H*IDM_H0)*(1-(v_r_0/IDM_V0)^IDM_Delta)^(3/2);
        A32 = -IDM_Amax/Tau_H*(IDM_Delta*v_r_0^(IDM_Delta-1)/IDM_V0^IDM_Delta);
        A34 = 0;
        p5 = 1/Tau_H + (k5+1)/Tau_A;
        p4 = (k4*Tau_H+k5+1)/(Tau_H*Tau_A) + A31*eta - A32;
        p3 = (k1*Tau_H + k4)/(Tau_H*Tau_A) + A31*(1+eta*(k5+1)/Tau_A) - A32*(k5+1)/Tau_A + A34*k3/Tau_A;
        p2 = k1/(Tau_H*Tau_A) + A31*(k3+eta*k4+k5+1)/Tau_A - A32*k4/Tau_A + A34*(k2-eta*k1)/Tau_A;
        p1 = A31*(k2+k4)/Tau_A - A32*k1/Tau_A + A34*(kz-k1)/Tau_A;
        p0 = A31*kz/Tau_A;
        num = [0 0 0 0 0 A34 A31]*kz/Tau_A;
        den = [1 p5 p4 p3 p2 p1 p0];
        P_LOW = sort(roots(den),'descend','ComparisonMethod','real');
        %P_LOW = sort(eig(A-B*K),'descend','ComparisonMethod','real');
        P_HIGH = sort(P,'descend','ComparisonMethod','real');
        M_save(i,j) = real(P_LOW(1));
    end
end

figure(2)
contourf(linspace(5,200,100),3.6*linspace(1/3.6,IDM_V0-1/3.6,100),M_save,[-1 0]);
ax = gca; ax.FontSize = 20;
xlabel('Settling time T_s [s]','FontSize',20);
ylabel('Reference speed v_r [km/h]','FontSize',20);
title(['Stability of Regime 0 with poles allocated for Regime 1, with M_p = ' num2str(M_p)],'FontSize',20)