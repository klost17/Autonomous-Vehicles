IDM_Amax = IDM_Amax(acc1+n1,acc1+n1);
IDM_Bmax = IDM_Bmax(acc1+n1,acc1+n1);
IDM_Delta = IDM_Delta(acc1+n1);
Tau_H = Tau(acc1+n1,acc1+n1);
Tau_A = Tau(acc1,acc1);
eta = IDM_T(acc1+n1,acc1+n1);
IDM_T = IDM_T(acc1+n1,acc1+n1);
IDM_H0 = IDM_H0(acc1+n1);
IDM_V0 = IDM_V0(acc1+n1);
%%
clc

% Linearized system
A31 = 2*IDM_Amax/Tau_H/(IDM_H0+v_r_0*IDM_T)*(1-(v_r_0/IDM_V0)^IDM_Delta)^(3/2);
A32 = -IDM_Amax/Tau_H*(IDM_Delta*v_r_0^(IDM_Delta-1)/IDM_V0^IDM_Delta+(v_r_0+2*IDM_T*sqrt(IDM_Amax*IDM_Bmax))/((IDM_H0+v_r_0*IDM_T)*sqrt(IDM_Amax*IDM_Bmax))*(1-(v_r_0/IDM_V0)^IDM_Delta));
A34 = IDM_Amax/Tau_H*v_r_0/((IDM_H0+v_r_0*IDM_T)*sqrt(IDM_Amax*IDM_Bmax))*(1-(v_r_0/IDM_V0)^IDM_Delta);
A = [ 0    -1     -eta      1       0    ;...
      0     0       1       0       0    ;...
     A31   A32  -1/Tau_H   A34      0    ;...
      0     0       0       0       1    ;...
      0     0       0       0   -1/Tau_A ];
B = [0; 0; 0; 0; 1/Tau_A];
C = [0 1 0 0 0];

[NUMprop,DENprop] = ss2tf(A,B,C,0);
G = tf(NUMprop,DENprop);
figure(1)
rlocus(G)
ax = gca; ax.FontSize = 20;
title('Root Locus','FontSize',20)
xlabel('Real axis','FontSize',20);
ylabel('Imaginary axis','FontSize',20);
[NUMprop,DENprop] = ss2tf(A-kgain*B*C,kgain*B,C,0);
POLES = sort(roots(DENprop),'descend','ComparisonMethod','real')
sigma = abs(real(POLES(1))); omega = abs(imag(POLES(1)));
disp(['T_s = ' num2str(4/sigma) 's; M_p = ' num2str(exp(-pi*sigma/omega))])

% % Closed loop (proportional controller)
% p4 = 1/Tau_H + 1/Tau_A;
% p3 = 1/(Tau_H*Tau_A) + A31*eta - A32;
% p2 = A31*(1+eta/Tau_A) - A32/Tau_A;
% p1 = A31/Tau_A + A34*kgain/Tau_A;
% p0 = A31*kgain/Tau_A;
% num2 = [0 0 0 0 A34 A31]*kgain/Tau_A
% den2 = [1 p4 p3 p2 p1 p0]

% % Open loop
% p4 = 1/Tau_H + 1/Tau_A;
% p3 = 1/(Tau_H*Tau_A) + A31*eta - A32;
% p2 = A31*(1+eta/Tau_A) - A32/Tau_A;
% p1 = A31/Tau_A;
% p0 = 0;
% num2 = [0 0 0 0 A34 A31]/Tau_A
% den2 = [1 p4 p3 p2 p1 p0]