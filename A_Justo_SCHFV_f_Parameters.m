function [MITSIM_a_Free_thr_low,MITSIM_a_Free_thr_mid,...
    MITSIM_a_Free_thr_high,MITSIM_a_Free_bra_low,...
    MITSIM_a_Free_bra_midlow,MITSIM_a_Free_bra_mid,...
    MITSIM_a_Free_bra_midhigh,MITSIM_a_Free_bra_high,MITSIM_Alpha_thr,...
    MITSIM_Alpha_bra,MITSIM_Beta_thr,MITSIM_Beta_bra,MITSIM_Gamma_thr,...
    MITSIM_Gamma_bra,MITSIM_T_upper,MITSIM_T_lower,IDM_Amax,IDM_Bmax,...
    IDM_H0,IDM_V0,IDM_Delta,IDM_iSqab] = A_Justo_SCHFV_f_Parameters(N)

% For reproducibility
rng default;

% MITSIM parameter: free acceleration - low
MITSIM_a_free_thr_low=4.8; % 4.8 by Yang
MITSIM_a_free_thr_lows=abs(0.3*MITSIM_a_free_thr_low);
% MITSIM parameter: free acceleration - mid
MITSIM_a_free_thr_mid=4.8; % 6.7 by Yang
MITSIM_a_free_thr_mids=abs(0.3*MITSIM_a_free_thr_mid);
% MITSIM parameter: free acceleration - high
MITSIM_a_free_thr_high=4.8; % 7.8 by Yang
MITSIM_a_free_thr_highs=abs(0.3*MITSIM_a_free_thr_high);
% MITSIM parameter: free deceleration - low
MITSIM_a_free_bra_low=-8.7; % -2.0 by Yang
MITSIM_a_free_bra_lows=abs(0.3*MITSIM_a_free_bra_low);
% MITSIM parameter: free deceleration - midlow
MITSIM_a_free_bra_midlow=-8.7; % -2.9 by Yang
MITSIM_a_free_bra_midlows=abs(0.3*MITSIM_a_free_bra_midlow);
% MITSIM parameter: free deceleration - mid
MITSIM_a_free_bra_mid=-8.7; % -4.4 by Yang
MITSIM_a_free_bra_mids=abs(0.3*MITSIM_a_free_bra_mid);
% MITSIM parameter: free deceleration - midhigh
MITSIM_a_free_bra_midhigh=-8.7; % -5.2 by Yang
MITSIM_a_free_bra_midhighs=abs(0.3*MITSIM_a_free_bra_midhigh);
% MITSIM parameter: free deceleration - high
MITSIM_a_free_bra_high=-8.7; % -8.7 by Yang
MITSIM_a_free_bra_highs=abs(0.3*MITSIM_a_free_bra_high);
% MITSIM parameter: alpha (acceleration)
MITSIM_alpha_thr=2.15;
MITSIM_alpha_thrs=abs(0.3*MITSIM_alpha_thr);
% MITSIM parameter: alpha (deceleration)
MITSIM_alpha_bra=1.55;
MITSIM_alpha_bras=abs(0.3*MITSIM_alpha_bra);
% MITSIM parameter: beta (acceleration)
MITSIM_beta_thr=-1.67;
MITSIM_beta_thrs=abs(0.3*MITSIM_beta_thr);
% MITSIM parameter: beta (deceleration)
MITSIM_beta_bra=1.08;
MITSIM_beta_bras=abs(0.3*MITSIM_beta_bra);
% MITSIM parameter: gamma (acceleration)
MITSIM_gamma_thr=-0.89;
MITSIM_gamma_thrs=abs(0.3*MITSIM_gamma_thr);
% MITSIM parameter: gamma (deceleration)
MITSIM_gamma_bra=1.65;
MITSIM_gamma_bras=abs(0.3*MITSIM_gamma_bra);
% MITSIM parameter: upper threshold for following time headway
MITSIM_t_upper=1.36; %1.36 by Yang
MITSIM_t_uppers=abs(0.3*MITSIM_t_upper);
% MITSIM parameter: lower threshold for following time headway
MITSIM_t_lower=0.50; %0.50 by Yang
MITSIM_t_lowers=abs(0.3*MITSIM_t_lower);

% IDM parameter: Maximum acceleration [m/s^2]
IDM_amax=1;
IDM_amaxs=0.2;
% IDM parameter: Maximum deceleration [m/s^2]
IDM_bmax=3.5;
IDM_bmaxs=0.2;
% IDM parameter: Minimum net headway distance [m]
IDM_h0=2;
IDM_h0s=0.2;
% IDM parameter: Desired freeroad speed [m/s]
IDM_v0=40/3.6;
IDM_v0s=10/3.6;
% IDM parameter: Acceleration exponent [-]
IDM_delta=0.4;
IDM_deltas=0.0;

% Generation of vectors and matrices with a Gaussian distribution
for i=1:N
    % MITSIM parameters
    MITSIM_a_Free_thr_low(i,i)=normrnd(MITSIM_a_free_thr_low,MITSIM_a_free_thr_lows);
    MITSIM_a_Free_thr_mid(i,i)=normrnd(MITSIM_a_free_thr_mid,MITSIM_a_free_thr_mids);
    MITSIM_a_Free_thr_high(i,i)=normrnd(MITSIM_a_free_thr_high,MITSIM_a_free_thr_highs);
    MITSIM_a_Free_bra_low(i,i)=normrnd(MITSIM_a_free_bra_low,MITSIM_a_free_bra_lows);
    MITSIM_a_Free_bra_midlow(i,i)=normrnd(MITSIM_a_free_bra_midlow,MITSIM_a_free_bra_midlows);
    MITSIM_a_Free_bra_mid(i,i)=normrnd(MITSIM_a_free_bra_mid,MITSIM_a_free_bra_mids);
    MITSIM_a_Free_bra_midhigh(i,i)=normrnd(MITSIM_a_free_bra_midhigh,MITSIM_a_free_bra_midhighs);
    MITSIM_a_Free_bra_high(i,i)=normrnd(MITSIM_a_free_bra_high,MITSIM_a_free_bra_highs);
    MITSIM_Alpha_thr(i,i)=normrnd(MITSIM_alpha_thr,MITSIM_alpha_thrs);
    MITSIM_Alpha_bra(i,i)=normrnd(MITSIM_alpha_bra,MITSIM_alpha_bras);
    MITSIM_Beta_thr(i)=normrnd(MITSIM_beta_thr,MITSIM_beta_thrs);
    MITSIM_Beta_bra(i)=normrnd(MITSIM_beta_bra,MITSIM_beta_bras);
    MITSIM_Gamma_thr(i)=normrnd(MITSIM_gamma_thr,MITSIM_gamma_thrs);
    MITSIM_Gamma_bra(i)=normrnd(MITSIM_gamma_bra,MITSIM_gamma_bras);
    MITSIM_T_upper(i)=normrnd(MITSIM_t_upper,MITSIM_t_uppers);
    MITSIM_T_lower(i)=normrnd(MITSIM_t_lower,MITSIM_t_lowers);
    % IDM parameters
    IDM_Amax(i,i)=normrnd(IDM_amax,IDM_amaxs);
    IDM_Bmax(i,i)=normrnd(IDM_bmax,IDM_bmaxs);
    IDM_H0(i)=normrnd(IDM_h0,IDM_h0s);
    IDM_V0(i)=normrnd(IDM_v0,IDM_v0s);
    IDM_Delta(i)=normrnd(IDM_delta,IDM_deltas);
    IDM_iSqab(i)=1/(2*sqrt(IDM_Amax(i,i)*IDM_Bmax(i,i)));
end
end