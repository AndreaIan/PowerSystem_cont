%% Bifurcation analysis of the power system model, based on references:
% [1] J.H. Chow and A. Gebreselassie (1990) 
% "Dynamic voltage stability of a single machine constant power-load system," 
% In Proc 29th IEEE Conf. Decision Control, Honolulu HI, 3057-3062 
% 
% [2] I. Dobson, F.L. Alvarado and C.L. DeMarco (1992) 
% "Sensitivity of Hopf bifurcations to power system parameters," 
% In Proc. 31st IEEE Conf. Decision Control, Tucson AZ, 2928-2933 
% 
% and used to test the robust bifurcation margin algorithm in  
% [3] "Computation of bifurcation margins based on robust control concepts"
% (currently under review in SIAM Journal on Applied Dynamical Systems)


%
clc
clear

% The PATH where COCO has been downloaded must be added before performing the analyses

addCOCO;


% Parameter of the model. All are taken from [1], except where indicated:
T_q0_p=1.5; T_d0_p=5;
x_q=1; x_d=1; x_d_p=0.18;
x_T=0.15; x_e=0.3406;
T_A=0.4; K_A=30; T_f = 1.3; T_E = 0.56;

% The following values are from [2]
D = 0.05; k =0.1; 
PF = 0.95;
E_ref = 1.1;
K_f = 0.1; 


delta=0; % rotor angle (its value is uninfluential)


% % % % % Simulate the model at the first value of the bifurcation
% parameter in order to get a first guess for the state equilibrium
l_0= 0 ;

power_mod_simul = @(t,x)power_model(x,l_0,delta,T_q0_p,T_d0_p,x_q,x_d,x_d_p,x_T,x_e,T_A,K_A,T_f,T_E,K_f,D,k,PF,E_ref);


x0 = zeros(7,1); t0=0; tf=15;
x0(7)=0.1;
[t_sub,x_sub]=ode45(power_mod_simul,[t0,tf],x0); %%%%%%%%%    


x_fin=x_sub(end,:);

% COCO simulation

power_mod_coco = @(x, p)power_model(x, p,delta,T_q0_p,T_d0_p,x_q,x_d,x_d_p,x_T,x_e,T_A,K_A,T_f,T_E,K_f,D,k,PF,E_ref);

x0=x_fin;


pnames = {'l'}; % in the nominal test, the variable 'p' is only the speed
p0     = l_0;

prob = coco_prob();
prob = coco_set(prob, 'ep', 'NSA', true); % Detect and locate neutral saddles

ode_fcns = {power_mod_coco}; % vector field+derivatives

prob = coco_set(prob, 'ode', 'vectorized', false);
prob = ode_isol2ep(prob,'', ode_fcns{:}, x0, pnames, p0);
prob = coco_set(prob, 'cont', 'PtMX', 800);
bd1  = coco(prob, 'ep_test_power', [], 1, {'l'}, {[p0 2.5]});



figure(1); clf; hold on
thm = struct('special', {{'HB' 'SN' 'NSA'}});
coco_plot_bd(thm, 'ep_test_power', 'l', 'x'); hold on;



