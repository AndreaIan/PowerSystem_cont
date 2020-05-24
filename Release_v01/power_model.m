function f = power_model(x, p,delta,T_q0_p,T_d0_p,x_q,x_d,x_d_p,x_T,x_e,T_A,K_A,T_f,T_E,K_f,D,k,PF,E_ref)

% In this routine, the power system model originally proposed in the two references:
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


% This model was originally proposed in [1], and then augmented with a dynamic load model in [2].
% See [3] % for a complete genesis of the model.
% Since the model is a blended version of information given in [1] and [2], and new derivations from [3],
% the origin of the equations will be highlighted in the code.
% The same nomenclature used in the references above is used


% Definition of the bifurcation parameter for continuation analyses
l = p(1); % loading parameter

% States numbering:
% x(1)= E_d_p
% x(2)= E_q_p

% x(3)= V_R
% x(4)= E_FD
% x(5)= R_f

% x(6)= theta
% x(7)= V_L


 x_E=x_d_p+x_T+x_e; % from [1] 



I_d = 1/x_E * (x(2,1)-x(7,1)*cos(delta- x(6,1))); % from [1]
I_q= 1/x_E * (-x(1,1)+x(7,1)*sin(delta- x(6,1))); % from [1]

f(1,1) = 1/T_q0_p * (-x(1,1)+(x_q-x_d_p)*I_q); % from [1]
f(2,1) = 1/T_d0_p * (-x(2,1)-(x_d-x_d_p)*I_d+x(4,1)); % from [1]



P_tilde = -x(1,1)*cos(delta)+x(2,1)*sin(delta)-x(7,1)*sin(x(6,1)); % from [3]
Q_tilde = x(1,1)*sin(delta)+x(2,1)*cos(delta)-x(7,1)*cos(x(6,1)); % from [3]

P_L = x(7,1)/x_E * ( cos(x(6,1)) * P_tilde - sin(x(6,1)) * Q_tilde ); % from [3]

Q_L = x(7,1)/x_E * ( sin(x(6,1)) * P_tilde + cos(x(6,1)) * Q_tilde ); % from [3]


 E_s= 1/x(7) * sqrt( (x_e*P_L)^2 + (x_e*Q_L+x(7)^2)^2 ); % from [3]



f(3,1) = -1/T_A * x(3,1) +K_A/T_A*(E_ref-E_s-K_f*x(4,1)/T_f+x(5,1));% from [1]
f(4,1) = -1/T_E * x(4,1) +1/T_E*x(3,1);% from [1]
f(5,1) = -1/T_f * x(5,1) +K_f*x(4,1)/T_f^2;% from [1]


f(6,1) = 1/D * (P_L-l*PF); % from [2]
f(7,1) = 1/k * (Q_L-l*sqrt(1-PF^2)); % from [2]


end






