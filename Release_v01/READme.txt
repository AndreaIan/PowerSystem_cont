%Release v1_0 - 25/05/2020
% Author: Andrea Iannelli (ETH Zürich)
% contact: iannelli@control.ee.ethz.ch
%------------------------------------------------------------------

The MATLAB files in this folder present an implementation of the vector field of
the power system model originally proposed in the two references:
[1] J.H. Chow and A. Gebreselassie (1990) 
"Dynamic voltage stability of a single machine constant power-load system," 
In Proc 29th IEEE Conf. Decision Control, Honolulu HI, 3057-3062 

[2] I. Dobson, F.L. Alvarado and C.L. DeMarco (1992) 
"Sensitivity of Hopf bifurcations to power system parameters," 
In Proc. 31st IEEE Conf. Decision Control, Tucson AZ, 2928-2933 

and used to test the robust bifurcation margin algorithm in the paper 
[3] A. Iannelli, M. Lowenberg, A. Marcos (2019) 
"Computation of bifurcation margins based on robust control concepts"
(currently under review in SIAM Journal on Applied Dynamical Systems).

  
The repository consists of 2 files:
- power_model.m is a MATLAB function that implements the vector field. 
This can be used for example as input to: ode45 to simulate the dynamics;
COCO to perform numerical continuation.
-Coco_analysis.m is a script that defines the system's parameters, and perform 
numerical continuation analysis by initializing it with the steady-state 
equilibrium obtained via simulation. This file can thus be used to generate all
the nominal analyses of the power system model presented in [3]



NOTE: the continuation solver COCO from 
https://sourceforge.net/p/cocotools/wiki/Home/
must be loaded in order to perform the analyses.

