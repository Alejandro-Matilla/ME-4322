% six bar linkage
%Static equilibrium

clc;
clear;

% define the joints
A = [7 4 0];
B = [5 16 0];
C = [25 25 0];
D = [23 10 0];
E = [18 35 0];
F = [43 32 0];
G = [45 17 0];

% define the lengths of the links
lAB = norm(B - A);
lBC = norm(C - B);
lCD = norm(D - C);
lDE = norm(E - D);
lEF = norm(F - E);
lFG = norm(G - F);

%weight of links
WAB = [0 -1 0];
WBEC = [0 -1 0];
WCD = [0 -1 0];
WED = [0 -1 0];
WEF = [0 -1 0];
WFG = [0 -1 0];

%center of mass of each link
S1 = (A+B)/2;
S2 = (B+C+E)/3;
S3 = (C+D)/2;
S4 = (E+F)/2;
S5 = (F+G)/2;

syms FAx FAy FBx FBy FCx FCy FDx FDy FEx FEy FFx FFy FGx FGy Tin

ForceA = [FAx FAy 0];
ForceB = [FBx FBy 0];
ForceC = [FCx FCy 0];
ForceD = [FDx FDy 0];
ForceE = [FEx FEy 0];
ForceF = [FFx FFy 0];
ForceG = [FGx FGy 0];
InputTorque = [ 0 0 Tin];

%Applied Force
AppliedForce = [50 0 0];
%Static Equilibrium Conditions for link AB
eqn1 = ForceA + ForceB + WAB == 0;

%Sum of moments = 0 with respect to CoM of Link AB
%S1A x FA + S1B x FB + InputTorque = 0

eqn2 = cross(A-S1, ForceA) + cross(B-S1, ForceB)+ InputTorque == 0;

%Equations for Link BEC

%Sum of Force = 0
% -Fb + fc + fe + wbec = 0

eqn3 = -ForceB + ForceC + ForceE + WBEC == 0;

%SUm of Moments equals 0
eqn4 = cross(B-S2, ForceB) + cross(C-S2, ForceC) + cross(E-S2, ForceE) == 0;


%Equations for Link CD
%Static Equilibrium Conditions for link CD
%Sum of Force = 0
eqn5 = -ForceC + ForceD + WCD == 0;


%Sum of Moments = 0
%Sum of Moments = 0 for Link CD
eqn6 = cross(C-S3, -ForceC) + cross(D-S3, ForceD) == 0;

%Equations for Link EF


%Sum of Force = 0 for Link EF
eqn7 = -ForceE + ForceF + WEF == 0;

%Sum of Moments = 0 for Link EF
eqn8 = cross(E-S4, -ForceE) + cross(F-S4, ForceF) == 0;


%Equations for Link FG


%Sum of Force = 0 for Link FG
eqn9 = -ForceF + ForceG + WFG + AppliedForce == 0;

%Sum of Moments = 0 for Link FG

eqn10 = cross(F-S5, -ForceF) + cross(G-S5, ForceG) == 0;


%Solving the 10 equations

eqnMatrix = [eqn1, eqn2, eqn3, eqn4, eqn5, eqn6, eqn7, eqn8, eqn9, eqn10];
staticSolution = solve(eqnMatrix, [FAx FAy FBx FBy FCx FCy FDx FDy FEx FEy FFx FFy FGx FGy Tin]);


Force_Ax = double(staticSolution.FAx);
Force_By = double(staticSolution.FBy);
Force_Cx = double(staticSolution.FCx);
Force_Cy = double(staticSolution.FCy);
Force_Dx = double(staticSolution.FDx);
Force_Dy = double(staticSolution.FDy);
Force_Ex = double(staticSolution.FEx);


% Angular velocities calculations
% Loop ABCDA

syms wBEC wCD 

omega_AB = [0 0 1];

omega_BEC = [0 0 wBEC];
omega_CD = [0 0 wCD];

eqn11 = cross(omega_AB, B-A) + cross(omega_BEC, C-B) + cross(omega_CD, D-C) == 0;

loop1Solution = solve(eqn11, [wBEC wCD]);

angularVelocity_BEC = double(loop1Solution.wBEC)
angularVelocity_CD = double(loop1Solution.wCD)


% Second loop DCEFGD

syms wEF wFG

omega_AB = [0 0 1];

omega_BEC = [0 0 angularVelocity_BEC];
omega_CD = [0 0 angularVelocity_CD];

omega_EF = [0 0 wEF];
omega_FG = [0 0 wFG];

eqn12 = cross(omega_CD,C-D) + cross(omega_BEC, E-C) + cross(omega_EF, F-E) + cross(omega_FG, G-F) == 0;

loop1Solution = solve(eqn12, [wEF wFG]);

angularVelocity_EF = double(loop1Solution.wEF)
angularVelocity_FG = double(loop1Solution.wFG)


% Angular Acceleration
% Loop 1 ABCDA

syms aBEC aCD

alpha_AB = [0 0 0]; % Assuming no angular acceleration for AB
alpha_BEC = [0 0 aBEC];
alpha_CD = [0 0 aCD];

a_B_A = cross(alpha_AB, B- A) + cross(omega_AB, cross(omega_AB, B-A));
a_C_B = cross(alpha_BEC, C-B) + cross(omega_BEC, cross(omega_BEC, C-B));
a_D_C = cross(alpha_CD, D-C) + cross(omega_CD, cross(omega_CD, D-C));

eqn13 = a_B_A + a_C_B + a_D_C == 0;

loop1AccSolution = solve(eqn13, [aBEC aCD]);

alphaBEC = double(loop1AccSolution.aBEC)
alphaCD = double(loop1AccSolution.aCD)


% Angular Acceleration for Loop 2 DCEFGD

syms aEF aFG

alpha_EF = [0 0 aEF];
alpha_FG = [0 0 aFG];

angVel_EF = [0 0 angularVelocity_EF];
anVel_FG = [0 0 angularVelocity_FG];

alpha__BEC = [0 0 alphaBEC];
alpha__CD = [0 0 alphaCD];


a_C_D = cross(alpha__CD, C-D) + cross(omega_CD, cross(omega_CD, C-D));
a_E_C = cross(alpha__BEC, E-C) + cross(omega_BEC, cross(omega_BEC, E-C));
a_F_E = cross(alpha_EF, F-E) + cross(angVel_EF, cross(angVel_EF, F-E));
a_G_F = cross(alpha_FG, G-F) + cross(anVel_FG, cross(anVel_FG, G-F));

eqn14 = a_C_D + a_E_C + a_F_E + a_G_F == 0;

loop2AccSolution = solve(eqn14, [aEF, aFG]);

alphaFG = double(loop2AccSolution.aFG)
alphaEF = double(loop2AccSolution.aEF)

% Velocity at Joint B

vB_A = cross(omega_AB, B-A);

% VE_A = V_E_B + V_B_A Joint E

v_E_B = cross(omega_BEC, E-B);

vE_A = v_E_B + vB_A;

%V_S4/G = V_S4_F + V_F_G

V_S4_F = cross(omega_EF, S4-F);

V_F_G = cross(omega_FG, F-G);

vS4_G = V_S4_F + V_F_G;

% Velocity of S1

vS1_A = cross(omega_AB, S1 - A);

% Velocity of S2

vS2_D = cross(omega_CD, S2 - D);

%Velocity of S3

vS3_G = cross(omega_FG, S3 - D);

%Velocity of S4

% VS5_A = VB_A + VS5_B

VS5_B = cross(omega_BEC, S5 - B);

VS5_A = vB_A + VS5_B;

%Velocity at Joint C

VC_D = cross(omega_CD, C -D);

%Velocity at Joint F

VF_G = cross(omega_FG, F - G);

% Acceleration at Joint B

aB_A = cross(alpha_AB, B-A) + cross(omega_AB, cross(omega_AB, B-A));


% Acceleration at Joint E

% aE_A = aE_B + aB_A

aE_B = cross(alpha_BEC, E-B) + cross(omega_BEC, cross(omega_BEC, E-B));

aE_A = aE_B + aB_A;


% Acceleration at Joint C

aC_D = cross(alpha_CD, C-D) + cross(omega_CD, cross(omega_CD, C-D));

% Acceleration at Joint F

aF_G = cross(alpha_FG, F-G) + cross(omega_FG, cross(omega_FG, F-G));

% Acceleration at S1

aS1_A = cross(alpha_AB, S1-A) + cross(omega_AB, cross(omega_AB, S1 - A));

% Acceleration at S2

aS2_D = cross(alpha_CD, S2 - D) + cross(omega_CD, cross(omega_CD, S2 - D));

% Acceleration at S3

aS3_G = cross(alpha_FG, S3 - G) + cross(omega_FG, cross(omega_FG, S3 - G));

% Acceleration at S4
% aS4_G = aF_G + aS4_F

aS4_F = cross(alpha_EF, S4 - F) + cross(omega_EF, cross(omega_EF, S4 - F));
aS4_G = aF_G + aS4_F;

% Acceleration at S5
% aS5_A = aB_A + aS5_B

aS5_B = cross(alpha__BEC, S5 - B) + cross(omega_BEC, cross(omega_BEC, S5 - B));
aS5_A = aB_A + aS5_B