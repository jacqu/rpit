% NXTway-GS Parameters and State-Space Matrix Calculation

% Physical Constant
g = 9.81;						% gravity acceleration [m/sec^2]

% NXTway-GS Parameters
m = 0.03;						% wheel weight [kg]
R = 0.04;						% wheel radius [m]
Jw = m * R^2 / 2;				% wheel inertia moment [kgm^2]
M = 0.6;						% body weight [kg]
W = 0.14;						% body width [m]
D = 0.04;						% body depth [m]
H = 0.144;						% body height [m]
L = H / 2;						% distance of the center of mass from the wheel axle [m]
Jpsi = M * L^2 / 3;				% body pitch inertia moment [kgm^2]
Jphi = M * (W^2 + D^2) / 12;	% body yaw inertia moment [kgm^2]
fm = 0.0022;					% friction coefficient between body & DC motor
fw = 0;							% friction coefficient between wheel & floor

% DC Motor Parameters			
Jm = 1e-5;						% DC motor inertia moment [kgm^2]
Rm = 6.69;						% DC motor resistance [ƒ¶]
Kb = 0.468;						% DC motor back EMF constant [Vsec/rad]
Kt = 0.317;						% DC motor torque constant [Nm/A]
n = 1;							% gear ratio

% NXTway-GS State-Space Matrix Calculation
alpha = n * Kt / Rm;
beta = n * Kt * Kb / Rm + fm;
tmp = beta + fw;

E_11 = (2 * m + M) * R^2 + 2 * Jw + 2 * n^2 * Jm;
E_12 = M * L * R - 2 * n^2 * Jm;
E_22 = M * L^2 + Jpsi + 2 * n^2 * Jm;
detE = E_11 * E_22 - E_12^2;

A1_32 = -g * M * L * E_12 / detE;
A1_42 = g * M * L * E_11 / detE;
A1_33 = -2 * (tmp * E_22 + beta * E_12) / detE;
A1_43 = 2 * (tmp * E_12 + beta * E_11) / detE;
A1_34 = 2 * beta * (E_22 + E_12) / detE;
A1_44 = -2 * beta * (E_11 + E_12) / detE;
B1_3 = alpha * (E_22 + E_12) / detE;
B1_4 = -alpha * (E_11 + E_12) / detE;
A1 = [
	0 0 1 0
	0 0 0 1
	0 A1_32 A1_33 A1_34
	0 A1_42 A1_43 A1_44
	];
B1 = [
	0 0
	0 0
	B1_3 B1_3
	B1_4 B1_4
	];
C1 = eye(4);
D1 = zeros(4, 2);

I = m * W^2 / 2 + Jphi + (Jw + n^2 * Jm) * W^2 / (2 * R^2);
J = tmp * W^2 / (2 * R^2);
K = alpha * W / (2 * R);
A2 = [
	0 1
	0 -J / I
	];
B2 = [
	0      0
	-K / I K / I
	];
C2 = eye(2);
D2 = zeros(2);

clear alpha beta tmp
clear E_11 E_12 E_22 detE
clear A1_32 A1_33 A1_34 A1_42 A1_43 A1_44 B1_3 B1_4 I J K
