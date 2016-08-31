% Fixed-Point Parameters
% We can calculate how far NXTway-GS can move by using the following equation
% >> double(intmax('int32')) * pi / 180 * R * S
% where R is the wheel radius and S is the slope of dt_theta (S = 2^-14 etc.)
%
%	  S		:	Range [m]
%	2^-10	:	1464.1
%	2^-14	:	  91.5
%	2^-18	:	   5.7
%	2^-22	:	   0.36

% Simulink.NumericType for wheel angle
% signed 32-bit integer, slope = 2^-14, bias = 0
dt_theta = fixdt(true, 32, 2^-14, 0);

% Simulink.NumericType for body pitch angle
% signed 32-bit integer, slope = 2^-20, bias = 0
dt_psi = fixdt(true, 32, 2^-20, 0);

% Simulink.NumericType for battery
% signed 32-bit integer, slope = 2^-17, bias = 0
dt_battery = fixdt(true, 32, 2^-17, 0);
