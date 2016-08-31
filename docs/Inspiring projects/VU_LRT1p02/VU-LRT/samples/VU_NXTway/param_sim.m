% Simulation and Virtual Reality Parameters

% Initial Values
PSI0 = 5 * pi / 180;				% initial value of body angle
X1IV = [							% x1 initial value
	0
	PSI0
	0
	0
	];
X2IV = [							% x2 initial value
	0
	0
	];

% Sample Rates
TS = 0.001;							% base sample rate [sec]
% cut sonar and wall hit simulation steps for accelerating simulation.
TS_SONAR = ts2;						% sonar calculation sample rate [sec]
TS_WALL = ts1 * 5;					% wall hit calculation sample rate [sec]

% Start Time of balancing
TIME_START = time_start / 1000;     % start time of stabilizing [sec] (= gyro calibration time)

% Environments
GP_MAX = 100;						% maximum game pad input
GYRO0 = 600;						% initial value of gyro sensor
BATTERY = 8000;						% initial value of battery [mV]

% MAP
WALL_COLOR = 128;									% gray
MAP_WALL = ~(imread('track.bmp') - WALL_COLOR);		% wall = 1, field = 0
MAP_WALL = MAP_WALL(:, :, 1);
START_POS = [100, 100];				% initial position ([z, x]) [cm]

% Conditions for Stopping Simulation
BODY_ANGLE_MAX = 15;				% maximum body angle
BODY_ANGLE_MIN = -15;				% minimum body angle

% Virtual Reality Parameters
TS_VR = 0.1;						% VRML refresh rate [sec]
BODY_HEIGHT = 13.2;					% NXTway body height [cm]				
CAMERA_HEIGHT = 13.2;				% camera height [cm]
CAMERA_DISTANCE = 50;				% distance between NXTway and camera [cm]
CAMERA_OFFSET = -8;					% z offset of Chaser View Point [cm]
ROTATION_X = [1, 0, 0];				% rotation vector around x axis
ROTATION_Y = [0, 1, 0];				% rotation vector around y axis

clear START_PHI START_PSI WALL_COLOR
