% Sampling period

global Sampling;
Sampling = 0.01;

% Replay data buffer

global CameraPlayData;
CameraPlayData.t(1) = 0;
CameraPlayData.u(1).u = [ 0 0 0 ];
CameraPlayData.i = 1;

% Image size (pixels)

global Image_x Image_y;
Image_x = 640;
Image_y = 480;

global Ref_x Ref_y;
Ref_x = [ Image_x / 4 NaN Image_x / 4 NaN 3 * Image_x / 4 NaN 3 * Image_x / 4 ];
Ref_y = [ Image_y / 4 NaN 3 * Image_y / 4 NaN Image_y / 4 NaN 3 * Image_y / 4 ];

