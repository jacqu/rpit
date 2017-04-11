function [sys,x0,str,ts] = camera_anim(t,x,u,flag)
%CAMERA_ANIM S-function for animating the live image of an eye in hand cam.
%

%   The 8 components of the vector u are the coordintaes of 4 points.
%
%   Copyright 2012 Jacques Gangloff (jacques.gangloff@unistra.fr)
%   $Revision: 0.1 $

% Plots every major integration step, but has no states of its own

switch flag,

  %%%%%%%%%%%%%%%%%%
  % Initialization %
  %%%%%%%%%%%%%%%%%%
  case 0,
    [sys,x0,str,ts]=mdlInitializeSizes( );

  %%%%%%%%%%
  % Update %
  %%%%%%%%%%
  case 2,
    sys=mdlUpdate(t,x,u);

  %%%%%%%%%%%%%%%%
  % Unused flags %
  %%%%%%%%%%%%%%%%
  case { 1, 3, 4, 9 },
    sys = [];
    
  %%%%%%%%%%%%%%%
  % DeleteBlock %
  %%%%%%%%%%%%%%%
  case 'DeleteBlock',
    LocalDeleteBlock
    
  %%%%%%%%%%%%%%%
  % DeleteFigure %
  %%%%%%%%%%%%%%%
  case 'DeleteFigure',
    LocalDeleteFigure
  
  %%%%%%%%%
  % Close %
  %%%%%%%%%
  case 'Close',
    LocalClose
  
  %%%%%%%%%%%%
  % Playback %
  %%%%%%%%%%%%
  case 'Playback',
    LocalPlayback
   
  %%%%%%%%%%%%%%%%%%%%
  % Unexpected flags %
  %%%%%%%%%%%%%%%%%%%%
  otherwise
    error(['Unhandled flag = ',num2str(flag)]);
end

% end camera_anim

%
%=============================================================================
% mdlInitializeSizes
% Return the sizes, initial conditions, and sample times for the S-function.
%=============================================================================
%
function [sys,x0,str,ts]=mdlInitializeSizes( )

global Sampling;

%
% Call simsizes for a sizes structure, fill it in and convert it to a
% sizes array.
%
sizes = simsizes;

sizes.NumContStates  = 0;
sizes.NumDiscStates  = 0;
sizes.NumOutputs     = 0;
sizes.NumInputs      = 10;
sizes.DirFeedthrough = 1;
sizes.NumSampleTimes = 1;

sys = simsizes(sizes);

%
% Initialize the initial conditions
%
x0  = [];

%
% str is always an empty matrix
%
str = [];

%
% Initialize the array of sample times, for the simulation,
% the animation is updated every "Sampling" seconds
%
ts  = [Sampling 0];

%
% create the figure, if necessary
%
LocalCameraInit( );

% end mdlInitializeSizes

%
%=============================================================================
% mdlUpdate
% Update the Camera animation.
%=============================================================================
%
function sys=mdlUpdate(t,x,u) %#ok<INUSL>

global CameraPlayData;

fig = get_param(gcbh,'UserData');
if ishandle(fig),
  if strcmp(get(fig,'Visible'),'on'),
    ud = get(fig,'UserData');
    LocalCameraSets(t,ud,u);
    CameraPlayData.t(CameraPlayData.i) = t;
    CameraPlayData.u(CameraPlayData.i).u = u;
    CameraPlayData.i = CameraPlayData.i + 1;
  end
end;
 
sys = [];

% end mdlUpdate

%
%=============================================================================
% LocalDeleteBlock
% The animation block is being deleted, delete the associated figure.
%=============================================================================
%
function LocalDeleteBlock

fig = get_param(gcbh,'UserData');
if ishandle(fig),
  delete(fig);
end

% end LocalDeleteBlock

%
%=============================================================================
% LocalDeleteFigure
% The animation figure is being deleted.
%=============================================================================
%
function LocalDeleteFigure

return
  
% end LocalDeleteFigure


%
%=============================================================================
% LocalClose
% The callback function for the animation window close button.  Delete
% the animation figure window.
%=============================================================================
%
function LocalClose

delete(gcbf)

% end LocalClose

%
%=============================================================================
% LocalPlayback
% The callback function for the animation window playback button.  Playback
% the animation.
%=============================================================================
%
function LocalPlayback

global CameraPlayData;
global Sampling;

try
  ud = get(gcbf,'UserData');
  for i=1:CameraPlayData.i - 1,
    tic;
    LocalCameraSets(CameraPlayData.t(i),ud,CameraPlayData.u(i).u);
    PlayDuration = toc;
    
    % Slow down playback to be as realtime as possible on fast machines
    if ( PlayDuration ) < 0.1 * Sampling
        pause( Sampling );
    end
  end
catch %#ok<CTCH>
    % do nothing
end

% end LocalPlayback

%
%=============================================================================
% LocalCameraSets
% Local function to set the position of the graphics objects in the
% Camera animation window.
%=============================================================================
%
function LocalCameraSets(time,ud,u)

global Image_y;

% Define the measurement

Mes_x = [ u(1) NaN u(3) NaN u(5) NaN u(7) ];
Mes_y = [ Image_y - u(2) NaN Image_y - u(4) NaN Image_y - u(6) NaN Image_y - u(8) ];

% Update measurement

set( ud.Camera_mes,...
  'XData',    Mes_x,...
  'YData',    Mes_y,...
  'ZData',    [ 0 NaN 0 NaN 0 NaN 0 ] );

% Redraw figure

pause(0)
drawnow

% end LocalCameraSets

%
%=============================================================================
% LocalCameraInit
% Local function to initialize the Camera animation.  If the animation
% window already exists, it is brought to the front.  Otherwise, a new
% figure window is created.
%=============================================================================
%
function LocalCameraInit( )

global Image_x Image_y;
global Ref_x Ref_y;

%
% The name of the reference is derived from the name of the
% subsystem block that owns the Camera animation S-function block.
% This subsystem is the current system and is assumed to be the same
% layer at which the reference block resides.
%
sys = get_param(gcs,'Parent');

%
% The animation figure handle is stored in the Camera block's UserData.
% If it exists, initialize the reference mark, time, cart, and Camera
% positions/strings/etc.
%
Fig = get_param(gcbh,'UserData');
if ishandle(Fig),
  FigUD = get(Fig,'UserData');
  figure(Fig);
  return
end

%
% The animation figure doesn't exist, create a new one and store its
% handle in the animation block's UserData
%
FigureName = 'Camera Visualization';
Fig = figure(...
  'Units',           'pixel',...
  'Position',        [ 1 1 Image_x Image_y+50 ],...
  'Name',            FigureName,...
  'NumberTitle',     'off',...
  'IntegerHandle',   'off',...
  'HandleVisibility','callback',...
  'Resize',          'off',...
  'Renderer',        'painters',...
  'DeleteFcn',       'camera_anim([],[],[],''DeleteFigure'')',...
  'CloseRequestFcn', 'camera_anim([],[],[],''Close'');');

% Axes limits correspond to the limit of the Camera frame
AxesH = axes(...
  'Parent',             Fig,...
  'Units',              'pixel',...
  'Position',           [ 1 50 Image_x Image_y ],...
  'CLim',               [ 1 64 ], ...
  'Xlim',               [ 0 Image_x ],...
  'Ylim',               [ 0 Image_y ],...
  'Zlim',               [ -1 1 ],...
  'CameraPosition',     [ Image_x / 2 Image_y / 2 1 ],...
  'CameraTarget',       [ Image_x / 2 Image_y / 2 0 ],...
  'DataAspectRatio',    [ 1 1 1 ],...
  'Visible',    'off',...
  'Projection', 'orthographic');

% Animation initialization

% Draw the measurement
Camera_mes = line(...
  'Parent',   AxesH,...
  'XData',    Ref_x,...
  'YData',    Ref_y,...
  'ZData',    [ 0 NaN 0 NaN 0 NaN 0 ],...
  'LineStyle','none',...
  'Marker',   'X',...
  'LineWidth',30,...
  'Color',    'r' );

% Draw the play button

uicontrol(...
  'Parent',  Fig,...
  'Style',   'pushbutton',...
  'Position',[ 10 10 Image_x-20 30 ],...
  'String',  'Playback', ...
  'Callback','camera_anim([],[],[],''Playback'');',...
  'Interruptible','off',...
  'BusyAction','cancel');

%
% all the HG objects are created, store them into the Figure's UserData
%

FigUD.Camera_mes          = Camera_mes;
FigUD.Block               = get_param(gcbh,'Handle');
set(Fig,'UserData',FigUD);

drawnow
cameratoolbar(Fig, 'Show');

%
% store the figure handle in the animation block's UserData
%

set_param(gcbh,'UserData',Fig);

% end LocalCameraInit
