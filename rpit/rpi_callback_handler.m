function rpi_callback_handler(hDlg, hSrc)
	
	% Initialize call to get the root directory of the target install.
	% Function called during compilation process.
  set_param( bdroot,'PostCodeGenCommand','postGenFunc(buildInfo)' );
  
  % Auto-configure some critical simulation parameters
  slConfigUISetVal( hDlg, hSrc, 'StopTime', 'Inf' );									% Process runs forever
  slConfigUISetVal( hDlg, hSrc, 'SolverType', 'Fixed-step' );					% Fixed step solver
  slConfigUISetVal( hDlg, hSrc, 'Solver', 'FixedStepDiscrete' );			% Discrete system
  slConfigUISetVal( hDlg, hSrc, 'InlineParams', 'off' );							% Params can be changed on the fly
  slConfigUISetEnabled( hDlg, hSrc, 'InlineParams', 0 );							% Force no inline params
  slConfigUISetVal( hDlg, hSrc, 'ProdHWDeviceType', 'Intel->x86/Pentium' );	
  																																		% x86 hardware
  slConfigUISetVal( hDlg, hSrc.Components, 'SupportContinuousTime', 'on' );
                                                                      % Allow continuous blocks
  slConfigUISetVal( hDlg, hSrc.Components, 'ExtMode', 'on' );					% Force external mode
  slConfigUISetEnabled( hDlg, hSrc.Components, 'ExtMode', 0 );				% Don't allow disabling
  slConfigUISetVal( hDlg, hSrc.Components, 'ExtModeTransport', 0 );		% Select tcpip transport
  slConfigUISetEnabled( hDlg, hSrc.Components, 'ExtModeTransport', 0 );
  																																		% Other tranports disabled
  slConfigUISetVal( hDlg, hSrc.Components, 'ERTCustomFileTemplate', 'ert_rpi_file_process.tlc' );
  																																		% Set custom file template
  slConfigUISetEnabled( hDlg, hSrc.Components, 'ERTCustomFileTemplate', 0 );
  																																		% Disable change of file template
  slConfigUISetVal( hDlg, hSrc.Components, 'GenerateSampleERTMain', 'off' );
  																																		% Do not generate sample ert_main.c
  slConfigUISetEnabled( hDlg, hSrc.Components, 'GenerateSampleERTMain', 0 );
  																																		% Don't allow activating this flag
