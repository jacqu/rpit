function qnx_callback_handler(hDlg, hSrc)
  set_param(bdroot,'PostCodeGenCommand','postGenFunc(buildInfo)');
  curVal=slConfigUIGetVal(hDlg, hSrc, 'ModelReferenceCompliant');
  slConfigUISetVal(hDlg, hSrc, 'ModelReferenceCompliant', 'on');
  curVal=slConfigUIGetVal(hDlg, hSrc, 'ModelReferenceCompliant');
  slConfigUISetEnabled(hDlg, hSrc, 'ModelReferenceCompliant', false);
  
  slConfigUISetVal(hDlg, hSrc, 'CompOptLevelCompliant', 'on');
  slConfigUISetEnabled(hDlg, hSrc, 'CompOptLevelCompliant', false);
  
  % Setup these options as desired and gray them out
%   slConfigUISetVal(hDlg, hSrc, 'ProdHWDeviceType', 'Motorola HC12');
%   slConfigUISetEnabled(hDlg, hSrc, 'ProdHWDeviceType', 0);
%   slConfigUISetVal(hDlg, hSrc, 'ProdEqTarget','on');
%   slConfigUISetEnabled(hDlg, hSrc, 'ProdEqTarget', 0);

  slConfigUISetVal(hDlg, hSrc, 'ZeroExternalMemoryAtStartup','off');
  slConfigUISetVal(hDlg, hSrc, 'ZeroInternalMemoryAtStartup','off');

  slConfigUISetVal(hDlg, hSrc, 'NoFixptDivByZeroProtection', 'on');
  slConfigUISetVal(hDlg, hSrc, 'EfficientFloat2IntCast', 'on');

  slConfigUISetVal(hDlg, hSrc, 'GenerateSampleERTMain', 'on');
  slConfigUISetEnabled(hDlg, hSrc, 'GenerateSampleERTMain', 0);

  slConfigUISetVal(hDlg, hSrc.Components, 'TargetOS', 'VxWorksExample');
  slConfigUISetEnabled(hDlg, hSrc.Components, 'TargetOS', 0);
  
  slConfigUISetVal(hDlg, hSrc.Components, 'ExtMode', 'on');
  slConfigUISetEnabled(hDlg, hSrc.Components, 'ExtMode', 1);
  
  slConfigUISetVal(hDlg, hSrc.Components, 'ExtModeTransport', '0');
  slConfigUISetEnabled(hDlg, hSrc.Components, 'ExtModeTransport', 1);
  
  slConfigUISetVal(hDlg, hSrc.Components, 'ExtModeMexArgs', 'getpref(''qnx_ert'',''TargetIP'') 1 17725');
  slConfigUISetEnabled(hDlg, hSrc.Components, 'ExtModeMexArgs', 1);
  
  slConfigUISetVal(hDlg, hSrc, 'GenerateMakefile', 'on');
  slConfigUISetEnabled(hDlg, hSrc, 'GenerateMakefile', 0);

  slConfigUISetVal(hDlg, hSrc, 'GenerateErtSFunction', 'off');
  slConfigUISetEnabled(hDlg, hSrc, 'GenerateErtSFunction', 0);

  slConfigUISetVal(hDlg, hSrc, 'MultiInstanceERTCode', 'off');
  slConfigUISetEnabled(hDlg, hSrc, 'MultiInstanceERTCode', 0);

  slConfigUISetVal(hDlg, hSrc, 'MatFileLogging', 'off');
  slConfigUISetEnabled(hDlg, hSrc, 'MatFileLogging', 0);

  slConfigUISetVal(hDlg, hSrc, 'GRTInterface', 'off');
  slConfigUISetEnabled(hDlg, hSrc, 'GRTInterface', 0);

   slConfigUISetVal(hDlg, hSrc, 'ERTCustomFileTemplate', 'qnx_file_process.tlc');
   slConfigUISetEnabled(hDlg, hSrc, 'ERTCustomFileTemplate', 0);

  slConfigUISetVal(hDlg, hSrc, 'SupportNonInlinedSFcns', 'off');
  %slConfigUISetEnabled(hDlg, hSrc, 'SupportNonInlinedSFcns', 0);

  slConfigUISetVal(hDlg, hSrc, 'UtilityFuncGeneration', 'Auto');
  %slConfigUISetEnabled(hDlg, hSrc, 'UtilityFuncGeneration', 0);

  slConfigUISetVal(hDlg, hSrc, 'IncludeMdlTerminateFcn', 0);
  slConfigUISetEnabled(hDlg, hSrc, 'IncludeMdlTerminateFcn', 0);
  
  slConfigUISetVal(hDlg, hSrc, 'InitFltsAndDblsToZero','off');
  
  slConfigUISetVal(hDlg, hSrc, 'ZeroExternalMemoryAtStartup','off');
  
  slConfigUISetVal(hDlg, hSrc, 'ZeroInternalMemoryAtStartup','off');
  
  slConfigUISetVal(hDlg, hSrc, 'PurelyIntegerCode','off');
  
  slConfigUISetVal(hDlg, hSrc, 'SupportNonFinite','off');
  
  slConfigUISetVal(hDlg, hSrc, 'IncludeMdlTerminateFcn','off');
  
  slConfigUISetEnabled(hDlg, hSrc, 'IncludeMdlTerminateFcn',0);
  
  slConfigUISetVal(hDlg, hSrc, 'SupportNonInlinedSFcns','off');
  
  slConfigUISetVal(hDlg, hSrc, 'ERTFirstTimeCompliant','on');
  
  slConfigUISetVal(hDlg, hSrc, 'TargetLibSuffix', '.a');
  slConfigUISetEnabled(hDlg, hSrc, 'TargetLibSuffix', 0); 
  
  

  
  % Set the precompilation directory to the s12x directory.
%   fpath = which(mfilename());
%   [s12xdir, filename] = fileparts(fpath);
%   set_param(bdroot,'TargetPreCompLibLocation',s12xdir);
  
