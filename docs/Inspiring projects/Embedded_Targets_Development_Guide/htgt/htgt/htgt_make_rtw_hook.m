function htgt_make_rtw_hook(hookMethod,modelName,rtwroot,templateMakefile,buildOpts,buildArgs)
% Hook file for the Workshop Host Target 
 
  switch hookMethod
   case 'error'
    disp('error');
   case 'entry'
    disp('Starting build');
    disp('Good hook spot to run Model Advisor checks');
   case 'before_tlc'
    % do nothing  
   case 'after_tlc'
    % do nothing
   case 'before_make'
    % do nothing
   case 'after_make'
    % do nothing
   case 'exit'
    disp('Done building');    
  end
