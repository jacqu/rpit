function postGenFunc(buildInfo)
   % This function is used to set the token specifying the location of the
   % "target"
   %TODO: this should live in getpref.
   fpath = which(mfilename());
  [qnxsrcdir,  filename] = fileparts(fpath);   
   savedir = pwd;
   cd (qnxsrcdir);
   cd ('..');
   qnxdir = pwd;
   cd(savedir);
   buildInfo.addTMFTokens('|>QNX_MW_ROOT<|', qnxdir);
   
 