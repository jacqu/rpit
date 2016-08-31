% cygwin_naming(...)
% Converts files from using Windows naming conventions into using cygwin 
% POSIX naming conventions.
function argout = cygwin_naming(varargin)
    arg = strrep(varargin,'rt_logging.c','');
    arg = strrep(arg,'c:','/cygdrive/c');
    arg = strrep(arg,'C:','/cygdrive/c');
    arg = strrep(arg,'\','/');
    [i j] = size(arg{:});
    if j > 0 && arg{1}(j) == '/'
        arg{1}(j) = '\';
    end
    argout = arg{1};