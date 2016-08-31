function qnx_download(modelName,makertwObj)

disp(['### Downloading ', modelName, ' to QNX Target Board...']);

if isunix
    disp('Download not implemented');
else
% Temporary file with commands for ftp
filename = [tempname,'.ftp'];
fid = fopen(filename, 'w');
[~, upname, ~] = fileparts(filename);
upname = [modelName,'_',upname];
ftpcmd = {
'verbose'
['open ',getpref('qnx_ert','TargetIP')]
'ftp'
'password'
'binary'
% Construct unique destination file name
['put ',fullfile(makertwObj.BuildDirectory,'..',modelName),' ',upname]
'bye'
};
for i=1:length(ftpcmd)
    fprintf(fid,'%s\n',ftpcmd{i});
end
fclose(fid);
command = sprintf('%s -s:%s','ftp',filename);
[status, out] = system(command);
disp(out);
delete(filename);

% Execute the uploaded file
tokens = makertwObj.BuildInfo.Tokens;

for i=1:length(tokens)
    if strcmp(tokens(i).DisplayLabel,'|>QNX_MW_ROOT<|')
        QNX_MW_ROOT = tokens(i).Value;
    end
end

plink = fullfile(QNX_MW_ROOT,'qnx','plink.exe');

% Temporary file with commands for plink
filename = [tempname,'.plink'];
fid = fopen(filename, 'w');
plinkcmd = {
'root'
['chmod +x /tmp/',upname]
['/tmp/',upname,' &']
'exit'
};
for i=1:length(plinkcmd)
    fprintf(fid,'%s\n',plinkcmd{i});
end
fclose(fid);
command = sprintf('%s -telnet %s < %s',plink,getpref('qnx_ert','TargetIP'),filename);
[status, out] = system(command);
disp(out);
delete(filename);
end
