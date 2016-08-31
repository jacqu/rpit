function [ ] = qnxAfterMakeHook( modelName )
if (strcmp(get_param(modelName,'SystemTargetFile')  ,'ert_qnx.tlc') && ...
    strcmp(get_param(modelName,'TemplateMakefile')  ,'ert_qnx.tmf'))
  
    % Check if user chose to Download to QNX in Settings
    if verLessThan('matlab', '8.1')
        makertwObj = get_param(gcs, 'MakeRTWSettingsObject');
    else
        % TODO use rtwprivate('get_makertwsettings',gcs,'BuildInfo') - see
        % R2013a Simulink Coder release notes.
        makertwObj = coder.internal.ModelCodegenMgr.getInstance;
    end
    makertwArgs = makertwObj.BuildInfo.BuildArgs;
    downloadToQNX = 1;
    for i=1:length(makertwArgs)
        if strcmp(makertwArgs(i).DisplayLabel,'QNX_DOWNLOAD')
            downloadToQNX = str2double(makertwArgs(i).Value);
        end
    end
    
    % allow a back door for tests to skip download to hardware
    if evalin('base','exist(''downloadToQNX'')')
        downloadToQNX = evalin('base', 'downloadToQNX' );
    end
    
    if ~i_isPilSim && ~i_isModelReferenceBuild(modelName) &&...
            downloadToQNX
        qnx_download(modelName,makertwObj)
    end

end

end

function isPilSim = i_isPilSim
    s = dbstack;
    isPilSim = false;
    for i=1:length(s)
        if strfind(s(i).name,'build_pil_target')
            isPilSim=true;
            break;
        end
    end
end
    
function isMdlRefBuild = i_isModelReferenceBuild(modelName)
    mdlRefTargetType = get_param(modelName, 'ModelReferenceTargetType');
    isMdlRefBuild = ~strcmp(mdlRefTargetType, 'NONE');
end