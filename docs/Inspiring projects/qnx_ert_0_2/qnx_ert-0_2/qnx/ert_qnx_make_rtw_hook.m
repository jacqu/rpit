function ert_qnx_make_rtw_hook(hookMethod,modelName,~,~,~,~)
switch hookMethod
    case 'after_make'
        qnxAfterMakeHook(modelName);
end