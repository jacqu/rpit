function closeNXT(h)

switch class(h),
    case {'nxtusb','w32serial'}
        if h.FileID~=1,
            fclose(h);
        end; %if
    otherwise
        error('Unrecognized type of handle');
end;



        