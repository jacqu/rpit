% ecrobot_hooks(file)
% This function scans the user-created .mdl file as text and determines
% whether or not USB is used.  If so, it places the 1ms usb process
% function in the type 2 isr hook function.
function ecrobot_hooks(inid)
  % status variables
  isr = '';
  head = '';
  f = 0;
  g = 0;
  fid = fopen('ecrobot_hooks.h','w+');
  while ~feof(inid)
    s = fgetl(inid);
    if  (~isempty(strfind(s,'lego_nxt_lib/USB Interface')) || ~isempty(strfind(s,'lego_nxt_lib/USB Rx'))...
            || ~isempty(strfind(s,'lego_nxt_lib/USB Tx')))&& (f<1)
      % usb interface block included in model
      isr = strcat(isr,sprintf('\tGetResource(RES_Rx);\n\tecrobot_process1ms_usb();\n\tReleaseResource(RES_Rx);\n'));
      head = strcat(head,sprintf('\n\n#ifdef EXT_MODE\n#error "USB block cannot be used when ext. mode is enabled"\n#endif'));
      if (g<1)
          head = strcat(head,sprintf('\n\nDeclareResource(RES_Rx);'));
      end
      f = 1;
    elseif (~isempty(strfind(s,'lego_nxt_lib/BT Interface')) || ~isempty(strfind(s,'lego_nxt_lib/BT Rx'))...
            || ~isempty(strfind(s,'lego_nxt_lib/BT Tx')))&& (g<1)
        while (isempty(strfind(s,'Parameters')))
            s = fgetl(inid);
        end
        if (strfind(s,'"master,'))
            ip = substring(substring(s,find(s==',')),-1,1);
            isr = strcat(isr,sprintf(['\tGetResource(RES_Rx);\n\tecrobot_init_bt_master(' ip ',"1234");\n\tReleaseResource(RES_Rx);\n']));
        else
            isr = strcat(isr,sprintf('\tGetResource(RES_Rx);\n\tecrobot_init_bt_slave("1234");\n\tReleaseResource(RES_Rx);\n'));
        end
        head = strcat(head,sprintf('\n\n#ifdef EXT_MODE\n#error "BT block cannot be used when ext. mode is enabled"\n#endif'));
      if (f<1)
          head = strcat(head,sprintf('\n\nDeclareResource(RES_Rx);'));
      end
      g = 1;
    end
  end
  % write strings to ecrobot_hooks.h
  fprintf(fid, ['#ifndef ECROBOT_HOOKS_H\n',...
        '#define ECROBOT_HOOKS_H\n\n',...
        '#include "ecrobot_interface.h"\n',...
        '#include "kernel.h"\n',...
        '#include "kernel_id.h"\n\n',...
        '/* ECRobot hooks */',...
        '%s\n\nvoid user_1ms_isr_type2(void) {\n',...
        '%s}\n\n#endif\n'], head, isr);
  fclose(fid);
end