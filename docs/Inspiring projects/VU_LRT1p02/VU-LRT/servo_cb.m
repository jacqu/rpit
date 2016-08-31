% servo_cb(...)
% Switchboard type function to handle all callbacks from the 'Servo Motor'
% dynamic masked block.
function servo_cb(blk, state, initrevsrc, initrev)
  switch state
      case 'init'
          servo_init_cb(blk, initrevsrc, initrev);
      case 'initrevsrc'
          initrevsrc_cb(blk);
      otherwise
          disp('callback not implemented.');
  end
end

% servo_init_cb(...)
% Initialization callback to modify block appearance when internal
% parameters are changed and applied, or when block is initially loaded.
function servo_init_cb(blk, initrevsrc, initrev)
  switch initrevsrc
      case 'External'
          if strcmp(get_param([blk '/Ri'],'BlockType'),'Constant')
              replace([blk '/Ri'],'built-in/Inport');
              renumber(blk)
          end
      case 'Internal'
          if strcmp(get_param([blk '/Ri'],'BlockType'),'Inport')
              replace([blk '/Ri'],'built-in/Constant')
              set_param([blk '/Ri'],'Value','initrev')
              renumber(blk)
          end
  end
end

function replace(oldblock,newblock)
  pos = get_param(oldblock,'Position');
  orient = get_param(oldblock,'Orientation');
  delete_block(oldblock);
  add_block(newblock,oldblock,'Position',pos,'Orientation',orient);
end

% renumber(...)
% Local function which renumbers ports to maintain block functionality.
function renumber(blk)
  % Renumber ports
  % when using external upper limit,
  % set blk/up port to 1
  n = 1;
  set_param([blk '/pow'],'Port',num2str(n));
  if strcmp(initrevsrc,'External')
    n = 2;
    set_param([blk '/freq'],'Port',num2str(n))
  end
end 

% initrevsrc_cb(...)
% Callback function specific to the 'Ri' input.
function initrevsrc_cb(blk)
  en = get_param(blk,'MaskEnables');
  switch get_param(blk,'initrevsrc')
    case 'External'
      en{4} = 'off';
    case 'Internal'
      en{4} = 'on';
  end
  set_param(blk,'MaskEnables',en)
end