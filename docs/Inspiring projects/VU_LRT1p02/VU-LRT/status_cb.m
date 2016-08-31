function status_cb(blk, statussrc)
  switch statussrc
      case 'Show'
          if strcmp(get_param([blk '/status'],'BlockType'),'Terminator')
              replace([blk '/status'],'built-in/Outport');
              renumber(blk);
              disp('show');
          end
      case 'Hide'
          if strcmp(get_param([blk '/status'],'BlockType'),'Outport')
              replace([blk '/status'],'built-in/Terminator');
              renumber(blk);
              disp('hide');
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
  if strcmp(statussrc,'Show')
    set_param([blk '/status'],'Port',num2str(n));
    n = 2;
  end
  set_param([blk '/out'],'Port',num2str(n))
end 