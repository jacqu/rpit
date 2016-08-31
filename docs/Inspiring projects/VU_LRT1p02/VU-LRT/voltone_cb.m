% voltone_cb(...)
% Switchboard type function to handle all callbacks from the 'Volume Tone'
% dynamic masked block.
function voltone_cb(blk,volsrc,vol,freqsrc,freq) 
  switch volsrc
    case 'External'
      if strcmp(get_param([blk '/vol'],'BlockType'),'Constant')
        replace([blk '/vol'],'built-in/Inport');
      end
    case 'Internal'
      if strcmp(get_param([blk '/vol'],'BlockType'),'Inport')
        replace([blk '/vol'],'built-in/Constant')
        set_param([blk '/vol'],'Value','vol','OutDataTypeMode','uint8','OutDataTypeStr','uint8','OutScaling','2^0')
      end
  end
  switch freqsrc
    case 'External'
      if strcmp(get_param([blk '/freq'],'BlockType'),'Constant')
        replace([blk '/freq'],'built-in/Inport');
      end
    case 'Internal'
      if strcmp(get_param([blk '/freq'],'BlockType'),'Inport')
        replace([blk '/freq'],'built-in/Constant')
        set_param([blk '/freq'],'Value','freq','OutDataTypeMode','uint32','OutDataTypeStr','uint32','OutScaling','2^0')
        
      end
  end
  renumber(blk,freqsrc,volsrc);
end

% replace(...)
% Local function to replace an Inport block with a Constant block, or vice
% versa.
function replace(oldblock,newblock)
  pos = get_param(oldblock,'Position');
  orient = get_param(oldblock,'Orientation');
  delete_block(oldblock);
  add_block(newblock,oldblock,'Position',pos,'Orientation',orient);
end

% renumber(...)
% Local function which renumbers ports to maintain block functionality.
function renumber(blk,freqsrc,volsrc)
  % Renumber ports
  % when using external upper limit,
  % set blk/up port to 1
  n = 1;
  str = sprintf('image(imread(''tone.jpg''));\n');
  if strcmp(freqsrc,'External')
    set_param([blk '/freq'],'Port',num2str(n))
    str = sprintf('%sport_label(''input'',%d,''freq'')\n',str,n);
    n = 2;
  end
  if strcmp(volsrc,'External')
    set_param([blk '/vol'],'Port',num2str(n))
    str = sprintf('%sport_label(''input'',%d,''vol'')\n',str,n);
  end
  set_param(blk,'MaskDisplay',str);
end 