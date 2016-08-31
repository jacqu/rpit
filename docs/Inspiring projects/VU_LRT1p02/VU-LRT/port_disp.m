function port_disp(blk)

temp = get_param(blk,'MaskDisplay');
switch get_param(blk,'port')
    case 'Port A'
        t = 'A';
    case 'Port B'
        t = 'B';
    case 'Port C'
        t = 'C';
    case 'Port 1'
        t = '1';
    case 'Port 2'
        t = '2';
    case 'Port 3'
        t = '3';
    case 'Port 4'
        t = '4';
end;
x = strfind(temp,'''');
y = strfind(temp,'text(0.1');
letter = x(find(x>y,1));
temp(letter+1) = t;
set_param(blk,'MaskDisplay',temp);