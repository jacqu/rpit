% Called before Simulink starts

if exist('last_positions.mat', 'file') == 2
  load last_positions;
  init_positions = mod( last_positions, 4096 );
else
  init_positions = 0;
end