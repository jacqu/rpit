function blkStruct = slblocks

% slblocks
%   defines the Simulink library block representation
%   for the Raspberry pi blockset.

%   Copyright 2010 The MathWorks, Inc.

blkStruct.Name    = ['RPI Blockset'];
blkStruct.OpenFcn = '';
blkStruct.MaskInitialization = '';
blkStruct.MaskDisplay = '';

% Define the library list for the Simulink Library browser.
% Return the name of the library model and the name for it
%
Browser(1).Library = 'rpi_blkst';
Browser(1).Name    = 'RPI Blockset';
Browser(1).IsFlat  = 0;

blkStruct.Browser = Browser;

% define information for model updater
% What's this?, in R2010a, this is needed to avoid an error when opening
% the library browser.
%blkStruct.ModelUpdaterMethods.fhDetermineBrokenLinks = @sl3dBrokenLinksMapping;
blkStruct.ModelUpdaterMethods.fhDetermineBrokenLinks = [];

% End of slblocks.m
