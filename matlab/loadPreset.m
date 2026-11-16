function [cars, motors] = loadPreset()
%LOADPRESET Read the shared JSON presets (../data/cars.json, ../data/motors.json).
%   [cars, motors] = loadPreset()  cars: struct array, motors: struct (fields PMSM, BLDC, ACIM, SRM, DC_Series)
dataDir = fullfile(fileparts(mfilename('fullpath')), '..', 'data');
c = jsondecode(fileread(fullfile(dataDir, 'cars.json')));
m = jsondecode(fileread(fullfile(dataDir, 'motors.json')));
cars = c.cars;
motors = m.motors;   % jsondecode turns "DC Series" into DC_Series
end
