% Calculate thevelocity in m/s of four targets with following doppler 
% frequency shifts: [3 KHz, -4.5 KHz, 11 KHz, -3 KHz].

% Doppler Velocity Calculation
c = 3*10^8;         %speed of light
frequency = 77e9;   %frequency in Hz (1e9 Hz = 1 GHz)

% Calculate the wavelength
wavelength = c / frequency;

% Define the doppler shifts in Hz using the information from above 
doppler_shifts = [3e3, -4.5e3, 11e3, -3e3];

% Calculate the velocity of the targets  fd = 2*vr/lambda
Vr = doppler_shifts * wavelength / 2;

% Display results
disp(Vr);
