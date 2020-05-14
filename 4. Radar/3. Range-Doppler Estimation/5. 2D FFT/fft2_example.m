% 2-D Transform
% The 2-D Fourier transform is useful for processing 2-D signals
% and other 2-D data such as images.
% Create and plot 2-D data with repeated blocks.

% https://de.mathworks.com/help/matlab/ref/fft2.html

% The following steps can be used to compute a 2D FFT in MATLAB:
% 
% 1. Take a 2D signal matrix
% 2. In the case of Radar signal processing. Convert the signal in
%    MxN matrix, where M is the size of Range FFT samples and N is the
%    size of Doppler FFT samples:
%    signal  = reshape(signal, [M, N]);
% 3. Run the 2D FFT across both the dimensions.
%    signal_fft = fft2(signal, M, N);
% 4. Shift zero-frequency terms to the center of the array.
%    signal_fft = fftshift(signal_fft);
% 5. Take the absolute value
%    signal_fft = abs(signal_fft);
% 6. Here since it is a 2D output, it can be plotted as an image.
%    Hence, we use the imagesc function
%    imagesc(signal_fft);

close all;

% Steps 1..2
P = peaks(20);
X = repmat(P, [5 10]);

figure; imagesc(X);

% Step 3
Y = fft2(X, 100, 200);

% Step 4
Y = fftshift(Y);

% Step 5
Y = abs(Y);

% Step 6
figure; imagesc(Y)

