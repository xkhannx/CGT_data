function p = zeroPhaseKinematics(ang,samp_freq,cutoff)

fs=samp_freq;   % Sampling frequency (Hz)
dt=1/fs;        % Sample Time (s)
fc = cutoff;    % Cutoff frequency (Hz)

N = length(ang);                    % Find the length of the vector
p = zeros(N,3);                     % Preallocate the output vector
Fa = fft(ang);                      % Do the transform (Thanks MATLAB!)
freq = ((0:1/N:1-1/N)*fs).';        % Calculate frequencies of the fft
Fa(freq>=fc & freq<=fs-fc) = 0;     % Remove high frequency content
p(:,1) = real(ifft(Fa));            % Reconstruct the time domain
p(:,2) = gradient(p(:,1))/dt;       % Differentiate to find velocity
p(:,3) = gradient(p(:,2))/dt;       % Again to find acceleration

end
