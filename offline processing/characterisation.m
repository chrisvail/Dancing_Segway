clear all

filename = 'bgs';

%% Preprocessing WAV File

[sig fs] = audioread(strcat(filename, '.wav'));

sig = sig(:,1);
x = sig;

x = moving_avg(x, 10)';

%% Beatfinding Processing

E = energy_calc(x, fs);
var = var_calc(E, fs);

alpha = (max(var) / mean(var)) / 6.5;
threshold = alpha*mean(var);

figure(1)
plot(1:length(var),var)

bpm = tempo_calc(var, threshold)


%% Colour Processing

%must separate segmentwise -- how? what are we looking for?
spec = spec_calc(sig, fs);

N = 1 * fs;
col = [];

for i=1:N:length(sig)-N+1
	magnitude = abs(fft(sig(i:i+N-1)));
    m_max = max(magnitude);
    
    df = fs/N;
    f = 0:df:fs/2;
        
    spec = magnitude(1:length(f))/m_max;
    
    %{
    figure(2);
    plot(1:1000,spec(1:1000))
    xlabel('Frequency (Hz)')
    ylabel('Magnitude') 
    title('Spectrum')
    
    pause
    %}
    
    col = [col spec];
end

%% Function Definitions

function y = moving_avg(sig, N_taps)

    for i=N_taps:length(sig)
        temp = 0;
        
        for j = 0:N_taps-1
            temp = temp + sig(i-j);
        end
        y(i) = temp/N_taps;
    end
end

function E = energy_calc(sig, fs)

    N = 0.02 * fs;
    E = [];

    for i=1:N:length(sig)-N+1
        seg = sig(i:i+N-1);
        E = [E seg'*seg];
    end
end

function var = var_calc(E, fs)
    N = 0.02 * fs;    
    var = [];

    for i=1:length(E)
        seg_var = 1/length(E) * (E(i)-mean(E))^2;
        var = [var seg_var];
    end
end

function bpm = tempo_calc(var, threshold)

    beats = [];
    
    for i=1:length(var)
        if var(i) > threshold
            if length(beats) == 0 | i - beats(length(beats)) > 5
                beats = [beats i];
            end
        end
    end
    
    avg_period = mean(diff(beats));
    bpm = 60/(avg_period*0.02);
end

function spec_dB = spec_calc(sig, fs)
    magnitude = abs(fft(sig));
    m_max = max(magnitude);
    
    N = length(sig);
    df = fs/N;
    f = 0:df:fs/2;
        
    spec = magnitude(1:length(f))/m_max;
    spec_dB = 20*log10(spec);
end