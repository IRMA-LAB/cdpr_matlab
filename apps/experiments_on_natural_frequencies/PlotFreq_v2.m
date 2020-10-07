function [pks,f_peaks] = PlotFreq_v2(v,line,name)

    v_mean = mean(v);
    v = v-v_mean;
    Fs = 100;            % Sampling frequency                    
    T = 1/Fs;             % Sampling period       
    L = length(v);             % Length of signal
    t = (0:L-1)*T;  
    Y = fft(v);
    P2 = abs(Y/L);
    P1 = P2(1:L/2+1);
    P1(2:end-1) = 2*P1(2:end-1);
    f = Fs*(0:(L/2))/L;
    plot(f(1:401),P1(1:401),line,'DisplayName',name,'LineWidth',2) 
    
     [pks,locs] = findpeaks(P1);
     [pks,idx] = sort(pks,'descend');
     locs = locs(idx);
     f_peaks = f(locs);
     f_peaks(100:end) = [];
     pks(100:end) = [];
%         
%     f_max = f_peaks(1:4);
%     f_max = floor((f_max*1000))/1000;
    
end