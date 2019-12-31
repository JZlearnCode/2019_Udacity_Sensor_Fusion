% Author: Judy Z 
clear all
clc;

%% Radar Specifications 
fc = 77e9; %e9 to bring to GHz
max_range = 200; %meters
range_resolution = 1; %meters
max_velocity = 70; %meters per second
speed_of_light = 3e8; % meters per second 
velocity_resolution = 3; % m/s
%% User Defined Range and Velocity of target
target_initial_position = 100; % meters 
target_initial_velocity = 50;    
%% FMCW Waveform Generation
% Design the FMCW waveform by giving the specs of each of its parameters.
% Calculate the Bandwidth (B), Chirp Time (Tchirp) and Slope (slope) of the FMCW
% chirp using the requirements above.

%Operating carrier frequency of Radar 
sweep_bandwidth = speed_of_light/(2*range_resolution);
sweep_time_factor = 5.5;
round_trip_time = 2 * max_range / speed_of_light; 
chirp_time = sweep_time_factor * round_trip_time; 
slope = sweep_bandwidth / chirp_time; %slope of FMCW chirp
%The number of chirps in one sequence. Its ideal to have 2^ value for the ease of running the FFT
%for Doppler Estimation. 
Nd=128;                   % #of doppler cells OR #of sent periods % number of chirps

%The number of samples on each chirp. 
Nr=1024;                  %for length of time OR # of range cells

% Timestamp for running the displacement scenario for every sample on each
% chirp
t=linspace(0,Nd*chirp_time,Nr*Nd); %total time for samples

%Creating the vectors for Tx, Rx and Mix based on the total samples input.
Tx=zeros(1,length(t)); %transmitted signal
Rx=zeros(1,length(t)); %received signal
Mix = zeros(1,length(t)); %beat signal

%Similar vectors for range_covered and time delay.
range_covered=zeros(1,length(t));
time_delay=zeros(1,length(t));


%% Signal generation and Moving Target simulation
% Running the radar scenario over the time. 

for i=1:length(t)         
    %For each time stamp update the Range of the Target for constant velocity. 
    range_covered(i) = target_initial_position + (target_initial_velocity*t(i));
    time_delay(i) = (2*range_covered(i))/speed_of_light;%time used for signal to leave source, reach target and get back to source
    %For each time sample we need update the transmitted and
    %received signal. 
    Tx(i) = cos(2*pi*(fc*t(i) + 0.5*slope*t(i)^2));
    dt = t(i)-time_delay(i);
    Rx(i) = cos(2*pi*(fc*dt + 0.5*slope*dt^2));
 
    %Now by mixing the Transmit and Receive generate the beat signal
    %This is done by element wise matrix multiplication of Transmit and
    %Receiver Signal
    Mix(i) = Tx(i).*Rx(i);
    
end

%% RANGE MEASUREMENT
%reshape the vector into Nr*Nd array. Nr and Nd here would also define the size of
%Range and Doppler FFT respectively.
Mix = reshape(Mix, [Nr, Nd]);

%run the FFT on the beat signal along the range bins dimension (Nr) and
%normalize.
signal_fft = fft(Mix, Nr);
%scale it by dividing the fft result by the length of the time-domain signal
signal_fft = signal_fft./Nr;
% Take the absolute value of FFT output
signal_fft = abs(signal_fft);
% Output of FFT is double sided signal, but we are interested in only one side of the spectrum.
% Hence we throw out half of the samples.
signal_fft = signal_fft(1:Nr/2);

%plotting the range
figure ('Name','Range from First FFT')
% plot FFT output 
plot(signal_fft);
axis ([0 200 0 1]);
title('Single-sided amplitude spectrum');
xlabel('frequency (Hz)');
ylabel('Amplitude');
saveas(gcf,'./images/FFT_result.png');

%% RANGE DOPPLER RESPONSE
% The 2DFFT on the mixed signal (beat signal) outputs and generates 
% a range doppler map.

% Range Doppler Map Generation.

% The output of the 2D FFT is an image that has reponse in the range and
% doppler FFT bins. So, it is important to convert the axis from bin sizes
% to range and doppler based on their Max values.

Mix=reshape(Mix,[Nr,Nd]);

% 2D FFT using the FFT size for both dimensions.
sig_fft2 = fft2(Mix,Nr,Nd);

% Taking just one side of signal from Range dimension.
sig_fft2 = sig_fft2(1:Nr/2,1:Nd);
sig_fft2 = fftshift (sig_fft2);
RDM = abs(sig_fft2);
RDM = 10*log10(RDM) ;

%use the surf function to plot the output of 2DFFT and to show axis in both
%dimensions
doppler_axis = linspace(-100,100,Nd);
range_axis = linspace(-200,200,Nr/2)*((Nr/2)/400);
figure,surf(doppler_axis,range_axis,RDM);
saveas(gcf,'./images/2D_FFT.png');

%% CFAR implementation
%Slide Window through the complete Range Doppler Map (RDM)
%Select the number of Training Cells in both the dimensions.
num_training_range = 12;
num_training_doppler = 3;
%Select the number of Guard Cells in both dimensions around the Cell under 
%test (CUT) for accurate estimation
num_guard_range = 4;
num_guard_doppler = 1;
% offset the threshold by SNR value in dB
offset = 10; 

%design a loop such that it slides the CUT across range doppler map by
%giving margins at the edges for Training and Guard Cells.
%For every iteration sum the signal level within all the training
%cells. To sum convert the value from logarithmic to linear using db2pow
%function. Average the summed values for all of the training
%cells used. After averaging convert it back to logarithimic using pow2db.
%Further add the offset to it to determine the threshold. Next, compare the
%signal under CUT with this threshold. If the CUT level > threshold assign
%it a value of 1, else equate it to 0.
CFAR = zeros(size(RDM)); 

range_train_guard = num_training_range + num_guard_range; 
doppler_train_guard = num_training_doppler + num_guard_doppler;
min_range_idx = range_train_guard + 1;
max_range_idx = Nr /2 - range_train_guard;
min_doppler_idx = doppler_train_guard + 1;
max_doppler_idx = Nd - doppler_train_guard;

% number of cells in train, guard and CUT 
train_block_size = (2*range_train_guard+1) * (2*doppler_train_guard+1); 
guard_block_size = (2*num_guard_range+1)*(2*num_guard_doppler+1);
num_train_cells = train_block_size - guard_block_size; 
        
% Use RDM[x,y] as the matrix from the output of 2D FFT to implement CFAR
for range_idx = min_range_idx : max_range_idx
    for doppler_idx = min_doppler_idx : max_doppler_idx
        sum_noise_level = 0;
        r_min = range_idx - range_train_guard;
        r_max = range_idx + range_train_guard;
        d_min = doppler_idx - doppler_train_guard;
        d_max = doppler_idx + doppler_train_guard;
        for r_id = r_min : r_max
            for d_id = d_min : d_max
                is_train_cell = (abs(range_idx - r_id) > num_guard_range || abs(doppler_idx - d_id) > num_guard_doppler);
                if is_train_cell
                    cur_noise = db2pow(RDM(r_id, d_id)); 
                    sum_noise_level = sum_noise_level + cur_noise; 
                end
            end
        end
        % average noise level
        avg_noise_level = sum_noise_level/num_train_cells;
  
        % calculate threshold 
        thresh = pow2db(avg_noise_level);
        thresh = thresh + offset;
        
        CUT = RDM(range_idx, doppler_idx);
        
        if CUT > thresh
            CFAR(range_idx, doppler_idx) = 1;
        end
    end
end

% The process above will generate a thresholded block, which is smaller
% than the Range Doppler Map as the CUT cannot be located at the edges of
% matrix. Hence,few cells will not be thresholded. CFAR has those values  
% set to 0, to keep the map size same
  
%display the CFAR output using the Surf function like we did for Range
%Doppler Response output.
figure('Name', '2D CFAR');
surf(doppler_axis,range_axis, CFAR);
colorbar;
saveas(gcf,'./images/CFAR_result.png')

 
 