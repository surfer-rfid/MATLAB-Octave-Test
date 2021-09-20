function testRXChain

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%                                                                                          %
% Filename: testRXChain.m                                                                  %
% Creation Date: 11/03/2015                                                                %
% Author: Edward Keehr                                                                     %
%                                                                                          %
% Copyright Superlative Semiconductor LLC 2021                                             %
% This source describes Open Hardware and is licensed under the CERN-OHL-P v2              %
% You may redistribute and modify this documentation and make products                     %
% using it under the terms of the CERN-OHL-P v2 (https:/cern.ch/cern-ohl).                 %
% This documentation is distributed WITHOUT ANY EXPRESS OR IMPLIED                         %
% WARRANTY, INCLUDING OF MERCHANTABILITY, SATISFACTORY QUALITY                             %
% AND FITNESS FOR A PARTICULAR PURPOSE. Please see the CERN-OHL-P v2                       %
% for applicable conditions.                                                               %
%                                                                                          %
% Test the SDM along with the postfilters to make sure that reasonable SNR                 %
% can be achieved. Also, check that the time-domain versions of the filters implement the  %
% same transfer functions as their ideal prototypes.                                       %
%                                                                                          %
% 110315 - This file tests the decimation filter for received signals of different freq.   %
% and amplitude.                                                                           %
% 110415 - In this file, we add the postfilter and compute the remaining SNR               %
% 111715 - Copy over to release directory and add respectable comments.                    %
% 112815 - Modify to generate output files for Verilog comparison.                         %
% 021916 - Modify to accomodate data capture at SDM input.                                 %
% 032816 - Get changes in filters designed to reduce LUT checked out.                      %
%                                                                                          %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% 
%                                                                                          %
% Section 0 - Preliminaries - set simulation constants                                     %
%                                                                                          %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%%% Use Richard Schreier's Delta Sigma MATLAB Toolbox.
addpath ./delsig
pkg load all
load -mat genNotchBBFTotal032816.mat;

npoints=2^16;                                                           %%% Number of simulation points
Fs=36000000;                                                            %%% Sim. and SDM sample rate
nfft_points=npoints/2;                                                  %%% Number of points to be used in FFT
time_vec=(0:npoints-1)/Fs;                                              %%% Time vector to be used for plotting
bin_freq=Fs/2/nfft_points;                                              %%% FFT bin frequency
sine_ampl=0.03125*10^(-50/20);                                          %%% Amplitude of desired RX signal at SDM input
sine_ampl=0.03125*10^(0/20); 

sinc_order=4;                                                           %%% Order of first CIC/decimation filter
sinc_dec_fact=8;                                                        %%% Decimation factor of first CIC/decimation filter
sinc_Fs=Fs/sinc_dec_fact;                                               %%% Sampling rate of system after the first decimation

sinc_filter_pre=conv(ones(1,sinc_dec_fact),ones(1,sinc_dec_fact));      %%% Construct an ideal CIC/decimation filter
sinc_filter=conv(sinc_filter_pre,sinc_filter_pre);                      %%% This will be used to make a replica signal which we can subtract from the real signal to get the error
sinc_filter=conv(sinc_filter,[0 0 0 0 1]);                              %%% Add a delay to match the real hardware implementation


sinc_2_dec_fact=128;                                                    %%% Decimation factor of second decimation filter
sinc_2_Fs=sinc_Fs/sinc_2_dec_fact;                                      %%% Sampling rate of system after the second decimation

%sinc_2_filter_den=conv([128 -127],[128 -127]);
%sinc_2_filter_den=conv([64 -63],[64 -63]);

sinc_2_filter_num=ones(1,128);
sinc_2_filter_den=[64 -63];

order=5;                                                                %%% Specify the parameters of what we think the RX SDM in the SX1257 is doing
osr=32;
opt=1;
H_inf=1.5;
f0=0;

ntf = synthesizeNTF(order,osr,opt,H_inf,f0);                            %%% Generate the NTF of the SDM, assuming that the STF is unity

time_vec_dec=time_vec(1:sinc_dec_fact:end);                             %%% Create a decimated time vector for plotting
time_2_vec_dec=time_vec_dec(1:sinc_2_dec_fact:end);                     %%% Create a decimated time vector for plotting

tone_freq_Hz_vec=bin_freq*[0 289 404 625 1087];                         %%% Check different tones to check frequency response at different positions
extra_gain_vec=[16 1 1 8 16];                                           %%% Scale different signals to see how big you must really make register widths
%tone_freq_Hz_vec=bin_freq*[289];
%extra_gain_vec=[1];
snr_vec=zeros(1,length(tone_freq_Hz_vec));                              %%% SNR result vector

adder_D1_hist=zeros(7,length(time_vec),length(tone_freq_Hz_vec));       %%% Retain the adder values so that we can see how many registers are utilized
adder_B1_hist=zeros(3,length(time_vec_dec),length(tone_freq_Hz_vec));
adder_B2_hist=zeros(3,length(time_vec_dec),length(tone_freq_Hz_vec));
adder_D2_hist=zeros(2,length(time_vec_dec),length(tone_freq_Hz_vec));

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%                                                                                          %
% Section 1 - Loop frequencies of interest and run simulation                              %
%                                                                                          %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%for loop_f=1
for loop_f=1:length(tone_freq_Hz_vec)

  tic
  disp(sprintf('At freq loop %d',loop_f));
  sdm_in=extra_gain_vec(loop_f)*sine_ampl*cos(2*pi*tone_freq_Hz_vec(loop_f)*time_vec);         %%% Generate the input tone
  sdm_out_ideal=sdm_in;                                                         %%% Assuming unity STF, the ideal SDM output is the same as the input
  [sdm_out_real,xn,xmax,y]=simulateDSM(sdm_in,ntf,2,0);                         %%% Run the RX SDM
   sdm_out_real=round((sdm_out_real+1)/2);                                      %%% Convert the SDM output to what we really see from the SDM (i.e. 1,0,1,0)
  
                                                                                %%% Model the delay at the beginning of CIC8
  sdm_out_ideal=[-1 sdm_out_ideal(1:end-1)];
  
  sinc_out_ideal=filter(sinc_filter,1,sdm_out_ideal);                           %%% Run the ideal SDM output through the ideal CIC/Decimation filter
  sinc_out_ideal=sinc_out_ideal(1:sinc_dec_fact:end);                           %%% And then perform an ideal decimation
  sinc_out_ideal=filter([0 1],1,sinc_out_ideal);                                %%% Add an additional delay to model the clock domain crossing
  [sinc_out_real,adder_D1_hist_loop_f]=decimFilterBitExact(sdm_out_real);       %%% Perform the bit exact decimation on the real SDM output
  
  adder_D1_hist(:,:,loop_f)=adder_D1_hist_loop_f;                               %%% Populate the adder result matrix
  
  tuned_out_ideal=filter(Bsave,Asave,64*sinc_out_ideal);                        %%% Perform ideal baseband filtering on the ideal signal
  [tuned_out_real,adder_B1,adder_B2]=tunedIIRFilterBitExact(sinc_out_real);     %%% The gain of 256 is to ensure that the RX signal has sufficient amplitude to be demodulated
  adder_B1_hist(:,:,loop_f)=adder_B1;                                           %%% Populate the adder result matrix
  adder_B2_hist(:,:,loop_f)=adder_B2;
  
  error_chfilt=tuned_out_ideal-tuned_out_real;                                  %%% Examine the error
  snr_vec(loop_f)=10*log10(sum(tuned_out_ideal(2^12:end).^2)/sum(error_chfilt(2^12:end).^2));      %%% Look at the total SNR since this is what goes into the CR circuit
  
  sinc_2_out_ideal=filter(sinc_2_filter_num,16*sinc_2_filter_den,[0 sinc_out_ideal(1:end-1)]);
  sinc_2_out_ideal=sinc_2_out_ideal(1:sinc_2_dec_fact:end);
  sinc_2_out_ideal=filter([1],1,sinc_2_out_ideal);                              %%% Add an additional delay to model the clock domain crossing
  [sinc_2_out_real,adder_D2_hist_loop_f]=decimFilterSecondBitExact(sinc_out_real(1:end));
  adder_D2_hist(:,:,loop_f)=adder_D2_hist_loop_f;                               %%% Populate the adder result matrix
  error_sinc_2=sinc_2_out_ideal-sinc_2_out_real;                                %%% Examine the error, knowing precise SNR not a big deal right now
  
  figure(loop_f); clf; hold on;                                                 %%% Plot individual results per frequency tone
  %plot(time_vec_dec,256*sinc_out_real,'k');
  plot(time_vec_dec,tuned_out_real,'b');
  plot(time_vec_dec,tuned_out_ideal,'r');
  plot(time_vec_dec,1*error_chfilt,'g');
  grid on;
  
  figure(loop_f+length(tone_freq_Hz_vec)); clf; hold on;                        %%% Plot individual results per frequency tone
  %plot(time_vec_dec,256*sinc_out_real,'k');
  plot(time_2_vec_dec,sinc_2_out_real,'b');
  plot(time_2_vec_dec,sinc_2_out_ideal,'r');
  plot(time_2_vec_dec,1*error_sinc_2,'g');
  grid on;
  
  fid1=fopen(sprintf("./rtl_test_vectors/cic8_in_loopf_%d.dat",loop_f),"w");
  fid2=fopen(sprintf("./rtl_test_vectors/cic8_out_loopf_%d.dat",loop_f),"w");
  fid3=fopen(sprintf("./rtl_test_vectors/chfilt_out_loopf_%d.dat",loop_f),"w");
  fid4=fopen(sprintf("./rtl_test_vectors/dc_out_loopf_%d.dat",loop_f),"w");

  for loop_i=1:length(sdm_out_real);
    fprintf(fid1,"%d\n",sdm_out_real(loop_i));
  endfor
  
  for loop_i=1:length(sinc_out_real);
    fprintf(fid2,"%d\n",sinc_out_real(loop_i));
  endfor
  
  for loop_i=1:length(tuned_out_real);
    fprintf(fid3,"%d\n",tuned_out_real(loop_i));
  endfor
  
  for loop_i=1:length(sinc_2_out_real);
    fprintf(fid4,"%d\n",sinc_2_out_real(loop_i));
  endfor
  
  fclose(fid1);
  fclose(fid2);
  fclose(fid3);
  fclose(fid4);
  
  toc
  
endfor

snr_vec                                                                         %%% Debugging message - just spit out the SNR vector

%%% Display the utilized bit widths in all of the actual hardware

disp(sprintf('Max bits D1 Adder 1: %d',ceil(log10(max(max(abs(adder_D1_hist(1,:,:)))))/(log10(2)))+1));
disp(sprintf('Max bits D1 Adder 2: %d',ceil(log10(max(max(abs(adder_D1_hist(2,:,:)))))/(log10(2)))+1));
disp(sprintf('Max bits D1 Adder 3: %d',ceil(log10(max(max(abs(adder_D1_hist(3,:,:)))))/(log10(2)))+1));
disp(sprintf('Max bits D1 Adder 4: %d',ceil(log10(max(max(abs(adder_D1_hist(4,:,:)))))/(log10(2)))+1));
disp(sprintf('Max bits D1 Adder 5: %d',ceil(log10(max(max(abs(adder_D1_hist(5,:,:)))))/(log10(2)))+1));
disp(sprintf('Max bits D1 Adder 6: %d',ceil(log10(max(max(abs(adder_D1_hist(6,:,:)))))/(log10(2)))+1));
disp(sprintf('Max bits D1 Adder 7: %d',ceil(log10(max(max(abs(adder_D1_hist(7,:,:)))))/(log10(2)))+1));
%disp(sprintf('Max bits D1 Adder 8: %d',ceil(log10(max(max(abs(adder_D1_hist(8,:,:)))))/(log10(2)))+1));
disp('\n');
disp(sprintf('Max bits B1 Adder 1: %d',ceil(log10(max(max(abs(adder_B1_hist(1,:,:)))))/(log10(2)))+1));
disp(sprintf('Max bits B1 Adder 2: %d',ceil(log10(max(max(abs(adder_B1_hist(2,:,:)))))/(log10(2)))+1));
disp(sprintf('Max bits B1 Adder 3: %d',ceil(log10(max(max(abs(adder_B1_hist(3,:,:)))))/(log10(2)))+1));
disp('\n');
disp(sprintf('Max bits B2 Adder 1: %d',ceil(log10(max(max(abs(adder_B2_hist(1,:,:)))))/(log10(2)))+1));
disp(sprintf('Max bits B2 Adder 2: %d',ceil(log10(max(max(abs(adder_B2_hist(2,:,:)))))/(log10(2)))+1));
disp(sprintf('Max bits B2 Adder 3: %d',ceil(log10(max(max(abs(adder_B2_hist(3,:,:)))))/(log10(2)))+1));
disp('\n');
disp(sprintf('Max bits D2 Adder 1: %d',ceil(log10(max(max(abs(adder_D2_hist(1,:,:)))))/(log10(2)))+1));
disp(sprintf('Max bits D2 Adder 2: %d',ceil(log10(max(max(abs(adder_D2_hist(2,:,:)))))/(log10(2)))+1));