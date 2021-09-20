%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%                                                                              %
% 110815 - test_txSDMBitExact                                                  %
%                                                                              %
% Filename: test_txSDMBitExact.m                                               %
% Creation Date: 11/08/2015                                                    %
% Author: Edward Keehr                                                         %
%                                                                              %
% Copyright Superlative Semiconductor LLC 2021                                 %
% This source describes Open Hardware and is licensed under the CERN-OHL-P v2  %
% You may redistribute and modify this documentation and make products         %
% using it under the terms of the CERN-OHL-P v2 (https:/cern.ch/cern-ohl).     %
% This documentation is distributed WITHOUT ANY EXPRESS OR IMPLIED             %
% WARRANTY, INCLUDING OF MERCHANTABILITY, SATISFACTORY QUALITY                 %
% AND FITNESS FOR A PARTICULAR PURPOSE. Please see the CERN-OHL-P v2           %
% for applicable conditions.                                                   %
%                                                                              %
% This file tests the bit exact SDM developed for the RFIDr project            %
%                                                                              %
% 031616 - Add delays to model the SDM after timing closure.                   %
% 031818 - Get plots ready for presentation.                                   %
%                                                                              %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function test_txSDMBitExact

%%% Use Richard Schreier's Delta Sigma MATLAB Toolbox.
addpath ./delsig;
pkg load all;
more off

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%                                                                         %
% Section 0 - Simulation parameters                                       %
%                                                                         %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

Fs=36e6;                        %%% Sampling rate of SX1257
M=8;                            %%% Maximum Miller modulation index
BLF=187500;                     %%% Backscatter link frequency

rand('seed',90210);             %%% Pick arbitrary seed for rand generator
randn('seed',90210);            %%% Also need to separately seed randn generator
tag_rn16_data=rand(1,16)>0.5;   %%% RN16 data from the tag
sel_data=rand(1,161)>0.5;       %%% Select packet data
que_data=rand(1,22)>0.5;        %%% Query packet data
ack_data=tag_rn16_data;         %%% ACK packet data is the received RN16 data
Tari=1/46875;                   %%% TX Tari

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%                                                                         %
% Section 1 - Generate TX waveform and apply to SDM                       %
% Note that this will be done in the digital domain so it is just bits    %
% to bits at the input.                                                   %
%                                                                         %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

if(0)
tic
%%% Generate the RFID transmit data stream. 1's and 0's.
[rdr_strm,align_strm,blank_strm]=generateRFIDTransmit_pre(sel_data,que_data,ack_data,Tari,BLF,M,Fs);
toc
rdr_strm=round(rdr_strm);
disp(sprintf("In Length = %d",length(rdr_strm)));
%%%tic
%%% Run bit exact TX SDM
[out,y,lfsr_vec,k_qtzr,adder_hist]=txSDMBitExact(rdr_strm);
%%%toc

%%% Convert 1's and 0's to 1's and -1's.
v=round(2*out-1);

%%% Save data for explort to verilog

if(0)
fid1=fopen("./rtl_test_vectors/tx_sdm_in.dat","w");
fid2=fopen("./rtl_test_vectors/tx_sdm_out.dat","w");
fid3=fopen("./rtl_test_vectors/tx_sdm_lfsr.dat","w");

for loop_i=1:length(rdr_strm);
  fprintf(fid1,"%d\n",rdr_strm(loop_i));
  fprintf(fid2,"%d\n",out(loop_i));
  fprintf(fid3,"%d\n",lfsr_vec(loop_i));
endfor

fclose(fid1);
fclose(fid2);
fclose(fid3);
endif

save test_txsdm_results rdr_strm out lfsr_vec y v k_qtzr adder_hist;

end

load test_txsdm_results;

%ah1=adder_hist(1,1:50)
%ah2=adder_hist(2,1:50)
%ah3=adder_hist(3,1:50)
%ah4=adder_hist(4,1:50)
%ah5=adder_hist(5,1:50)
%ah6=adder_hist(6,1:50)
%ah7=adder_hist(7,1:50)
%ah8=adder_hist(8,1:50)

v=v(2:end);                 %%% Not sure why - 031616 - Sample 1 gives DC leakage
y=y(2:end);
rdr_strm=rdr_strm(2:end);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%                                                                         %
% Section 2 - Examine data to ensure that SDM is operating properly.      %
%                                                                         %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

figure(7);
%%% Plot cross-correlation of the SDM dither to make sure that the LFSR generates a maximal length pattern.
plot(xcorr(2*lfsr_vec-1,'unbiased'));

%%% Display the statistically-computed gain of the SDM quantizer.
%%% Note that the gain of a 2-level quantizer can only be computed statistically.
disp(sprintf("K of QTZR is %2.6f",k_qtzr));

%%% Generate a time vector so that plots can have an actual time axis.

timevec=(0:length(rdr_strm)-1)/Fs;

sig_idxs=intersect(find(timevec>0.00302),find(timevec<0.00782));                    %%% Isolate the indices of the output where signal is present
noise_idxs=intersect(find(timevec>0.01702),find(timevec<0.01842));                  %%% Isolate the indices of the output where only noise is present.

%%% Check to see if the quantizer gain varies much depending on what goes into the SDM

k_qtzr_sig=sum(v(sig_idxs).*y(sig_idxs))/sum(y(sig_idxs).*y(sig_idxs))              %%% Quantizer gain with large signal going into SDM.
k_qtzr_noise=sum(v(noise_idxs).*y(noise_idxs))/sum(y(noise_idxs).*y(noise_idxs))    %%% Quantizer gain with no signal going into the SDM.
k_qtzr=k_qtzr_noise;                                                                %%% Set the quantizer gain in subsequent computations to be that of when signal is passing through.

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%                                                                         %
% Section 3 - Compute STF of SDM by hand so that we can generate an ideal %
% output of the SDM unaffected by quantization noise. In this fashion we  %
% can take an FFT of the quantization noise only while signal is present  %
% and examine its properties (to make sure there are no large spurious    %
% tones generated, for example.                                           %
%                                                                         %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

b1=1;

a1=40;
a2=10;
a3=1;

c1=2;
c2=1;
c3=1;

g1=1/(2^10);

%Bstf=[0 1.2 -2.13506 0.9600];
%Astf=[1 -1.799023 0.863961 -0.04];
%k_qtzr=0.012;

Bstf=k_qtzr*b1*[0 0 0 (a1+a2*c2) (-2*a1+g1*c3*a1-a2*c2+a3*c2*c3) (a1)]; %031615 - Add two more delays to model the registers instered to close timing
Astf=[1 (-3+g1*c3+c1*k_qtzr*(a1+a2*c2)) (3-g1*c3+c1*k_qtzr*(-2*a1+g1*c3*a1-a2*c2+a3*c2*c3)) (-1+c1*k_qtzr*a1)];

rdr_strm_equiv=filter(Bstf,Astf,rdr_strm);

%%% Model the FIR filter present in the SX1257 DAC

Ntxfir=64;
Btxfir=[1];
for loop_i=1:(Ntxfir-1)
  Btxfir=conv(Btxfir,[1 -exp(-j*2*pi*loop_i/Ntxfir)]);
endfor
Btxfir=real(Btxfir/(sum(Btxfir)));

%%% Now filter both the ideal signal output and the real SDM output by the DAC FIR filter.

rdr_strm_dfilt=filter(Btxfir,[1],rdr_strm_equiv);
sdm_strm_dfilt=filter(Btxfir,[1],v);

%%% Now filter both the ideal signal and the real SDM output by the analog channel filter present in the SX1257.

[b_txlpf,a_txlpf]=butter(3,209000/(Fs/2));

rdr_strm_afilt=filter(b_txlpf,a_txlpf,rdr_strm_dfilt);
sdm_strm_afilt=filter(b_txlpf,a_txlpf,sdm_strm_dfilt);

%%% Compute the residual error signal.
err_strm_afilt=sdm_strm_afilt-rdr_strm_afilt;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%                                                                         %
% Section 4 - Plots and adder maximum register width requirement listing. %
%                                                                         %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
if(0)
figure(1); clf; hold on;
plot(timevec,rdr_strm_afilt,'b');
plot(timevec,sdm_strm_afilt,'r');
plot(timevec,err_strm_afilt,'g');

figure(2); clf; hold on;
plot(timevec,v,'b');

figure(3); clf;
pwelch(v(noise_idxs)-rdr_strm_equiv(noise_idxs),2^13,0.5,2^13,Fs,'onesided','dB');
grid on;

figure(4); clf;
pwelch(err_strm_afilt(noise_idxs),2^13,0.5,2^13,Fs,'onesided','dB');
grid on;
end
figure(5); clf;
[spectra,freq]=pwelch(v(noise_idxs),2^13,0.5,2^13,Fs,'onesided','plot','none');
freq(1)=1000; %Make sure energy at 0Hz is represented somehow
semilogx(freq,10*log10(spectra)+10*log10(freq(3)-freq(2))+6,'b','Linewidth',3); hold on; %Normalize by frequency spacing and fact that DC signal is -6dBV
[spectra,freq]=pwelch(sdm_strm_afilt(noise_idxs),2^13,0.5,2^13,Fs,'onesided','plot','none');
freq(1)=1000; %Make sure energy at 0Hz is represented somehow
semilogx(freq,10*log10(spectra)+10*log10(freq(3)-freq(2))+6,'g','Linewidth',3);
grid on;
hx=xlabel('Frequency (Hz)');
set(hx,"fontsize",14);
hx=ylabel('PSD (dBc/Hz)');
set(hx,"fontsize",14);
hx=title('Power Spectral Density of SDM During CW Output');
set(hx,"fontsize",14);
hx=legend('Raw Bits','After SX1257 Filtering','Location','Southwest');
set(hx, "linewidth", 4, "fontsize", 14,'position',[0.3 0.75 0.325 0.13]);
set(gca, "linewidth", 1, "fontsize", 14);
ylim([-140 0]);
if(0)
figure(6); clf;
[spectra,freq]=pwelch(v(noise_idxs),2^13,0.5,2^13,Fs,'onesided','plot','none');
plot(freq,10*log10(spectra),'b','Linewidth',3); hold on;
[spectra,freq]=pwelch(sdm_strm_afilt(noise_idxs),2^13,0.5,2^13,Fs,'onesided','plot','none');
plot(freq,10*log10(spectra),'g','Linewidth',3);
grid on;
end
disp(sprintf('Max bits Adder 1: %d',ceil(log10(max(max(abs(adder_hist(1,:)))))/(log10(2)))+1));
disp(sprintf('Max bits Adder 2: %d',ceil(log10(max(max(abs(adder_hist(2,:)))))/(log10(2)))+1));
disp(sprintf('Max bits Adder 3: %d',ceil(log10(max(max(abs(adder_hist(3,:)))))/(log10(2)))+1));
disp(sprintf('Max bits Adder 4: %d',ceil(log10(max(max(abs(adder_hist(4,:)))))/(log10(2)))+1));
disp(sprintf('Max bits Adder 5: %d',ceil(log10(max(max(abs(adder_hist(5,:)))))/(log10(2)))+1));
disp(sprintf('Max bits Adder 6: %d',ceil(log10(max(max(abs(adder_hist(6,:)))))/(log10(2)))+1));
disp(sprintf('Max bits Adder 7: %d',ceil(log10(max(max(abs(adder_hist(7,:)))))/(log10(2)))+1));
disp(sprintf('Max bits Adder 8: %d',ceil(log10(max(max(abs(adder_hist(8,:)))))/(log10(2)))+1));

%%% Check to see if the quantization noise of the ideal SDM matches that of the ideal SDM
%%% Currently it will not, as input dither is required to achieve a match
%%% However, the existing bit exact SDM model has dither injected into an internal node.

if(0)
  form='CRFF';
  a_q=[0.625 0.625 1];
  g_q=0.015625;
  b_q=[0.5 0 0 0];
  c_q=[0.5 0.25 0.0625];

  ABCDq=stuffABCD(a_q,g_q,b_q,c_q,form);

  [v_ideal,xn,xmax,y] = simulateDSM_ABCD(0.5*rdr_strm,ABCDq,2,0);

  figure(5); clf;
  pwelch(v_ideal(noise_idxs)-rdr_strm_equiv(noise_idxs),2^13,0.5,2^13,Fs,'onesided','dB');
  grid on;
endif

endfunction
