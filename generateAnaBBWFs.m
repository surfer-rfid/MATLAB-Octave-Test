function [tx_strm,rx_strm,blank_strm,reset_strm]=generateAnaBBWFs(sel_data,que_data,tag_rn16_data,ack_data,tag_pcepccrc_data,Tari,BLF,M,Fs,tag_duty_cycle)

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%                                                                              %
% 091515 - generateAnaBBWFs - generate analog baseband waveforms               %
%                                                                              %
% Filename: generateAnaBBWFs.m                                                 %
% Creation Date: 09/15/2015                                                    %
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
% Components:                                                                  %
% -TX DC or VLF (since TX and RX may not be synced)                            %
%     SDR: step is 198.36Hz, 26 MHz XTAL                                       %
%     SPIRIT1: also 26MHz XTAL, step is 33Hz                                   %
%     Quite possibly they are merely a factor of 6 off                         %
% -Phase noise                                                                 %
% -Backscatter waveform                                                        %
%                                                                              %
% 110515 - Modify file to permit external modification of TX+PN,RX complex     %
% scalings in a nested loop (thereby avoiding the requirement to generate the  %
% waveforms every time we wish to change magnitude and phase).                 %
%                                                                              %
%                                                                              %
% 111815 - Copy over to release directory and augment with comments.           %
%                                                                              %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%                                                                            %
% Section 0 - Generate TX W/F                                                %
%                                                                            %
% We can't use the SDR for both TX and RX b/c it requires switching over     %
% and recalibrating within 50us. Therefore, the TX must be another SDR or    %
% a STS1TX with an attenuator up front (with the atten driven by the FPGA)   %
%                                                                            %
% (At least this is likely the case - assume it is until proven otherwise)   %
%                                                                            %
% Nevertheless, we have 50us-67us of CW between the end of TX and start of   %
% backscatter. We should also model the TX packet since it will need to be   %
% flushed relatively quickly.                                                %
%                                                                            %
% For now, we assume that cancellation has stabilized before the TX of       %
% interest has started playing back.                                         %
%                                                                            %
% 110515 - Actually the TX will be the SX1257 as it permits full duplex.     %
%                                                                            %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

[rdr_strm,align_strm,blank_strm]=generateRFIDTransmit_pre(sel_data,que_data,ack_data,Tari,BLF,M,Fs);

%%% SX1257 TX has filtering.
%%% This is important for reducing signal content in RX frequency range.
%%% Note that RT/FT cannot exceed 0.33*Tari=7us.

%%% Design digital TX FIR, very loosely

Ntxfir=64;
Btxfir=[1];
for loop_i=1:(Ntxfir-1)
  Btxfir=conv(Btxfir,[1 -exp(-j*2*pi*loop_i/Ntxfir)]);
endfor
Btxfir=real(Btxfir/(sum(Btxfir)));

%%% Filter all of the signals, including the alignment data and the blanking data.
%%% Yes, this is very lazy, but it is an easy way to ensure that the propagation
%%% delay of the three streams matches up.

rdr_strm_dfilt=filter(Btxfir,[1],rdr_strm);
align_strm_dfilt=filter(Btxfir,[1],align_strm);
blank_strm_dfilt=filter(Btxfir,[1],blank_strm);

%%% Now, resample at anaFs.
%%% Oh wait, we actually are at anaFs, so we do nothing at the moment.

rdr_strm_ana=rdr_strm_dfilt;
align_strm_ana=align_strm_dfilt;
blank_strm_ana=blank_strm_dfilt;

%%% Filter the TX signal with a model of the SX1257 TX analog filter.

[b_txlpf,a_txlpf]=butter(3,209000/(Fs/2));

%%% Again, filters both the data and align/blanking signals together to ensure that
%%% propagation delays are matched.

rdr_strm=filter(b_txlpf,a_txlpf,rdr_strm_ana);
align_strm=filter(b_txlpf,a_txlpf,align_strm_ana);
blank_strm=filter(b_txlpf,a_txlpf,blank_strm_ana);

len_out=length(rdr_strm);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%                                                                            %
% Section 1 - Generate RFID backscatter                                      %
%                                                                            %
% Do this first, as it will dictate the length of the entire simulation      %
% Pad the backscatter in front and back in order to run simulation for the   %
% time it takes to turn around and reply.                                    %
%                                                                            %
% 110515 - Add 2 in front of generate RFID backscatter because this lets the %
% output have a power of unity. Without the "2", it's 1 0 1 0 1 0.           %
%                                                                            %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

tag_rn16_pkt_pre=2*generateRFIDBackscatter(tag_rn16_data,BLF,M,Fs,tag_duty_cycle);
reset_rn16=cat(2,zeros(1,length(tag_rn16_pkt_pre)+1000),ones(1,16),zeros(1,len_out-length(tag_rn16_pkt_pre)-1016));   %%% Make sure reset pulse is decimatable, so length of 16.
tag_rn16_pkt=cat(2,tag_rn16_pkt_pre,zeros(1,len_out-length(tag_rn16_pkt_pre)));                                       %%% Append zeros to make this packet as long as TX stream.

tag_pcepccrc_pkt_pre=2*generateRFIDBackscatter(tag_pcepccrc_data,BLF,M,Fs,tag_duty_cycle);
reset_pcepccrc=cat(2,zeros(1,length(tag_pcepccrc_pkt_pre)+1000),ones(1,16),zeros(1,len_out-length(tag_pcepccrc_pkt_pre)-1016));
%%% Make sure reset pulse is decimatable, so length of 16.
tag_pcepccrc_pkt=cat(2,tag_pcepccrc_pkt_pre,zeros(1,len_out-length(tag_pcepccrc_pkt_pre)));                           %%% Append zeros to make this packet as long as TX stream.

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%                                                                            %
% Section 2 - Assemble Signals                                               %
%                                                                            %
% Here, we find the transmit alignment points and align the tag packets to   %
% them.                                                                      %
%                                                                            %
% This setup assumes that the phase relationships are not significantly      %
% changing over time, which may not be true. In general, we will require the %
% CDR circuit to track some level of movement (phase shift and amplitude     %
% change).                                                                   %
%                                                                            %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

if(1)
figure(40); clf; hold on;
stem((1/Fs)*(0:1:length(align_strm)-1),align_strm/0.005,'g');
end

%%% Find alignment points in the alignment stream.
%%% Since the original alignment signals were filtered instead of delayed by an ideal delay,
%%% they have shrunk considerably in amplitude.
%%% Since we don't need to know where the alignment points are exactly, we just find
%%% where the alignment pulses rise above 0.0025V.

align_points=sort(intersect(find(align_strm>0.0025),find(circshift(align_strm,[0 1])<=0.0025)));

%%% Check to make sure nothing weird happened. We should get 2 and only 2 alignment points.

if(length(align_points) ~= 2)
error("Number of alignment points is incorrect");
endif

%%% Circshift the two RX packets to the empty holes that we have left for them in the TX stream.

tag_strm=(circshift(tag_rn16_pkt,[0 align_points(1)])+circshift(tag_pcepccrc_pkt,[0 align_points(2)]));
reset_strm=(circshift(reset_rn16,[0 align_points(1)])+circshift(reset_pcepccrc,[0 align_points(2)]));

%ana_bb_cplx=noisy_rdr_strm+tag_strm;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%                                                                             %
% Section 3 - Define and Apply 2nd order Butterworth filter applied in the    %
% SDR analog RX baseband.                                                     %
%                                                                             %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

[b_rxlpf,a_rxlpf]=butter(2,(0.75e6/Fs/2));
tx_strm=filter(b_rxlpf,a_rxlpf,rdr_strm);
rx_strm=filter(b_rxlpf,a_rxlpf,tag_strm);
blank_strm=filter(b_rxlpf,a_rxlpf,blank_strm);      %%% The blanking stream should be logical high or low.
blank_strm=blank_strm > 0.75;                       %%% But since we have filtered it, it is more like an analog waveform now, so we must square it up.
reset_strm=filter(b_rxlpf,a_rxlpf,reset_strm);      %%% Same thing with the reset stream - it should be high or low but now it is a rounded pulse.
reset_strm=reset_strm > 0.05;                       %%% Again, square it up so that it is treated as a logic signal at about the same width that it was before.

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%                                                                             %
% Section 4 - Check that things are more or less working properly             %
% by running spectrum analysis and plotting the results.                      %
%                                                                             %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

if(0)

timevec=(0:1:length(tx_strm)-1)/Fs;

figure(1); clf; hold on;
plot(timevec, real(tx_strm), 'b');
plot(timevec, imag(tx_strm), 'r');
plot(timevec, real(rx_strm), 'g');
plot(timevec, imag(rx_strm), 'm');

SAparams = struct();
SAparams.rate_Hz = Fs; % sampling rate of the input signal.
SAparams.pRef_W = 1e-3; % unity signal represents 0 dBm (1/1000 W).
SAparams.pNom_dBm = 0; % show 0 dBm as 0 dB.
SAparams.filter = 'brickwall';
SAparams.RBW_window_Hz = 1000; % convolve power spectrum with a 1k filter.
SAparams.RBW_power_Hz = 1; % show power density as dBc in 1 Hz.
SAparams.noisefloor_dB = -250; % don't add artificial noise.
SAparams.logscale = true; % use logarithmic frequency axis.

% plot nominal spectrum

figure(4); clf;
spectrumAnalyzer('signal', tx_strm, SAparams, 'fMin_Hz', 100, 'fig', 4);
ylabel('PSD of Reader Output');
grid on;

figure(5); clf;
spectrumAnalyzer('signal', rx_strm, SAparams, 'fMin_Hz', 100, 'fig', 5);
ylabel('PSD of Tag Output');
grid on;

figure(6); clf;
spectrumAnalyzer('signal', tag_pcepccrc_pkt_pre-mean(tag_pcepccrc_pkt_pre), SAparams, 'fMin_Hz', 100, 'fig', 6);
ylabel('PSD of Tag Output');
grid on;


endif

endfunction