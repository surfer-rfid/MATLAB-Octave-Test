function [resampled_rdr_strm,resampled_align_strm,resampled_blank_strm]=generateRFIDTransmit_pre(sel_data,que_data,ack_data,Tari,BLF,M,ana_Fs)

%mms_pkt=generateRFIDTransmit([1 0 1 1 0 0 1 0 1 1 1 0 1 0 0 0],187500,4,96000000)

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%                                                                              %
% 091615 - generateRFIDTransmit_pre                                            %
%                                                                              %
% Filename: generateRFIDTransmit_pre.m                                         %
% Creation Date: 09/16/2015                                                    %
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
% This file models the RFID transmitter while a pattern is played back         %
%                                                                              %
% In this case, we model Select+Query+ACK with minimal gaps.                   %
% This permits us to examine the worst-case scenarios for cancellation         %
% convergence and for HPF settling.                                            %
% We also generate a set of alignment points to align the tag responses.       %
%                                                                              %
% 110415 - Add a 'pre' section prior to select waveform where initial cal.     %
% of the antenna reflection network can proceed.                               %
%                                                                              %
% 091116 - Alter this to match the TX on the FPGA so that we can sync this     %
% file's results to the FPGA state timing.                                     %
% Also cut the QuerAck space by 16 bits since we will be doing a RN16_I.       %
%                                                                              %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

pkg load all;
more off;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%                                                                          %
% Section 0 - Check Inputs                                                 %
%                                                                          %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%%% LOL!!! Just kidding! We didn't do it!

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%                                                                          %
% Section 1 - Define Unit Symbols used in Reader -> Tag Communications     %
%                                                                          %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

upsamp_factor=round(ana_Fs*Tari/16);                                  %%% We need to upsample because the low pulse width lasts for slightly less than half of the data 0 period.
intermediate_Fs=upsamp_factor/(Tari/16);                              %%% This upsampling also implies an intermediate sampling rate.
%Tpri=Tari*(85/16)*(3/64) or about 4/16 Tari

mod_depth=1;                                                          %%% Why not? It minimizes the bit width of the SDM, that's for sure.

Delimiter=(1-mod_depth)*ones(1,round(intermediate_Fs*12.5e-6));       %%% Delimiter goes on for 12.5us.
PW=(1-mod_depth)*ones(1,7*upsamp_factor);                             %%% PW is 7/16 Tari.
Data0=cat(2,ones(1,9*upsamp_factor),PW);                              %%% 1 Tari is 16/16 Tari.
Data1=cat(2,ones(1,21*upsamp_factor),PW);                             %%% 1.75 Tari is 28/16 Tari.
RTCal=cat(2,ones(1,37*upsamp_factor),PW);                             %%% 2.75 Tari is 44/16 Tari.
TRCal=cat(2,ones(1,78*upsamp_factor),PW);                             %%% 5.3125 Tari is 85/16 Tari.
PreSpace=ones(1,4095*upsamp_factor);                                  %%% 112617 - Add more time. Add spacing prior to sending out any packets so that the TX cancellation algorithm can converge.
SelQuerSpace=ones(1,6*16*upsamp_factor);                              %%% T4 > 6 Tari > 2*RTCal.
QuerAckSpace=ones(1,3*16*upsamp_factor+15*4*upsamp_factor+ceil((187500/BLF)*M*(16+6+16+1)*4*upsamp_factor));                                     %%% RTCal+15Tpri+(16+6+16+1)Tpri (RN16_I).
QuerAlignPoint=cat(2,zeros(1,3*16*upsamp_factor),1,zeros(1,15*4*upsamp_factor+ceil((187500/BLF)*M*(16+6+16+1)*4*upsamp_factor)-1));              %%% Set alignment point for RX packet in sim.
AckAfterward=cat(2,ones(1,3*16*upsamp_factor+ceil((187500/BLF)*M*(16+6+16+96+16+1)*4*upsamp_factor)),QuerAckSpace);                                 %%% RTCal+(16+6+16+96+16+1)Tpri (CP+EPC+CRC).
AckAlignPoint=cat(2,zeros(1,3*16*upsamp_factor),1,zeros(1,ceil((187500/BLF)*M*(16+6+16+96+16+1)*4*upsamp_factor)-1),zeros(1,length(QuerAckSpace))); %%% Set alignment point for RX packet in sim.

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%                                                                          %
% Section 2 - Put Together TX Sub-Patterns                                 % 
%                                                                          %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

sel_pattern=[];

%%% Look at the input data bit by bit and generate the waveform with proper low duty cycle.

for loop_i=1:length(sel_data)
  if(sel_data(loop_i)==0)
    sel_pattern=cat(2,sel_pattern,Data0);
  elseif(sel_data(loop_i)==1)
    sel_pattern=cat(2,sel_pattern,Data1);
  else
    error("Bad value in Select Pattern");
  endif  
endfor

%%% Put together the entire waveform, beginning with a Frame Sync, and ending with a dummy 1.

sel_waveform=cat(2,Delimiter,Data0,RTCal,sel_pattern,Data1);

que_pattern=[];

%%% Look at the input data bit by bit and generate the waveform with proper low duty cycle.

for loop_i=1:length(que_data)
  if(que_data(loop_i)==0)
    que_pattern=cat(2,que_pattern,Data0);
  elseif(que_data(loop_i)==1)
    que_pattern=cat(2,que_pattern,Data1);
  else
    error("Bad value in Query Pattern");
  endif
endfor
    
%%% Put together the entire waveform, beginning with a Preamble, and ending with a dummy 1.
    
que_waveform=cat(2,Delimiter,Data0,RTCal,TRCal,que_pattern,Data1);

ack_pattern=[];

%%% Look at the input data bit by bit and generate the waveform with proper low duty cycle.

for loop_i=1:length(ack_data)
  if(ack_data(loop_i)==0)
    ack_pattern=cat(2,ack_pattern,Data0);
  elseif(ack_data(loop_i)==1)
    ack_pattern=cat(2,ack_pattern,Data1);
  else
    error("Bad value in Ack Pattern");
  endif  
endfor    
    
%%% Put together the entire waveform, beginning with a Frame Sync, and ending with a dummy 1.
    
ack_waveform=cat(2,Delimiter,Data0,RTCal,ack_pattern,Data1);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%                                                                           %
% Section 3 - Put together entire TX Pattern                                %
%                                                                           %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%%% The reader stream is the {0,1} data stream that goes to the SDM.

intermediate_rdr_strm=cat(2,PreSpace,sel_waveform,SelQuerSpace,que_waveform,QuerAckSpace,ack_waveform,AckAfterward);
[resampled_rdr_strm,Haaf]=resample(intermediate_rdr_strm,ana_Fs,intermediate_Fs);

%%% The align stream is a stream with two pulses that denote where the RX packets should be placed in a composite data stream.

intermediate_align_strm=cat(2,zeros(1,length(PreSpace)+length(sel_waveform)+length(SelQuerSpace)+length(que_waveform)),QuerAlignPoint,zeros(1,length(ack_waveform)),AckAlignPoint);
[resampled_align_strm,Haaf]=resample(intermediate_align_strm,ana_Fs,intermediate_Fs);

%%% The blank stream is a stream that allows for blanking of the RX signal path while the TX is in operation,
%%% permitting fast recovery of the RX signal path after the TX playback completes.

intermediate_blank_strm=cat(2,zeros(1,length(PreSpace)+length(sel_waveform)+length(SelQuerSpace)+length(que_waveform)),ones(1,length(QuerAckSpace)),zeros(1,length(ack_waveform)),ones(1,length(AckAfterward)));
[resampled_blank_strm,Haaf]=resample(intermediate_blank_strm,ana_Fs,intermediate_Fs);

endfunction