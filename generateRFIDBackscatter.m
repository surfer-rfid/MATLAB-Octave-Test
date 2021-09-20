function resampled_mms_pkt=generateRFIDBackscatter(seq,BLF,M,ana_Fs,duty_cycle)

%mms_pkt=generateRFIDBackscatter([1 0 1 1 0 0 1 0 1 1 1 0 1 0 0 0],187500,4,96000000)

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%                                                                              %
% 091615 - generateRFIDBackscatter                                             %
%                                                                              %
% Filename: generateRFIDBackscatter.m                                          %
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
% This file models the RFID tag backscatter while a pattern is played back     %
%                                                                              %
% This file assumes that a pilot tone is generated.                            %
% This file also assumes MMS coding, but permits use of different              %
% M values.                                                                    %
% This file assumes that tag backscatter coding starts in the low-power        %
% state (power absorbing state) because this is what the RFID spec.            %
% depicts and what makes sense. A proviso is added to switch this around.      %
% Phase is in radians.                                                         %
%                                                                              %
% 101715 - Add capability of altering duty cycle to +/- 5%.                    %
%                                                                              %
% 111715 - Save release version in release directory.                          %
% Added more comments and better code formatting where appropriate.            %
%                                                                              %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

more off;
pkg load all;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%                                                                          %
% Section 0 - Check Inputs                                                 %
%                                                                          %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%%% This is the only input checked for now.
%%% Maybe later we will put in more checks!!!

if((duty_cycle != 0.45) && (duty_cycle != 0.5) && (duty_cycle != 0.55))
  disp("Bad duty cycle entered");
endif

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%                                                                          %
% Section 1 - Define Manchester Symbols                                    %
%                                                                          %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%%% Effectively add an upsampling of 20 here, which is what we need in order
%%% to set the duty cycle to one of the extreme values.

if(duty_cycle==0.45)
  mms_unit=[ones(1,18) zeros(1,22)];
elseif(duty_cycle==0.5)
  mms_unit=[ones(1,20) zeros(1,20)];
elseif(duty_cycle==0.55)
  mms_unit=[ones(1,22) zeros(1,18)];
else
  disp("I said: bad duty cycle entered!");
endif  

%%% Generate the bit symbols as defined in the UHF RFID specification.
%%% There are 4 states to the Miller modulation state diagram.
%%% 2 states correspond to a logic zero bit, while two states correspond to a logic one bit.

mms_zro_s1=repmat(mms_unit,1,M);
mms_zro_s4=1-mms_zro_s1;
mms_one_s2=[repmat(mms_unit,1,M/2) fliplr(repmat(mms_unit,1,M/2))];
mms_one_s3=1-mms_one_s2;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%                                                                          %
% Section 2 - Construct Backscattered Packet                               %
%                                                                          %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

pilot=repmat(mms_zro_s1,1,16);  %%% Here, we take advantage of the large pilot signal to permit the PLL to lock.
preamble=[0 1 0 1 1 1];         %%% The preamble is defined by the UHF RFID specification.
packet=cat(2,preamble,seq,1);   %%% Concatenate the preamble, data sequence, and trailing 1 bits to complete the packet.

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%                                                                          %
% Section 3 - Map Packet onto MMS state space                              %
%                                                                          %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

mms_pkt=pilot;                  %%% Since the pilot signal is already modulated, we keep it as it is.
next_state=4;                   %%% Initialize the state machine

for loop_i=1:length(packet)

state=next_state;

switch(state)
  case 1
    if(packet(loop_i)==0)
      mms_pkt=cat(2,mms_pkt,mms_zro_s4);
      next_state=4;
    elseif(packet(loop_i)==1)
      mms_pkt=cat(2,mms_pkt,mms_one_s2);
      next_state=2;
    else
      error("Backscatter packet values can only be 0 or 1");
    endif
  case 2
    if(packet(loop_i)==0)
      mms_pkt=cat(2,mms_pkt,mms_zro_s4);
      next_state=4;
    elseif(packet(loop_i)==1)
      mms_pkt=cat(2,mms_pkt,mms_one_s3);
      next_state=3;
    else
      error("Backscatter packet values can only be 0 or 1");
    endif
  case 3
    if(packet(loop_i)==0)
      mms_pkt=cat(2,mms_pkt,mms_zro_s1);
      next_state=1;
    elseif(packet(loop_i)==1)
      mms_pkt=cat(2,mms_pkt,mms_one_s2);
      next_state=2;
    else
      error("Backscatter packet values can only be 0 or 1");
    endif
  case 4
    if(packet(loop_i)==0)
      mms_pkt=cat(2,mms_pkt,mms_zro_s1);
      next_state=1;
    elseif(packet(loop_i)==1)
      mms_pkt=cat(2,mms_pkt,mms_one_s3);
      next_state=3;
    else
      error("Backscatter packet values can only be 0 or 1");
    endif
  otherwise
    error("Invalid State");
endswitch

endfor

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%                                                                          %
% Section 4 - Map backscatter signal to analog Fs                          %
% Note that BLF is the inverse of the period of a 1-0                      % 
%                                                                          %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

upsamp_factor=20;                             %%% This reflects the upsampling done earlier at each BLF clock cycle.
intermediate_Fs=upsamp_factor*(2*BLF);        %%% This computes the effective sampling rate we have established with our choice of signaling so far.

upsampled_mms_pkt=mms_pkt;
[resampled_mms_pkt,Haaf]=resample(upsampled_mms_pkt,ana_Fs,intermediate_Fs);  %%% This resamples the constructed signal to the simulation sampling rate.
%time_vec=(0:1:length(resampled_mms_pkt)-1)/ana_Fs;

endfunction