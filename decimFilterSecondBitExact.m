function [out,adder_hist]=decimFilterSecondBitExact(in,reset)

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%                                                                                   %
% 110315 - decimFilterSecondBitExact                                                %
%                                                                                   %
% Filename: decimFilterSecondBitExact.m                                             %
% Creation Date: 11/03/2015                                                         %
% Author: Edward Keehr                                                              %
%                                                                                   %
% Copyright Superlative Semiconductor LLC 2021                                      %
% This source describes Open Hardware and is licensed under the CERN-OHL-P v2       %
% You may redistribute and modify this documentation and make products              %
% using it under the terms of the CERN-OHL-P v2 (https:/cern.ch/cern-ohl).          %
% This documentation is distributed WITHOUT ANY EXPRESS OR IMPLIED                  %
% WARRANTY, INCLUDING OF MERCHANTABILITY, SATISFACTORY QUALITY                      %
% AND FITNESS FOR A PARTICULAR PURPOSE. Please see the CERN-OHL-P v2                %
% for applicable conditions.                                                        %
%                                                                                   %
% This file implements the fourth-order decimate-by-8 filter called for in the      %
% sdmAndDigFilterChain design file. Distributed truncation is implemented to the    %
% extent possible, in addition to the use of wrap-around logic. Test scenarios will %
% be required to exercise the wraparound logic.                                     %
%                                                                                   %
% 110415 - decimFilterBitExact110315B_oneshot                                       %
%                                                                                   %
% Make the one shot version of this filter so that we can simulate it in a feedback %
% loop with the TX cancelling array.                                                %
%                                                                                   %
% 110515 - decimFilterSecondBitExact110415_oneshot                                  %
%                                                                                   %
% This filter realizes the second decimation filter used in the TX cancellation     %
% apparatus. Notwithstanding the previous history notes, this filter is a second-   %
% order decimate-by-128 filter.                                                     %
%                                                                                   %
% 111715 - decimFilterSecondBitExact_oneshotI                                       % 
%                                                                                   %
% This file is copied over from 110415 version to be a nicely commented and         %
% formatted file. This is a CIC decimation filter, in case it is not readily        %
% apparent.                                                                         %
%                                                                                   %
% 112915 - decimFilterSecondBitExact                                                %
%                                                                                   %
% This file is copied over from 111715 version to run in an all-at-once fashion for %
% the testRXChain simulation so that we can generate test vectors for Verilog sims. %
%                                                                                   %
% 121315 - This filter is changed into an IIR filter in order to cut down on the    %
% number of adders and registers required.                                          %
%                                                                                   %
% 032816 - This filter is converted into a first-stage IIR and a second stage       %
% integrate-and-dump because it turns out that we save an interesting amount of LUT %
% this way.                                                                         %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%%% This is a decimate-by-128 filter

dec_fact=128;

%%% Initialize variables representing registers.

regi1=0;
regi2=0;

%%% Adder hist retains a history of all of the adder values so that we can properly allocate
%%% register widths in the real hardware

adder_hist=zeros(2,length(in));

%%% Preallocate output register memory to speed up for loop.

out=zeros(1,floor(length(in)/dec_fact));

for loop_a=1:length(in)

  adder_hist(1,loop_a)=bitsat(in(loop_a)+floor(63*regi1/(2^5))+1,19);
  %adder_hist(2,loop_a)=bitsat(floor(regi1/(2^4))+regi2,21);
  adder_hist(2,loop_a)=bitsat(2*in(loop_a)+regi2,21);
  %%% The +1 should make use of the carry in bit
 
  if(mod(loop_a,dec_fact)==1)
    out(floor((loop_a-1)/dec_fact)+1)=floor(regi2/(2^5));
    %regi2=bitsat(floor(regi1/(2^4)),21);
    regi2=bitsat(2*in(loop_a),21);
  else
    regi2=floor(adder_hist(2,loop_a));
  endif
%%% Update HF clock domain registers at the end of every cycle, after combinational logic has settled.   
  
  regi1=floor(adder_hist(1,loop_a)/(2^1));
  
endfor
  
endfunction  
 
function out=bitsat(in,bits)

  if(in >= 2^(bits-1))
    out=2^(bits-1)-1;
  elseif(in < -2^(bits-1))
    out=-2^(bits-1);
  else
    out=in;
  endif
  
endfunction
 

    