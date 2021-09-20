function [out,adder_hist]=decimFilterSecondBitExact_oneshotQ(in,reset)

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%                                                                                   %
% 110315 - decimFilterSecondBitExact_oneshotQ                                       %
%                                                                                   %
% Filename: decimFilterSecondBitExact_oneshotQ.m                                    %
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
% apparent. Note that since we have both I and Q sets of filters and since these    %
% files hold state between loop iterations, we need files for both I and Q.         %
%                                                                                   %
% 122815 - Modified to be in compliance with reduced-LUT full-run design            %
% developed in January                                                              %
%                                                                                   %
% 091616 - Modified to represent actual hardware                                    %
%                                                                                   %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%%% This is a decimate-by-128 filter

dec_fact=128;

%%% Make physical registers persistent variables

persistent regi1=0;
persistent regi2=0;
persistent out=0;

%%% Also implement a counter that tells which phase of the high frequency clock domain should
%%% be used to generate a sample of the low frequency clock domain.

persistent dec_ctr=0;

%%% Add a provision to clear all state variables

  if(reset)
    regi1=0;
    regi2=0;
    out=0;
    dec_ctr=0;
  endif

%%% Adder hist retains a history of all of the adder values so that we can properly allocate
%%% register widths in the real hardware   
  
  adder_hist=zeros(1,2);

  adder_hist(1)=bitsat(in+floor(63*regi1/(2^5))+1,19);
  %adder_hist(2)=bitsat(floor(regi1/(2^4))+regi2,21);
  adder_hist(2)=bitsat(2*in+regi2,21);
 
%%% Update LF clock domain registers at the end of every 8 cycles, after combinational logic has settled.  
 
  if(dec_ctr==0)
    %out=floor(adder_hist(1)/(2^3));
    out=floor(regi2/(2^5)); %debug 121916
    %regi2=bitsat(floor(regi1/(2^4)),21);
    regi2=bitsat(2*in,21);
  else
    regi2=floor(adder_hist(2));
  endif
%%% Update HF clock domain registers at the end of every cycle, after combinational logic has settled.   
  
  regi1=floor(adder_hist(1)/(2^1));
  
%%% The counter is modulo 128 since the decimation rate is 128    
  
  dec_ctr=mod(dec_ctr+1,dec_fact);
  
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
    