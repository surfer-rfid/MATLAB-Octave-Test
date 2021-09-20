function [out,adder_hist]=decimFilterBitExact_oneshotQ(in,reset)

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%                                                                                   %
% 110315 - decimFilterBitExact_oneshotQ                                             %
%                                                                                   %
% Filename: decimFilterBitExact_oneshotQ.m                                          %
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
% loop with the TX cancelling array. Since this file holds state at each step of    %
% the simulation, we need a separate file for both I and Q.                         %
%                                                                                   %
% 111715 - decimFilterBitExact_oneshotI                                             %
%                                                                                   %
% Make archived version from 110315B with nice style and comments                   %
% 091616 - Make same as HW version                                                  %
%                                                                                   %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%%% This is a decimate-by-8 filter

dec_fact=8;

%%% Make physical registers persistent variables

persistent regi1a=0;
persistent regi1b=1;
persistent regi1c=0;
persistent regi1d=1;
persistent regi1e=0;
persistent regi1f=1;
persistent regi1g=0;
persistent regi1h=1;
persistent regi2=0;
persistent regi3=0;
persistent regi4=0;

persistent regd0=0;
persistent regd1=0;
persistent regd2=0;
persistent regd3=0;
persistent out_reg=0;

%%% Also implement a counter that tells which phase of the high frequency clock domain should
%%% be used to generate a sample of the low frequency clock domain.

persistent dec_ctr=0;

%%% Add a provision to clear all state variables

  if(reset)
    regi1a=0;
    regi1b=1;
    regi1c=0;
    regi1d=1;
    regi1e=0;
    regi1f=1;
    regi1g=0;
    regi1h=1;
    regi2=0;
    regi3=0;
    regi4=0;

    regd0=0;
    regd1=0;
    regd2=0;
    regd3=0;
    out_reg=0;
  
    dec_ctr=0;
  endif

%%% Adder hist retains a history of all of the adder values so that we can properly allocate
%%% register widths in the real hardware  
  
  adder_hist=zeros(1,8);

  adder_hist(1)=bitroll(regi1a+regi1b+regi1c+regi1d+regi1e+regi1f+regi1g+regi1h-4,4);
  adder_hist(2)=bitroll(2*adder_hist(1)+regi2,13);
  adder_hist(3)=bitroll(regi2+regi3,13);
  adder_hist(4)=bitroll(regi3+regi4,13);
  adder_hist(5)=bitroll(regd0-regd1,13);
  adder_hist(6)=bitroll(adder_hist(5)-regd2,13);
  adder_hist(7)=bitroll(adder_hist(6)-regd3,13);
  
%%% Update LF clock domain registers at the end of every 8 cycles, after combinational logic has settled.  
  
  if(dec_ctr==0)
    out_reg=adder_hist(7);
    regd3=adder_hist(6);
    regd2=adder_hist(5);
    regd1=regd0;
    regd0=regi4;
  endif
  
%%% Update HF clock domain registers at the end of every cycle, after combinational logic has settled.  
  
  out=out_reg;
  regi4=adder_hist(4);
  regi3=adder_hist(3);
  regi2=adder_hist(2);
  regi1h=regi1g;
  regi1g=regi1f;
  regi1f=regi1e;
  regi1e=regi1d;
  regi1d=regi1c;
  regi1c=regi1b;
  regi1b=regi1a;
  if(in > 0.5)      %Convert from 1,-1 to 1,0 or 1,0 to 1,0
    regi1a=1;
  else
    regi1a=0;
  end
  
%%% The counter is modulo 8 since the decimation rate is 8  
  
  dec_ctr=mod(dec_ctr+1,dec_fact);
  
endfunction  

%%% CIC filters employ adders which roll over

function out=bitroll(in,bits)

  over=in-(2^(bits-1)-1);
  under=(2^(bits-1))+in;

  if(over > 0)
    out=-(2^(bits-1))-1+over;
  elseif(under < 0)
    out=(2^(bits-1))+under;
  else
    out=in;
  endif
  
endfunction
    