function [out,adder_hist]=decimFilterBitExact(in)

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%                                                                                   %
% 110315 - decimFilterBitExact                                                      %
%                                                                                   %
% Filename: decimFilterBitExact.m                                                   %
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
% 111715 - decimFilterBitExact                                                      %
%                                                                                   %
% Copied from 110315B version and placed into release directory for cleanup and     %
% commenting.                                                                       %
%                                                                                   %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%%% This is a decimate by 8 filter.

dec_fact=8;

%%% Initialize variables representing registers.

regi1a=1;
regi1b=0;
regi1c=1;
regi1d=0;
regi1e=1;
regi1f=0;
regi1g=1;
regi1h=0;
regi1i=1;
regi2=0;
regi3=0;
regi4=0;

regd0=0;
regd1=0;
regd2=0;
regd3=0;

%%% Adder hist retains a history of all of the adder values so that we can properly allocate
%%% register widths in the real hardware

adder_hist=zeros(6,length(in));

%%% Preallocate output register memory to speed up for loop.

out=zeros(1,floor(length(in)/dec_fact));

for loop_a=1:length(in)

  adder_hist(2,loop_a)=bitroll(2*(regi1b+regi1c+regi1d+regi1e+regi1f+regi1g+regi1h+regi1i-4)+regi2,13);
  adder_hist(3,loop_a)=bitroll(regi2+regi3,13);
  adder_hist(4,loop_a)=bitroll(regi3+regi4,13);
  
  if(mod(loop_a,dec_fact)==1)
    adder_hist(5,loop_a)=bitroll(regd0-regd1,13);
    adder_hist(6,loop_a)=bitroll(adder_hist(5,loop_a)-regd2,13);
    adder_hist(7,loop_a)=bitroll(adder_hist(6,loop_a)-regd3,13);
    out(floor((loop_a-1)/dec_fact)+1)=adder_hist(7,loop_a);
 
%%% Update LF clock domain registers at the end of every 8 cycles, after combinational logic has settled. 
  
    regd3=adder_hist(6,loop_a);
    regd2=adder_hist(5,loop_a);
    regd1=regd0;
    regd0=regi4;                    %%% This is to model clock-streching structure used for clock domain crossing
  else
    adder_hist(5,loop_a)=adder_hist(5,loop_a-1);
    adder_hist(6,loop_a)=adder_hist(6,loop_a-1);
    adder_hist(7,loop_a)=adder_hist(7,loop_a-1);
  endif
  
%%% Update HF clock domain registers at the end of every cycle, after combinational logic has settled.   
  
  regi4=adder_hist(4,loop_a);
  regi3=adder_hist(3,loop_a);
  regi2=adder_hist(2,loop_a);
  regi1i=regi1h;
  regi1h=regi1g;
  regi1g=regi1f;
  regi1f=regi1e;
  regi1e=regi1d;
  regi1d=regi1c;
  regi1c=regi1b;
  regi1b=regi1a;
  regi1a=in(loop_a); 
  
endfor
  
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
    