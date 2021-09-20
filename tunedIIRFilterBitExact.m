function [out,adder_B1_hist,adder_B2_hist]=tunedIIRFilterBitExact(in)

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%                                                                              %
% tunedIIRFilterBitExact                                                       %
%                                                                              %
% Filename: tunedIIRFilterBitExact.m                                           %
% Creation Date: 10/27/2015                                                    %
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
% Here, we perform saturation and rounding of the components of our            %
% channel filtering. The input and output are 16 bits, but the internal        %
% bitwidth is 18 bits in order to avoid boosting quantization noise from       %
% sections of the circuit which contain droop prior to compensation in         %
% later stages.                                                                %
%                                                                              %
% Made entire system biquad-based for more accuracy, cleanness of arch         %
% itecture.                                                                    %
%                                                                              %
% 102815C - Got the file to work at steady state in "B" version.               %
% in this file attempt to minimize required registers.                         %
% 102915 - In previous version permitted concise examination of global         %
% register maximums. In this file, attempt to reduce initial error             %
% 110215 - Get dual tuned integrator version to work                           %
% 110215E - Account for fact that non delayed integrators make ineff.          %
% use of LUTs in the FPGA.                                                     %
% 110415 - Made another version that was included in higher level sims.        %
% 111715 - Improved formatting and comments - stored in release directory      %
% 121115 - Switch to Direct Type II architecture.                              %
%                                                                              %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%%% Set parameters of the filter

scale=6;  %%% The constant (2^scale) is used to multiply the coefficients. We must divide by (2^scale) after each integrator 
bits=18;  %%% We didn't really end up using this for anything - it was meant to put a limit on the filter adder bit width

A1_1=122; %%% This multiplication coefficient can be realized with one adder
A2_1=61;  %%% This multiplication coefficient can be realized with one adder

A1_2=118; %%% This multiplication coefficient can be realized with two adders
A2_2=60;  %%% This multiplication coefficient can be realized with one adder

% total adders: 7 (including zero adders)

%%% Allocate memory so that the loop moves quickly.

out=zeros(1,length(in));

%%% Store the adder states for examination later - we want to see how many bits per adder/register are truly required.

adder_B1_hist=zeros(3,length(in));
adder_B2_hist=zeros(3,length(in));

%%% Clear persistent variables of the subfilters 

[dum1,dum2]=dfII1(0,0,0,bits,scale,1);
[dum1,dum2]=dfII2(0,0,0,bits,scale,1);

%%% Run the entire filter, which is broken up into submodules

for loop_a=1:length(in)

  [outBq1,adder_B1]=dfII1(8*in(loop_a),A1_1,A2_1,bits,scale,0);    %%% Implements the first biquad (poles) of the filter
  [outBq2,adder_B2]=dfII2(outBq1,A1_2,A2_2,bits,scale,0);          %%% Implements the second biquad (poles) of the filter
  out(loop_a)=bitsat(outBq2,32);
  
  adder_B1_hist(:,loop_a)=adder_B1;                                %%% Stores the adder variables
  adder_B2_hist(:,loop_a)=adder_B2;                                %%% Stores the adder variables

endfor

endfunction

function [out,adder]=dfII1(in,A1,A2,bits,scale,clear)

  %%% Declare the registers as persistent variables.

  persistent stg1_reg=0;
  persistent stg2_reg=0;

  %%% Clear the registers when calling the top level filter function for the first time
  
  if(clear)
    stg1_reg=0;
    stg2_reg=0;
  endif
  
  %%% The structure shown herein follows the diagram shown on notes p.582
  %%% Compute the adder outputs for each clock cycle
  
  adder=zeros(1,3);
  
  adder(2)=bitsat(floor(stg1_reg*A1/(2^5))-floor(stg2_reg*A2/(2^5)),32); 
  adder(1)=bitsat(floor((in+adder(2))/(2^1)),18);
  adder(3)=bitsat(floor(adder(1)/(2^2))-floor(stg2_reg/(2^2)),32);
  out=adder(3);
  
  %%% Update registers
  
  stg2_reg=stg1_reg;
  stg1_reg=adder(1);
  
endfunction

function [out,adder]=dfII2(in,A1,A2,bits,scale,clear)

  %%% Declare the registers as persistent variables.

  persistent stg1_reg=0;
  persistent stg2_reg=0;

  %%% Clear the registers when calling the top level filter function for the first time
  
  if(clear)
    stg1_reg=0;
    stg2_reg=0;
  endif
  
  %%% The structure shown herein follows the diagram shown on notes p.582
  %%% Compute the adder outputs for each clock cycle
  
  adder=zeros(1,3);
  
  adder(2)=bitsat(floor(stg1_reg*A1/(2^5))-floor(stg2_reg*A2/(2^5)),32); 
  adder(1)=bitsat(floor((in+adder(2))/(2^1)),18);
  adder(3)=bitsat(floor((adder(1)-stg2_reg)/(2^2)),32);
  out=adder(3);
  
  %%% Update registers
  
  stg2_reg=stg1_reg;
  stg1_reg=adder(1);
  
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