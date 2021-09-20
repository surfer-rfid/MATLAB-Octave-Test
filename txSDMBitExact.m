function [v,y,lfsr_vec,k_qtzr,adder_hist]=txSDMBitExact(in)

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%                                                                                   %
% 110815 - txSDMBitExact                                                            %
%                                                                                   %
% Filename: txSDMBitExact.m                                                         %
% Creation Date: 11/04/2015                                                         %
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
%                                                                                   %
% Here, we simulate the TX SDM in bit-exact format.                                 %
%                                                                                   %
% 110915 - getting funny behavior - try to debug                                    %
%                                                                                   %
% 111015 - Added internal dither.                                                   %
% 111715 - Copy to release directory, clean up and add proper commenting.           %
% 112515 - Need to add a register on "out" at some point to match Verilog.          %
% 031616 - Move registers around to improve timing. Also bypass sats in ff path.    %
%                                                                                   %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%                                                                                   %
% Section 0 - SDM Internal Multiplier Coefficients.                                 %
%                                                                                   %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

b1=1;

a1=40;
a2=10;
a3=1;

c1=2;
c2=2^10;
c3=1;

g1=1;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%                                                                                   %
% Section 1 - Allocate memory for output data, specify LFSR parameters.             %
%                                                                                   %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%%% SDM Registers

in_reg=0;
reg1=0;
reg2=0;
reg3=0;
reg4=0;
reg5=0;
reg6=0;
out_reg=0;

v=zeros(1,length(in));          %%% Quantizer output data
y=zeros(1,length(in));          %%% Quantizer input data
lfsr_vec=zeros(1,length(in));   %%% LFSR output data for analysis
adder_hist=zeros(8,length(in)); %%% Adder history so that we can see what width of adders and registers to use
lfsr_mask=2^18-1;               %%% We use an 18-bit LFSR
lfsr=2^18-1;                    %%% We pick the all-ones vector as the arbitrary starting seed of the LFSR

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%                                                                                   %
% Section 2 - Run SDM                                                               %
%                                                                                   %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
id=tic;
for loop_a=1:length(in)

  if(mod(loop_a,10000) == 0)
    toc(id);
    disp(sprintf("Loop: %d",loop_a));
  endif
    
  lfsr_vec(loop_a)=xor(bitand(2^17,lfsr),bitand(2^10,lfsr));            %%% The LFSR is used as a PRBSG to dither the SDM.
  lfsr=bitand(lfsr_mask,lfsr*2);                                        %%% Otherwise some funky tones come out when a DC signal is applied.
  lfsr+=lfsr_vec(loop_a);                                               %%% Change to match our verilog implementation    

  adder_hist(8,loop_a)=bitroll(reg4+reg5,12);
  v_int=adder_hist(8,loop_a) >= 0;                                      %%% Perform the 2-level quantization. 031316 - Make it >= 0
  y(loop_a)=reg6;
  v(loop_a)=out_reg;
  adder_hist(1,loop_a)=bitroll(b1*in_reg-2*c1*v_int+2,6);               %%% Since v is computed combinationally, and since the input is not
  adder_hist(2,loop_a)=bitroll(adder_hist(1,loop_a)+reg1,7);            %%% directly involved in computing "v", the input processesing happens at the end.
  ah2bs=bitsat(adder_hist(2,loop_a),6);
  adder_hist(3,loop_a)=bitroll(c2*ah2bs-g1*reg3,17);
  adder_hist(4,loop_a)=bitroll(adder_hist(3,loop_a)+reg2,18);
  ah4bs=bitsat(adder_hist(4,loop_a),17);
  adder_hist(5,loop_a)=bitroll(c3*floor(ah4bs/1024)+reg3,11);           %%% This is the only other quantization in the SDM
  ah5bs=bitsat(adder_hist(5,loop_a),10);
  %adder_hist(6,loop_a)=bitroll(a3*reg3+floor(adder_hist(4,loop_a)/128)+floor(adder_hist(4,loop_a)/512),12);        %%% This is the only other quantization in the SDM
  %adder_hist(7,loop_a)=bitroll(a1*reg1+adder_hist(6,loop_a),12);
  %adder_hist(8,loop_a)=bitsat(4*(2*lfsr_vec(loop_a)-1)+adder_hist(7,loop_a),11);
  adder_hist(6,loop_a)=bitroll(a3*reg3+a1*ah2bs+8*lfsr_vec(loop_a),12);
  %adder_hist(6,loop_a)=bitroll(a3*reg3+a1*reg1+8*lfsr_vec(loop_a),12); %%% Doesn't work - horrible results
  adder_hist(7,loop_a)=bitroll(floor(ah4bs/128)+floor(ah4bs/512)-4,12);
  
  in_reg=in(loop_a);
  reg1=ah2bs;                                                           %%% Update registers after the combinational logic has settled.
  reg2=ah4bs;
  reg3=ah5bs;
  reg4=adder_hist(6,loop_a);
  reg5=adder_hist(7,loop_a);
  reg6=adder_hist(8,loop_a);
  out_reg=v_int;
  
endfor

k_qtzr=sum((2*v-1).*y)/sum(y.*y);                                       %%% Compute quantizer gain
  
endfunction 

%%% The integrator outputs must be saturated!
%%% Integrator rollover will be disastrous for distortion, regulatory compliance

function out=bitsat(in,bits)

  if(in >= 2^(bits-1))
    out=2^(bits-1)-1;
  elseif(in < -2^(bits-1))
    out=-2^(bits-1);
  else
    out=in;
  endif
  
endfunction 

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



    