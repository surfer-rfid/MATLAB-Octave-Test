function crc=crc_ccitt16_tx(in_bits)

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%                                                                               %
% 101315 - crc_ccitt16_tx                                                       %
%                                                                               %
% Filename: crc_ccitt16_tx.m                                                    %
% Creation Date: 10/13/2015                                                     %
% Author: Edward Keehr                                                          %
%                                                                               %
% Copyright Superlative Semiconductor LLC 2021                                  %
% This source describes Open Hardware and is licensed under the CERN-OHL-P v2   %
% You may redistribute and modify this documentation and make products          %
% using it under the terms of the CERN-OHL-P v2 (https:/cern.ch/cern-ohl).      %
% This documentation is distributed WITHOUT ANY EXPRESS OR IMPLIED              %
% WARRANTY, INCLUDING OF MERCHANTABILITY, SATISFACTORY QUALITY                  %
% AND FITNESS FOR A PARTICULAR PURPOSE. Please see the CERN-OHL-P v2            %
% for applicable conditions.                                                    %
%                                                                               %
% This file takes a vector of 1's and 0's and outputs a vector of 1's and 0's   %
% to append to the input vector.                                                %
% Here, we use the shift register method because we will need to transfer this  %
% to hardware soon.                                                             %
%                                                                               %
% 090716 - We had this wrong all along. Calculate this according to Appendix    %
% F in the RFID specification.                                                  %
%                                                                               %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

  shift_reg=ones(1,16);

  for loop_i=1:length(in_bits)
    fb_bit=xor(in_bits(loop_i),shift_reg(16));
    shift_reg=[fb_bit shift_reg(1:15)];
    shift_reg(6)=xor(fb_bit,shift_reg(6));
    shift_reg(13)=xor(fb_bit,shift_reg(13));
  endfor
  
  crc=fliplr(~shift_reg);

  endfunction