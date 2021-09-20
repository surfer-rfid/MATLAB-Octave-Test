function saveTunableMicrowaveNWAsASCII

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%                                                                              %
% saveTunableMicrowaveNWAsASCII - 091016                                       %
%                                                                              %
% Filename: saveTunableMicrowaveNWAsASCII.m                                    %
% Creation Date: 09/10/2016                                                    %
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
% Save Tunable Microwave Network information in a format that can be           %
% accessed by Verilog.                                                         %
%                                                                              %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%load -binary nw25_bin.mat
load -binary nw082117_simple_qucs_bin.mat

format long;

%fid_nw25_ascii_top=fopen("./rtl_test_vectors/nw25_ascii.dat","w");
fid_nw082117_simple_qucs_ascii_top=fopen("./rtl_test_vectors/nw082117_simple_qucs_ascii.dat","w");

for loop_a=1:32
  for loop_b=1:32
    for loop_c=1:32
      for loop_d=1:32
        refl_val=refl_mat(1,loop_a,loop_b,loop_c,loop_d);
        %fprintf(fid_nw25_ascii_top,"%2.12f %2.12f\n",real(refl_val),imag(refl_val));
        fprintf(fid_nw082117_simple_qucs_ascii_top,"%2.12f %2.12f\n",real(refl_val),imag(refl_val));
      endfor
    endfor
  endfor
endfor
        
%fclose(fid_nw25_ascii_top);
fclose(fid_nw082117_simple_qucs_ascii_top);