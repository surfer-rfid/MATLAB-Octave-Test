function test_rfidCDR_comprehensive

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%                                                                                   %
% 101315 - test_rfidCDR_comprehensive                                               %
%                                                                                   %
% Filename: test_rfidCDR_comprehensive.m                                            %
% Creation Date: 10/13/2015                                                         %
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
% This file creates a backscatter-only signal to send to the CDR with some idealized%
% impairments (noise, phase rotation, different gain values, etc.)                  %
%                                                                                   %
% 111715 - Copy to release directory, clean up and improve commenting.              %
% 121615 - Modify to support new channel filtering and rfidCDR.                     %
% 022316 - Get it to work with Lattice simulation, new DR features in RTL.          %
% This includes in the output file, include the correct RAM spacing. The test vector%
% right now is not quite correct, as it is a Req_RN return packet followed by a     %
% PCEPC (ACK) returnb packet.                                                       %
% 031116 - Incorporate new CDR circuit to reduce Lattice LUT count.                 %
% 032416 - Updated to work with new DR interface (radio_state not rx addresses).    %
% 021318 - Check what SNR is required to demodulate signal half the time.           %
%                                                                                   %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

more off; % so that we can see debugging messages as they happen.
%%% Use Richard Schreier's Delta Sigma MATLAB Toolbox.
addpath ./delsig;
pkg load all;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%                                                                                   %
% Section 0 - Define loops related to generating backscatter patterns               %
%                                                                                   %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%%% Define the backscatter link frequency in terms of the BLF period.

%BLF_vec=100*round(2250000./([10 10.5 11 11.5 12 12.5 13 13.5 14])/100);
%BLF_vec=100*round(2250000./([10.5])/100);
BLF_vec=100*round(2250000./([13.5])/100);

%%% Make sure to check what happens during extreme duty cycles (effectively, period chattering every clock cycle).

%duty_cycle_vec=[0.45 0.5 0.55];
duty_cycle_vec=0.45;

%%% Test that the CORDIC algorithm in the FPGA can successfully determine the phase angle.

%angle_deg_vec=rand(1,10)*360;
angle_deg_vec=1;
angle_rad_vec=pi*angle_deg_vec/180;

%%% Full scale backscatter amplitude is -20dBm (scaling factor of "1" relative to full scale).
%%% Minimum backscatter amplitude is -70dBm (scaling factor of "0.00031" relative to full scale).
%%% Here, we check the bottom end - since we normalize up by 20dB, it's -40 to -50 dB.

%ampl_vec_dB=-41:-1:-50;
%ampl_vec_dB=-51; 
ampl_vec_dB=-53;                                    %%% This is in terms of dBFS.
ampl_vec=10.^(ampl_vec_dB/20);                      %%% Convert to an actual amplitude.
num_noise=1;                                        %%% The number of different noise waveform realizations to be run.
noise_offset=1;                                     %%% Can be used to select a particular noise waveform realization for examination.

load -mat genNotchBBFTotal121115;                   %%% Load filter coefficients for an ideal filter (it runs faster).

results_toplevel=cell(length(BLF_vec),length(duty_cycle_vec),length(angle_rad_vec),length(ampl_vec_dB));  %%% Create a single structure to hold the results of the simulation.

for loop_blf_a=1:length(BLF_vec)                    %%% Loop backscatter frequency.
  for loop_dc_b=1:length(duty_cycle_vec)            %%% Loop duty cycle.
    for loop_angle_c=1:length(angle_rad_vec)        %%% Loop angle.

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%                                                                                   %
% Section 1 - Generate input waveforms                                              %
%                                                                                   %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

      Fs=4.5e6;                                     %%% SDM sampling rate, prior to decimation.
      M=8;                                          %%% Miller modulation index for the backscattered waveform.

      rand("seed", 3285);                           %%% Seed the random number generator so that we get the same results for same simulation settings.

      tag_rn16_data=rand(1,16)>0.5;                 %%% Generate RN16 data.
      tag_pcepc_data=rand(1,112)>0.5;               %%% Generate PCEPC data.
      rn16_crc=crc_ccitt16_tx(tag_rn16_data);       %%% Generate CRC data for RN16.
      pcepc_crc=crc_ccitt16_tx(tag_pcepc_data);     %%% Generate CRC data for PCEPC.

      tag_rn16_packet=[tag_rn16_data rn16_crc];     %%% Concatenate RN16 CRC with RN16 data.
      tag_pcepc_packet=[tag_pcepc_data pcepc_crc];  %%% Concatenate PCEPC data with PCEPC CRC.

      resampled_rn16_pkt=2*generateRFIDBackscatter(tag_rn16_packet,BLF_vec(loop_blf_a),M,Fs,duty_cycle_vec(loop_dc_b));     %%% Multiply by 2 since this function outputs 0,1,0,1.
      resampled_pcepc_pkt=2*generateRFIDBackscatter(tag_pcepc_packet,BLF_vec(loop_blf_a),M,Fs,duty_cycle_vec(loop_dc_b));   %%% Multiply by 2 since this function outputs 0,1,0,1.

      resampled_rn16_pkt-=mean(resampled_rn16_pkt); %%% Remove DC from each returned packet.
      resampled_pcepc_pkt-=mean(resampled_pcepc_pkt);

      %%% Assemble backscatter packets into a time vector where spacing is similar to that dictated by the UHF RFID specification.
      
      backscatter_data=[zeros(1,length(resampled_rn16_pkt)) resampled_rn16_pkt zeros(1,length(resampled_rn16_pkt)) resampled_pcepc_pkt zeros(1,length(resampled_rn16_pkt))];
      backscatter_data=circshift(backscatter_data,[0 -1]);
      
      %%% Clear data is the waveform that mimics the CDR reset pulses from the MCU.
      
      clear_data=[ones(1,12) zeros(1,2*length(resampled_rn16_pkt)+388) 1 zeros(1,length(resampled_rn16_pkt)-401) zeros(1,length(resampled_pcepc_pkt)+400) 1 zeros(1,length(resampled_rn16_pkt)-401)];
      clear_data=circshift(clear_data,[0 -22]); %%% 022316 - move this here to avoid repeated shifts on loop_e.
      clear_data(20)=1;
 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%                                                                                   %
% Section 2 - Apply scalings to data and noise                                      %
%                                                                                   %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

      for loop_ampl_d=1:length(ampl_vec)            %%% Loop amplitude of signal.
        
        bit_errors=0;                               %%% Declare and reset BER statistics.
        bit_total=0;                                %%% We declare these variables here because
        pkt_errors=0;                               %%% we want to effectively average all of these variables
        pkt_total=0;                                %%% over many noise waveform realizations.
        pk_ampl_err=0;
        pk_angle_err=0;
        
        for loop_noise_e=1:num_noise                %%% Loop noise waveform realization.
          tic
          randn('seed', loop_noise_e+noise_offset); %%% Ensure that all noise generated in this section is repeatable by entering in
          rand('seed', loop_noise_e+noise_offset);  %%% the proper seed parameters.
          [noise_cplx]=generateRFIDPhaseNoise(length(backscatter_data),Fs); %%% This function provides noise out at -100dBc/Hz at 100kHz.
      
            %%% Full scale as seen by the digital back end will be -20dBm.
            %%% We expect cancelled DC signal to be -30dBm.
            %%% So carrier with respect to the full-scale signal is -10dB.
            %%% So we multiply noise by -10dB.

            noiseI_init=0.316*real(noise_cplx);
            noiseQ_init=0.316*imag(noise_cplx);

            %%% Also rotate the noise vectors to give a bit more of a 'fair' balance between I and Q.
            %%% This is actually damaging to us because it guarantees that neither I or Q has noise less clock recovery.

            noiseI=0.707*noiseI_init-0.707*noiseQ_init;
            noiseQ=0.707*noiseQ_init+0.707*noiseI_init;
      
            %%% Turn off noise for temp debugging.
            
            if(0)
              noiseI=0;
              noiseQ=0;
            endif
        
            %%% Assemble the complete RX waveform.
        
            %%% Scale signal so that -50dBFS is really -50dBFS for a 16-bit signal.
            %%% Note that because of the resampling in the waveform generation, 0dBFS signal do exceed 16 bits, but we ignore this issue for now.
        
            back_data_I=round(32768*(ampl_vec(loop_ampl_d)*backscatter_data*cos(angle_rad_vec(loop_angle_c))+noiseI));
            back_data_Q=round(32768*(ampl_vec(loop_ampl_d)*backscatter_data*sin(angle_rad_vec(loop_angle_c))+noiseQ));
            
            %%% Use ideal filters - the simulation will run much faster.
            %%% Divide by 4 - to match existing setup.
            %%% -50dBFS signal at period=10.5 samples should be +/- 20.
            
            back_data_I=floor(filter(Bsave,Asave,0.25*back_data_I));
            back_data_Q=floor(filter(Bsave,Asave,0.25*back_data_Q));

                
            radio_state=zeros(1,length(clear_data));
            
            for loop_i=1:length(clear_data)-1              
              if(clear_data(loop_i+1) == 1 && clear_data(loop_i) == 0 && radio_state(loop_i)==0)
                radio_state(loop_i+1)=14;
              elseif(clear_data(loop_i+1) == 1 && clear_data(loop_i) == 0 && radio_state(loop_i)==14) 
                radio_state(loop_i+1)=8;
              elseif(clear_data(loop_i+1) == 1 && clear_data(loop_i) == 0 && radio_state(loop_i)==8) 
                radio_state(loop_i+1)=14;
              else
                radio_state(loop_i+1)=radio_state(loop_i);
              endif
            endfor
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%                                                                                          %
% Section 3 - Test out CDR                                                                 %
%                                                                                          %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

            disp(sprintf("Running the CDR,BLF:%d,DC:%2.2f,ANG:%2.2f,MAG:%d,NOISE:%d",loop_blf_a,loop_dc_b,loop_angle_c,loop_ampl_d,loop_noise_e));

            [bits, magnitude, angle, mag_I, mag_Q, exit_code, irq, result, sample_I, sample_Q]=rfidCDR(back_data_I,back_data_Q,clear_data);  %%% Run the CDR.

            max(result)                                                                         %%% A debug message - what was worst-case packet result?
            
            if(1)
              figure(3); clf; hold on;
              stem(irq,'b');
              stem(result,'r');
            endif
            
            if(length(cell2mat(bits(1))) != length(tag_rn16_packet) || length(cell2mat(bits(2))) != length(tag_pcepc_packet)) %%% Handle error condition where wrong number of recovered bits exist.
              bit_errors=length(tag_rn16_packet)+length(tag_pcepc_packet);                                                    %%% Count bit errors.
              pkt_errors=2;                                                                                                   %%% Assume both packets are in error.
            else
              bit_errors+=sum(cell2mat(bits(1)) != tag_rn16_packet)+sum(cell2mat(bits(2)) != tag_pcepc_packet);               %%% If all good, add up the bit errors.
              pkt_errors+=any(cell2mat(bits(1)) != tag_rn16_packet)+any(cell2mat(bits(2)) != tag_pcepc_packet);               %%% If all good, add up the packet errors.
            end
            
            bit_total+=length(tag_rn16_packet)+length(tag_pcepc_packet);                                                      %%% Total number of bits - used to compute BER.
            pkt_total+=2;                                                                                                     %%% Total number of packets - used to compute PER.
            
            angle_result_deg=angle(2)*180/pi/(2^13) %%% For now, use the angle computed with the longest packet. In future, we may need several angle measurements to get a good one.
            magnitude                               %%% Permit these quantities to show up as debug messages.
         
            angle_error=abs(angle_deg_vec(loop_angle_c)-angle_result_deg);                                                    %%% Compute peak angle error.
            if(angle_error > pk_angle_err)
              pk_angle_err=angle_error;
            endif
 
            eltime=toc;
            disp(sprintf("The previous iteration took %2.2f seconds",eltime));
          
            fid1=fopen(sprintf("./rtl_test_vectors/test_cdr_amp_m%ddB_phs_%ddeg_freq_%d_in.dat",-round(20*log10(ampl_vec(loop_ampl_d))),round(angle_rad_vec(loop_angle_c)*180/pi),round(BLF_vec(loop_blf_a))),"w");  
            fid2=fopen(sprintf("./rtl_test_vectors/test_cdr_amp_m%ddB_phs_%ddeg_freq_%d_out.dat",-round(20*log10(ampl_vec(loop_ampl_d))),round(angle_rad_vec(loop_angle_c)*180/pi),round(BLF_vec(loop_blf_a))),"w");
            
            for loop_i=2:length(back_data_I)
              fprintf(fid1,"%d %d %d %d %d %d\n",back_data_I(loop_i),back_data_Q(loop_i),sample_I(loop_i),sample_Q(loop_i),radio_state(loop_i),clear_data(loop_i));
            endfor 
           
            %OK, the way the below code works is as follows. We have 8 positions for data in the RX RAM for 6 different types of packets.
            %The read and PCEPC packets take up two positions each.
            %If the loop_i below corresponds to one of the packet types we are testing in this file, then we save the # bits in packet
            %then continue writing the bits, the exit code, and the vector magnitude.
            
            %If the loop_i does not correspond to one of the packet types we are testing in this file, we write a bunch of zero
            %byte entries and jump the loop_i using the continue statement, avoiding all of the bit and exit code writing.
           
            for loop_i=1:8
            
              if(loop_i==1)         
                fprintf(fid2,"%d\n",bitand(255,length(tag_rn16_packet)));
                vector_index=1;
              elseif(loop_i==7)
                fprintf(fid2,"%d\n",bitand(255,length(tag_pcepc_packet)));
                vector_index=2;
              elseif(loop_i==2)
                for loop_j=1:18   %Because the RN16 packet is only 14 bytes, we must enter two zero byte entries to complete the RN16 slot before adding 16 zero bytes.
                  fprintf(fid2,"%d\n",bitand(255,0));
                endfor
                continue; 
              else
                for loop_j=1:16
                  fprintf(fid2,"%d\n",bitand(255,0));
                endfor
                continue;
              endif  
                
              bits_pkt=cell2mat(bits(vector_index));
              mag_I_pkt=mag_I(vector_index);
              mag_Q_pkt=mag_Q(vector_index);
              exit_code_pkt=exit_code(vector_index);
              
              if(mag_I_pkt < 0)
                mag_I_pkt = (2^32)+mag_I_pkt;
              endif
              if(mag_Q_pkt < 0)
                mag_Q_pkt = (2^32)+mag_Q_pkt;
              endif
              
              while ~isempty(bits_pkt)
                bits_byte=0;
                for loop_j=1:8 %% This will break if bits vector is not a multiple of 8.
                  bits_byte+=bits_pkt(loop_j)*(2^(loop_j-1));
                endfor
                fprintf(fid2,"%d\n",bitand(255,bits_byte));
                bits_pkt=bits_pkt(9:end);
              endwhile
              
              fprintf(fid2,"%d\n",bitand(255,exit_code_pkt));
              
              for loop_j=1:4
                fprintf(fid2,"%d\n",bitand(255,mag_I_pkt));
                mag_I_pkt = floor(mag_I_pkt / 256);
              endfor
                
              for loop_j=1:4
                fprintf(fid2,"%d\n",bitand(255,mag_Q_pkt));
                mag_Q_pkt = floor(mag_Q_pkt / 256);
              endfor
              
             endfor
            
          fclose(fid1);
          fclose(fid2); 
           
          endfor
          
          cell2mat(bits(1))
          tag_rn16_packet
          
          %cell2mat(bits(2))
          %tag_pcepc_packet
          
          ber=bit_errors/bit_total;
          per=pkt_errors/pkt_total;
          
          results_toplevel{loop_blf_a,loop_dc_b,loop_angle_c,loop_ampl_d}=[ber per pk_angle_err pk_ampl_err];
            
       endfor
      endfor
    endfor
  endfor   
            