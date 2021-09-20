function testSDMtoTXCancelChain

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%                                                                                       %
% testSDMtoTXCancelChain                                                                %
%                                                                                       %
% Filename: testSDMtoTXCancelChain.m                                                    %
% Creation Date: 11/5/2015                                                              %
% Author: Edward Keehr                                                                  %
%                                                                                       %
% Copyright Superlative Semiconductor LLC 2021                                          %
% This source describes Open Hardware and is licensed under the CERN-OHL-P v2           %
% You may redistribute and modify this documentation and make products                  %
% using it under the terms of the CERN-OHL-P v2 (https:/cern.ch/cern-ohl).              %
% This documentation is distributed WITHOUT ANY EXPRESS OR IMPLIED                      %
% WARRANTY, INCLUDING OF MERCHANTABILITY, SATISFACTORY QUALITY                          %
% AND FITNESS FOR A PARTICULAR PURPOSE. Please see the CERN-OHL-P v2                    %
% for applicable conditions.                                                            %
%                                                                                       %
% This file tests cancellation of the TX DC signal in the feedback loop with the sigma  %
% delta modulator. We also include the RX path filters in order to observe what sort of %
% SNR fallout exists.                                                                   %
%                                                                                       %
% 111815 - Copy over to release directory. Clean up with proper formatting and better   %
% comments.                                                                             %
% 122815 - Fix to ensure system works with new filtering design. Also, generate test    %
% vectors for TX cancellation simulation and also export .dat and .csv (pre-hex) files  %
% with the memory information.                                                          %
% 022116 - Switch over to the 2-bit cancellation matrix.                                %
% 070516 - Test with new tx cancel which does blind rotation compensation.              %
% 073016 - Save SDM outputs for testing of waveform storage.                            %
% 091016 - Put proper CDR in place. Save RX, TX inputs to system @lines 144 to 167      %
% 091116 - Use 141 not 161 Select bits. Also use 16 RN16_I bits                         %
% 091516 - Add provision to test what happens when we delay tx cancel feedback          %
% 100817 - Test latest tx cancel revisions - include LNA railing.                       %
% 082821 - Update for public release. Changed reset of txCancelAlgorithm to Fast version%
%                                                                                       %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%%% Use Richard Schreier's Delta Sigma MATLAB Toolbox.
addpath ./delsig;
pkg load all;

load -binary nw082117_simple_qucs_bin.mat;
load -binary nw082117_simple_qucs_step.mat;

more off; % so that we can see debugging messages as they happen.

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%                                                                                   %
% Section 0 - Define loops related to generating backscatter patterns               %
%                                                                                   %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%%% Define the Tx leakage rotation angle.

tx_lkg_rot_angle=0*2*pi/360;

%%% Declare the RX SDM parameters (guesstimated - this is RX SDM on the SX1257).

order=5;
osr=32;
opt=1;
H_inf=1.5;
f0=0;

%%% Define SDM NTF now because doing this for every loop takes a long time.

ntf = synthesizeNTF(order,osr,opt,H_inf,f0);

%%% Declare the decimation factors used in our design.

decim_fact1=8;
decim_fact2=128;

%%% Define the backscatter link frequency in terms of the BLF period.

%BLF_vec=100*round(2250000./([10 10.5 11 11.5 12 12.5 13 13.5 14])/100);
%BLF_vec=100*round(2250000./([10.5])/100);
BLF_vec=100*round(2250000./([12])/100);

%%% Make sure to check what happens during extreme duty cycles (effectively, period chattering every clock cycle).

%duty_cycle_vec=[0.45 0.5 0.55];
duty_cycle_vec=0.45;

%%% Test that the CORDIC algorithm in the FPGA can successfully determine the phase angle.
%%% (Note such an algorithm was not implemented on the FPGA but rather in the iPhone).

angle_deg_vec=rand(1,10)*360;
%angle_rad_vec=pi*angle_deg_vec/180;
angle_rad_vec=pi*1/180;

%%% Full scale backscatter amplitude is -20dBm (scaling factor of "1" relative to full scale).
%%% Minimum backscatter amplitude is -70dBm (scaling factor of "0.00031" relative to full scale).
%%% Here, we denote everything to dBm then scale it all to the SDM input later.

%ampl_vec_dB=-41:-1:-50;
ampl_vec_dB=-89;                                      %%% This is in terms of dBm. Was -70. We substract 19dB to be consistent with TX leakage, which is referred to SX1257 input.
%ampl_vec_dB=-30;
ampl_vec=(10.^(ampl_vec_dB/20))*(10^(-13/20));        %%% Convert to an actual amplitude, the -13dB is dBm to Vrms conversion.
num_noise=1;                                          %%% The number of different noise waveform realizations to be run.
noise_offset=1;                                       %%% Can be used to select a particular noise waveform realization for examination.

results_toplevel=cell(length(BLF_vec),length(duty_cycle_vec),length(angle_rad_vec),length(ampl_vec_dB));  %%% Create a single structure to hold the results of the simulation.

for loop_blf_a=1:length(BLF_vec)                      %%% Loop backscatter frequency.
  for loop_dc_b=1:length(duty_cycle_vec)              %%% Loop duty cycle.
    for loop_angle_c=1:length(angle_rad_vec)          %%% Loop angle.

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%                                                                                   %
% Section 1 - Generate input waveforms                                              %
%                                                                                   %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

      Fs=36e6;                                        %%% SDM sampling rate, prior to decimation.
      M=8;                                            %%% Miller modulation index for the backscattered waveform.

      rand("seed", 3285);                             %%% Seed the random number generator so that we get the same results for same simulation settings.
      
      tag_rn16_data=rand(1,16)>0.5;                   %%% Generate RN16 data.
      tag_pcepc_data=rand(1,112)>0.5;                 %%% Generate PCEPC data.
      rn16_crc=crc_ccitt16_tx(tag_rn16_data);         %%% Generate CRC data for RN16.
      pcepc_crc=crc_ccitt16_tx(tag_pcepc_data);       %%% Generate CRC data for PCEPC.
      %tag_rn16_packet=[tag_rn16_data rn16_crc];      %%% Concatenate RN16 CRC with RN16 data.
      tag_rn16_packet=tag_rn16_data;
      tag_pcepc_packet=[tag_pcepc_data pcepc_crc];    %%% Concatenate PCEPC data with PCEPC CRC.

      %sel_data=rand(1,161)>0.5;                      %%% Generate TX Select packet data.
      sel_data=rand(1,141)>0.5;                       %%% Generate TX Select packet data.
      que_data=rand(1,22)>0.5;                        %%% Generate TX Query packet data.
      ack_data=tag_rn16_data;                         %%% Generate TX ACK packet data.
      Tari=1/46875;                                   %%% Define TX Tari.
      
      %%% Generate the complete analog baseband waveform at RX, include RX analog baseband filtering.
      
      [tx_strm,rx_strm,blank_strm,reset_strm]=generateAnaBBWFs(sel_data,que_data,tag_rn16_packet,ack_data,tag_pcepc_packet,Tari,BLF_vec(loop_blf_a),M,Fs,duty_cycle_vec(loop_dc_b));

      %%% Debugging plot - ensure that the outputs of the analog baseband generation file are correct.
      
      if(1)
        figure(40); hold on;
        plot((1/Fs)*(0:1:length(tx_strm)-1),tx_strm,'r');
        plot((1/Fs)*(0:1:length(rx_strm)-1),rx_strm,'b');
        plot((1/Fs)*(0:1:length(reset_strm)-1),reset_strm,'m');
      end
   
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%                                                                                   %
% Section 2 - Apply scalings to data and noise                                      %
%                                                                                   %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

      for loop_ampl_d=1:length(ampl_vec)              %%% Loop amplitude of signal.
        
        bit_errors=0;                                 %%% Declare and reset BER statistics.
        bit_total=0;                                  %%% We declare these variables here because
        pkt_errors=0;                                 %%% we want to effectively average all of these variables
        pkt_total=0;                                  %%% over many noise waveform realizations.
        pk_ampl_err=0;
        pk_angle_err=0;
        
        %%% Apply amplitude scaling and phase rotation to the RX stream.
        
        rx_strm_I=ampl_vec(loop_ampl_d)*rx_strm*cos(angle_rad_vec(loop_angle_c));
        rx_strm_Q=ampl_vec(loop_ampl_d)*rx_strm*sin(angle_rad_vec(loop_angle_c));
        rx_strm_cplx=rx_strm_I+i*rx_strm_Q;
        
        for loop_noise_e=1:num_noise                  %%% Loop noise waveform realization.
          tic
          randn('seed', loop_noise_e+noise_offset);   %%% Ensure that all noise generated in this section is repeatable by entering in
          rand('seed', loop_noise_e+noise_offset);    %%% the proper seed parameters.
          [noise_cplx]=generateRFIDPhaseNoise(length(tx_strm),Fs); %%% This function provides noise out at -100dBc/Hz at 100kHz.
      
          if(0)
            noise_cplx=0;
          endif
          
          %%% In keeping with how we scaled the backscatter signal, we scale
          %%% the TX signal by 30dB.
          %%% We choose a representative angle that should evenly distribute noise between I and Q paths.
          
          tx_angle=40;
          tx_angle_rad=pi*tx_angle/180;
          
          noisy_tx_strm_cplx=(10^(11/20))*(tx_strm+tx_strm.*noise_cplx)*(10^(-13/20)); %The -13dB is the dBm to Vrms conversion. %The 20 dB is 30dBm out -10dB coupler loss -9dB RX input loss.

          %%% Save system inputs that go to Verilog simulation.
          
          if(1)
            format long;
            fid_rfidr_top=fopen("./rtl_test_vectors/rfidr_top_input_112617.dat","w");

            for loop_i=1:length(rx_strm_I)
              fprintf(fid_rfidr_top,"%2.12f %2.12f %2.12f %2.12f\n",rx_strm_I(loop_i),rx_strm_Q(loop_i),real(noisy_tx_strm_cplx(loop_i)),imag(noisy_tx_strm_cplx(loop_i)));
            endfor

            fclose(fid_rfidr_top);

          endif
                  
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%                                                                                   %
% Section 3 - Allocate memory for signal vectors so that loop gets sped up          %
%                                                                                   %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
 
          refl_trgt_mag=0.225;                                                                %%% Place the optimal reflection coefficient of the reflection network in a bad spot
          refl_trgt_angle=-60;                                                                %%% near the north east edge of the Smith Chart circle with radius=0.3.
          refl_trgt_angle_rad=pi*refl_trgt_angle/180;                                         %%% Convert from degrees to radians.
          refl_trgt=refl_trgt_mag*exp(i*refl_trgt_angle_rad);                                 %%% Make it a complex scaling factor.
          
          refl_actl=zeros(1,length(noisy_tx_strm_cplx));                                      %%% Vector of the actual reflection coefficients of the programmable reflection network.
          noisy_tx_strm_cplx_input=zeros(1,length(noisy_tx_strm_cplx));                       %%% The reflected TX signal scaled by the difference between actual and optimal reflection coefficients.
          total_input=zeros(1,length(noisy_tx_strm_cplx));                                    %%% Vector containing both scaled TX and backscatter signals.
          sdm_out_I=zeros(1,length(noisy_tx_strm_cplx));                                      %%% Sigma delta I channel output.
          sdm_out_Q=zeros(1,length(noisy_tx_strm_cplx));                                      %%% Sigma delta Q channel output.
          decim1_out_I=zeros(1,ceil(length(noisy_tx_strm_cplx)/decim_fact1));                 %%% Main I CIC/decimator output.
          decim1_out_Q=zeros(1,ceil(length(noisy_tx_strm_cplx)/decim_fact1));                 %%% Main Q CIC/decimator output.
          cancel_gain_bits_out=zeros(1,ceil(length(noisy_tx_strm_cplx)/decim_fact1));         %%% Send the cancel gain bits to Verilog simulation.
          decim2_out_I_out=zeros(1,ceil(length(noisy_tx_strm_cplx)/decim_fact1));
          decim2_out_Q_out=zeros(1,ceil(length(noisy_tx_strm_cplx)/decim_fact1));
          reset_strm_dec=zeros(1,ceil(length(noisy_tx_strm_cplx)/decim_fact1));               %%% Reset vector, modeled as the IRQ clear pulses from the MCU.
          decim2_out_I=zeros(1,ceil(length(noisy_tx_strm_cplx)/(decim_fact1*decim_fact2)));   %%% Second (DC only) I CIC/decimator output.
          decim2_out_Q=zeros(1,ceil(length(noisy_tx_strm_cplx)/(decim_fact1*decim_fact2)));   %%% Second (DC only) Q CIC/decimator output.
          cap_vec_hist=zeros(5,ceil(length(noisy_tx_strm_cplx)/(decim_fact1*decim_fact2)));   %%% Vector of capacitor control history values.
          
          simulateDSM_oneshotI(0,ntf,2,1);                                                    %%% Clear memory in I Sigma Delta modulator.
          simulateDSM_oneshotQ(0,ntf,2,1);                                                    %%% Clear memory in Q Sigma Delta modulator.
          decimFilterBitExact_oneshotI(0,1);                                                  %%% Clear memory in Main I CIC/Decimator.
          decimFilterBitExact_oneshotQ(0,1);                                                  %%% Clear memory in Main Q CIC/Decimator.
          decimFilterSecondBitExact_oneshotI(0,1);                                            %%% Clear memory in Second I CIC/Decimator.
          decimFilterSecondBitExact_oneshotQ(0,1);                                            %%% Clear memory in Second Q CIC/Decimator.
          txCancelAlgorithmFast(0,0,step_mat_A_qtz,step_mat_B_qtz,step_mat_C_qtz,step_mat_D_qtz,1); %%% Clear memory in TX Cancel algorithm.
          %%% 082821 - Above was originally "txCancelAlgorithm" during final tests, meaning that we might not have been resetting this correctly. Watch for weird stuff.

          cap_vec=[16 0 16 0 10^(-23/20)];                                                    %%% Initialize capacitance vector.
          cap_vec_new=[16 0 16 0 10^(-23/20)];
          lna_gain=10^(-23/20);                                                               %%% Minimum gain - other gain steps are 10^(-4/20), 10^(20/20).
          bb_gain=10^(23/20);                                                                 %%% Fixed - want to ensure that maximum signal that will rail LNA will rail SDM.
          
          disp(sprintf("The simulation will be %d points long",length(noisy_tx_strm_cplx)));

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%                                                                                   %
% Section 4 - Run the simulation                                                    %
%                                                                                   %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

          tic
          for loop_i=1:length(noisy_tx_strm_cplx)
          %for loop_i=1:100000
          
            if(mod(loop_i,10000)==0)
              toc
              disp(sprintf("We are at iteration %d",loop_i));
              tic
            endif
          
            refl_actl(loop_i)=refl_mat(1,cap_vec(1)+1,cap_vec(2)+1,cap_vec(3)+1,cap_vec(4)+1);        %%% This mimics the real-world effect of the reflection network response to cap. changes.
            noisy_tx_strm_cplx_input(loop_i)=exp(i*tx_lkg_rot_angle)*(refl_actl(loop_i)-refl_trgt)*noisy_tx_strm_cplx(loop_i);  %%% Scale the TX leakage by the complex amplitude of the difference between target and actual reflection coefficients.
            total_input_temp=lna_gain*(noisy_tx_strm_cplx_input(loop_i)+rx_strm_cplx(loop_i));        %%% Add RX signal. Now this is the complete input to the receive chain.
            
            if(abs(total_input_temp) > 10^(-28.8/20))
              total_input(loop_i) = (10^(-28.8/20))*bb_gain;
            elseif(abs(total_input_temp) < -10^(-28.8/20))
              total_input(loop_i) = (-10^(-28.8/20))*bb_gain;
            else
              total_input(loop_i) = total_input_temp*bb_gain;
            endif
            
            [sdm_out_I_tmp,x0_I_out]=simulateDSM_oneshotI(real(total_input(loop_i)),ntf,2,0);           %%% Run one step of the I SDM.
            [sdm_out_Q_tmp,x0_Q_out]=simulateDSM_oneshotQ(imag(total_input(loop_i)),ntf,2,0);           %%% Run one step of the Q SDM.
            
            sdm_out_I(loop_i)=sdm_out_I_tmp;                                                            %%% Make assignment to output vector.
            sdm_out_Q(loop_i)=sdm_out_Q_tmp;   
            
            [decim1_out_I_tmp,decim1_adder_hist_I]=decimFilterBitExact_oneshotI(sdm_out_I_tmp,0);       %%% Run one step of the I Main CIC/Decimator.
            [decim1_out_Q_tmp,decim1_adder_hist_Q]=decimFilterBitExact_oneshotQ(sdm_out_Q_tmp,0);       %%% Run one step of the Q Main CIC/Decimator.
            
            if(mod(loop_i,8)==1)                                                                        %%% Effectively perform a decimate-by-8.
            
              decim1_out_I((loop_i-1)/8+1)=decim1_out_I_tmp*blank_strm(loop_i);                         %%% Blank portions of the data stream corrupted by TX going to RX demod.
              decim1_out_Q((loop_i-1)/8+1)=decim1_out_Q_tmp*blank_strm(loop_i);                         %%% This also counts as making an assignment to an output vector.
              reset_strm_dec((loop_i-1)/8+1)=reset_strm(loop_i);                                        %%% Downsample the reset stream.
            
              [decim2_out_I_tmp,decim2_adder_hist_I]=decimFilterSecondBitExact_oneshotI(decim1_out_I_tmp,0);  %%% Run one step of the I Second CIC/Decimator.
              [decim2_out_Q_tmp,decim2_adder_hist_Q]=decimFilterSecondBitExact_oneshotQ(decim1_out_Q_tmp,0);  %%% Run one step of the Q Second CIC/Decimator.
              
              decim2_out_I_out((loop_i-1)/8+1)=decim2_out_I_tmp;
              decim2_out_Q_out((loop_i-1)/8+1)=decim2_out_Q_tmp;
              cancel_gain_bits_out((loop_i-1)/8+1)=lna_gain;
                           
            endif
            
            if(mod(loop_i,1024)==1)                                                                   %%% Effectively perform another downsampling by 128.
              
               decim2_out_I((loop_i-1)/1024+1)=decim2_out_I_tmp;                                      %%% Make assignment to output vector.
               decim2_out_Q((loop_i-1)/1024+1)=decim2_out_Q_tmp;
                
            endif
            
            if(mod(loop_i,2048)==1)
              %%% Run one step of the TX leakage cancellation algorithm
               cap_vec_new=txCancelAlgorithmFast(decim2_out_I_tmp,decim2_out_Q_tmp,step_mat_A_qtz,step_mat_B_qtz,step_mat_C_qtz,step_mat_D_qtz,0);
            endif  
          
            if(mod(loop_i,1024)==1) 
              cap_vec_hist(:,(loop_i-1)/1024+1)=cap_vec_new;
            endif
          
            %%% Model the delay of the tx cancel algorithm in actual hardware
            
            if(mod(loop_i,2048)==64)
              cap_vec(1)=cap_vec_new(1);
            endif
            
            if(mod(loop_i,2048)==64)
              cap_vec(2)=cap_vec_new(2);
            endif
            
            if(mod(loop_i,2048)==64)
              cap_vec(3)=cap_vec_new(3);
            endif
            
            if(mod(loop_i,2048)==64)
              cap_vec(4)=cap_vec_new(4);
            endif
            
            if(mod(loop_i,2048)==80)
              lna_gain=cap_vec_new(5);
            endif
            
          endfor %%% loop_i
          toc
          
          %%% Run all of the RX BBF at once, as it is not part of the feedback loop.
          %%% Perform the scaling that is required to keep the desired signal above the noise floor.
  
          [chFilt_out_I,adder_B1_I,adder_B2_I]=tunedIIRFilterBitExact(decim1_out_I);
          [chFilt_out_Q,adder_B1_Q,adder_B2_Q]=tunedIIRFilterBitExact(decim1_out_Q);
          

            disp(sprintf("Running the CDR,BLF:%d,DC:%2.2f,ANG:%2.2f,MAG:%d,NOISE:%d",loop_blf_a,loop_dc_b,loop_angle_c,loop_ampl_d,loop_noise_e));
            
            %%% Run the CDR all at once, as we did before.
            
            [bits, magnitude, angle, mag_I, mag_Q, exit_code, irq, result, sample_I, sample_Q]=rfidCDR(chFilt_out_I,chFilt_out_Q,reset_strm_dec);

            end_result=max(result)                                                                    %%% A debug message - what was worst-case packet result?
            
            if(1)
              figure(11); clf; hold on;
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
            magnitude                               %%% Permit these quantites to show up as debug messages.
         
            angle_error=abs(angle_deg_vec(loop_angle_c)-angle_result_deg);                                                    %%% Compute peak angle error.
            if(angle_error > pk_angle_err)
              pk_angle_err=angle_error;
            endif
          
          cell2mat(bits(1))
          tag_rn16_packet
          
          
         endfor %loop_noise_e
        endfor  %loop_ampl_d
       endfor   %loop_angle_c
      endfor    %loop_dc_b
     endfor     %loop_blf_a

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%                                                                                             %
% Section 5 - Save results                                                                    %
%                                                                                             %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

if(0)
fid_wvstrg_in=fopen("./rtl_test_vectors/wvstrg_input.dat","w");

for loop_i=1:length(sdm_out_I)
  fprintf(fid_wvstrg_in,"%d %d\n",sdm_out_I(loop_i) > 0,sdm_out_Q(loop_i) > 0); %% These values are +1 / -1
endfor

fclose(fid_wvstrg_in);

endif

if(0)
fid_wvstrg_in=fopen("./rtl_test_vectors/wvstrg_input_12.dat","w");

for loop_i=1:length(sdm_out_I)
  fprintf(fid_wvstrg_in,"%d %d\n",sdm_out_I(loop_i) > 0,sdm_out_Q(loop_i) > 0); %% These values are +1 / -1.
endfor

fclose(fid_wvstrg_in);

endif

if(1)
fid_txcancel_in=fopen("./rtl_test_vectors/txcancel_2bit_input_112617.dat","w");
fid_txcancel_out=fopen("./rtl_test_vectors/txcancel_2bit_output_112617.dat","w");

for loop_i=1:length(decim2_out_I_out)
  fprintf(fid_txcancel_in,"%d %d %d %d\n",decim2_out_I_out(loop_i),decim2_out_Q_out(loop_i),cancel_gain_bits_out(loop_i) > 0,mod(loop_i,256)==1);
endfor

for loop_i=1:max(size(cap_vec_hist))
  fprintf(fid_txcancel_out,"%d %d %d %d\n",cap_vec_hist(1,loop_i),cap_vec_hist(2,loop_i),cap_vec_hist(3,loop_i),cap_vec_hist(4,loop_i));
endfor

fclose(fid_txcancel_in);
fclose(fid_txcancel_out); 

endif

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% 
%                                                                                             % 
% Section 6 - Plot results                                                                    %
%                                                                                             %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
 
 %%% Generate time vectors for all three sampling rates seen in the simulation.
 
 time_vec=(0:length(noisy_tx_strm_cplx)-1)/Fs;
 time_vec_dec1=(0:length(decim1_out_I)-1)/(Fs/decim_fact1);
 time_vec_dec2=(0:length(decim2_out_I)-1)/(Fs/decim_fact1/decim_fact2);
 
 figure(1); clf; hold on;
 plot(time_vec,real(noisy_tx_strm_cplx_input),'b');
 plot(time_vec,imag(noisy_tx_strm_cplx_input),'r');
 xlabel('Time(s)');
 ylabel('TX Leakage (V)');
 title('TX Leakage seen at RX Antenna');
 legend('Real (I Path)','Imaginary (Q Path)','location','northeast');
 grid on;
 
 figure(2); clf; hold on;
 plot(time_vec,real(total_input),'b');
 plot(time_vec,imag(total_input),'r');
 xlabel('Time(s)');
 ylabel('RX SDM ADC Input(V)');
 title('TX Leakage and Tag Backscatter Signal seen at RX SDM Input');
 legend('Real (I Path)','Imaginary (Q Path)','location','northeast');
 grid on;
 
 figure(3); clf; hold on;
 plot(time_vec,sdm_out_I,'b');
 plot(time_vec,sdm_out_Q,'r');
 xlabel('Time(s)');
 ylabel('RX SDM ADC Output');
 title('RX SDM ADC Output');
  
 %%% Make an ideal decimation filter so that we can determine whether what is coming out
 %%% of the SDM is actually a meaningful signal or not.
 
 dec_filt1_ideal=conv(conv(ones(1,8),ones(1,8)),conv(ones(1,8),ones(1,8)));
 
 figure(4); clf; hold on;
 plot(time_vec,filter(dec_filt1_ideal,sum(dec_filt1_ideal),sdm_out_I),'b');
 plot(time_vec,filter(dec_filt1_ideal,sum(dec_filt1_ideal),sdm_out_Q),'r');
 xlabel('Time(s)');
 ylabel('RX SDM ADC Output - Ideally filtered by decim filter 1');
 title('RX SDM ADC Output - Ideally filtered by decim filter 1');
 
 figure(5); clf; hold on;
 plot(time_vec_dec1,decim1_out_I,'b');
 plot(time_vec_dec1,decim1_out_Q,'r');
 xlabel('Time(s)');
 ylabel('RX Decimation Filter 1 Output (LSB)');
 title('TX Leakage and Tag Backscatter Signal seen at RX Decimation Filter 1 Output');
 legend('Real (I Path)','Imaginary (Q Path)','location','northeast');
 grid on;
 
 figure(6); clf; hold on;
 plot(time_vec_dec2,decim2_out_I,'b');
 plot(time_vec_dec2,decim2_out_Q,'r');
 xlabel('Time(s)');
 ylabel('RX Decimation Filter 2 Output (LSB)');
 title('TX Leakage and Tag Backscatter Signal seen at RX Decimation Filter 2 Output');
 legend('Real (I Path)','Imaginary (Q Path)','location','northeast');
 grid on;
 
 %%% Merge four capacitor values into two effective subranging capacitor values.
 
  cap1=cap_vec_hist(1,:)*(2^5)+cap_vec_hist(2,:);
  cap2=cap_vec_hist(3,:)*(2^5)+cap_vec_hist(4,:);
 
 figure(7); clf; hold on;
 plot(time_vec_dec2,cap1,'b');
 plot(time_vec_dec2,cap2,'r');
 xlabel('Time(s)');
 ylabel('Capacitor Control Value (LSB)');
 title('TX Cancellation Capacitor Control Value');
 legend('Cap. 1','Cap. 2','location','northeast');
 grid on;
 
 figure(8); clf; hold on;
 plot(real(refl_trgt),imag(refl_trgt),'rx');
 plot(real(refl_actl),imag(refl_actl),'b');
 xlabel('Real Reflection Coefficient');
 ylabel('Imaginary Reflection Coefficient');
 title('TX Cancellation Algorithm Convergence as seen by Reflection Coefficient');
 legend('Optimum Reflection Coefficient','Algorithm Computed Coefficient','location','northeast');
 grid on;
 
 figure(9); clf; hold on;
 plot(time_vec,real(refl_actl),'b');
 plot(time_vec,imag(refl_actl),'r');
 xlabel('Time(s)');
 ylabel('Reflection Coefficient');
 title('TX Cancellation Algorithm Reflection Coefficient as Function of Time');
 legend('Real Part','Imaginary Part','location','northeast');
 grid on;
 
 figure(10); clf; hold on;
 plot(time_vec_dec1,chFilt_out_I,'b');
 plot(time_vec_dec1,chFilt_out_Q,'r');
 
 figure(11); clf; hold on;
 plot(time_vec_dec2,cap_vec_hist(1,:),'b');
 plot(time_vec_dec2,cap_vec_hist(2,:),'r');
 plot(time_vec_dec2,cap_vec_hist(3,:),'g');
 plot(time_vec_dec2,cap_vec_hist(4,:),'k');
 xlabel('Time(s)');
 ylabel('Capacitor Control Value (LSB)');
 title('TX Cancellation Capacitor Control Value');
 legend('Cap. 1','Cap. 2','Cap. 3','Cap. 4','location','northeast');
 grid on;