function [bits, magnitude, angle, magI_out, magQ_out, exit_code_out, irq, result, sampleI]=rfidCDR(dataI,dataQ,reset_sys)

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%                                                                                         %
% 100715 - rfidCDR                                                                        %
%                                                                                         %
% Filename: rfidCDR.m                                                                     %
% Creation Date: 10/07/2015                                                               %
% Author: Edward Keehr                                                                    %
%                                                                                         %
% Copyright Superlative Semiconductor LLC 2021                                            %
% This source describes Open Hardware and is licensed under the CERN-OHL-P v2             %
% You may redistribute and modify this documentation and make products                    %
% using it under the terms of the CERN-OHL-P v2 (https:/cern.ch/cern-ohl).                %
% This documentation is distributed WITHOUT ANY EXPRESS OR IMPLIED                        %
% WARRANTY, INCLUDING OF MERCHANTABILITY, SATISFACTORY QUALITY                            %
% AND FITNESS FOR A PARTICULAR PURPOSE. Please see the CERN-OHL-P v2                      %
% for applicable conditions.                                                              %
%                                                                                         %
% This file implements the proposed CDR system for the RFIDr FPGA.                        %
% It accepts the output of whatever filtering and signal conditioning come before it      %
% and outputs: bits, a magnitude, an angle, and a result (pass or error by various means) %
%                                                                                         %
% 102215 - Alter CDR to approximate an analog CDR. Use linear phase detector.             %
% 102315 - Rescaled from rfidCDR102215B                                                   %
% 103015 - Attempt to get to work with tuned filter.                                      %
% 111715 - This file is derived from rfidCDR103015C and placed into the release library.  %
% The file is better organized and enhanced with more comments.                           %
% 121615 - Updated to include Verilog modifications that reduce overall LUT count.        %
% 031116A - Improving version with Hogge PD.                                              %
% 032416 - Update DR with LUT-reducing changes, including use-one-DR-path-as-master-as    %
% dictated-by-software.                                                                   %
% 092616 - Disabled CRC check for length-16 packets, assuming that they are returned      %
% throwaway RN16s from the tag.                                                           %
%                                                                                         %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% The reset_sys vector is expected to come from the microcontroller or a higher-level state machine.
% When it is high, it should reset_sys persistent variables.
% This will be good because it can avoid unanticipated lock up issues.

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%                                                                                         %
% Section 0 - Check inputs                                                                %
%                                                                                         %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

if(length(dataI)!=length(dataQ))
  error("The length of data I is not equal to the length of data Q");
endif

if(length(dataI)!=length(reset_sys))
  error("The length of data is not equal to the length of reset_sys");
endif

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%                                                                                         %
% Section 1 - Run CDR top level simulation                                                %
%                                                                                         %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

disp("Running CR I");
[sampleI,state_I,foI,fmI,fdI,tank_lsbI]=clk_rcvy(dataI,reset_sys);

disp("Running Data Recovery");
[bits, magnitude, angle, magI_out, magQ_out, exit_code_out, irq, result, bit_counter, crc_out]=sampler(dataI,dataQ,sampleI,sampleI,reset_sys);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%                                                                                         %
% Section 2 - Plot debugging plots.                                                       %
%                                                                                         %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

if(1)
  figure(21); clf; hold on;
  timevec=1000*(0:length(sampleI)-1)/4500000;
  stem(timevec,1000*sampleI,'b');
  stem(timevec,dataI,'r');
  plot(timevec,1000*fdI,'m','LineWidth',5);
  plot(timevec,1000*fmI,'c','LineWidth',5);
  %plot(timevec,tank_lsbI/128,'g');
  plot(timevec,crc_out*1000,'g','LineWidth',5);
  plot(timevec,10*state_I,'k');
  legend('Sampling Clock','Oversampled Data','CR PLL State','location','SouthEast');
  xlabel('Time(ms)');
  ylabel('Amplitude (LSB)');
  title('Clock Recovery Near Sensitivity');
  grid on;
endif

if(0)
  figure(22); clf; hold on;
  stem(timevec,50*sampleQ,'b');
  stem(timevec,dataQ,'r');
endif

if(0)
  figure(21);
  plot(timevec,2.5*bit_counter,'g','LineWidth',5);
  plot(timevec,200*crc_out,'m','LineWidth',5);
  grid on;
endif

endfunction

function [clkIb, state_out, freq_out_vec, freq_mem_vec, freq_delta_vec, tank_lsb_out]=clk_rcvy(data_in,reset_sys)

%%% This is the top level clock recovery circuit logic

% Because we are too lazy to try out object oriented programming in octave
% what we will do here is describe one clock recovery circuit.
% We will process one of I or Q in whole, reset_sys the static variables, then process the other I or Q stream in whole.

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%                                                                                               %
% Section 0 - Initialize the clock recovery circuit                                             %
%                                                                                               %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

period=12;                                        %%% The initial period (in 1/Fs) estimated by the CR PLL.
state=0;                                          %%% Equivalent to the capaitor in analog PLL. Goes positive and negative.
counter=1;                                        %%% Equivalent to the VCO in analog PLL. Runs from 1 to period every cycle.
clkIb=zeros(1,length(data_in));                   %%% The output vector whose high pulses denote the boundary of an estimated chip period.
state_out=zeros(1,length(data_in));               %%% Memory for outputting the state for debugging purposes. 
tank_lsb_out=zeros(1,length(data_in));                      
reset_in_clkI=1;                                  %%% Memory for resetting circuits only active during clkI phase.
schmitt_mem=1;                                    %%% Memory for the input Schmitt trigger equivalent circuit.
%schmitt_thresh=32; changed on 110415
schmitt_thresh=0;                                 %%% Parameter: threshold of the Schmitt trigger.
data_hist=zeros(1,17);                            %%% Shift register to hold data for edge detection.
freq_mem=0;                                       %%% Used for filling out the frequency detector debugging.
freq_delta=0;                                     %%% Used for filling out the frequency detector debugging.
freq_out=0;                                       %%% Used for filling out the frequency detector debugging.
phase_delta_counter=0;
tank_lsb=0;
data_edge_block=0;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%                                                                                               %
% Section 1 - Run the CR circuit                                                                %
%                                                                                               %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

for loop_i=1:length(data_in)

  if(reset_sys(loop_i))
    state=0;
    data_prev=0;
    data=0;
    data_edge_block=0;
  end

  if(loop_i==1)
    data_prev=0;
  else
    data_prev=data;
  endif
    
  data=data_in(loop_i)>=0;
 
  data_delta=abs(data-data_prev) > 0.5;
  
  if(data_delta && data_edge_block==0)
    data_edge=1;
    data_edge_block=1;
  else
    data_edge=0;
  endif
  
  %%% Define quadrature phase edges based on the current period. 
 
  clkIph=(counter==1);                   %%% Clk I is at sample 1 of the period by definition.
  clkQph=(counter==floor(period/4)+1);   %%%
  clkIbphB=(counter==floor(period/2)+1);
  clkIbph=(counter==floor(period/2));    %%% Dumping the -1 was also invaluable %use floor not round 111115 - seemed to help.
  clkQbph=(counter==floor(3*period/4));  %%% The precise method by which these values are computed were determined experimentally.
  
  %%% Generate the output clock pulse for the data recovery unit.
  
  clkIb(loop_i)=clkIph;
  
  %%% If a reset occurred, store it until the clkI phase occurs.
  
  if(reset_sys(loop_i)==1)
    reset_in_clkI=1;
  endif
  
    %%% Determine where the data edge is in relation to the sample I and sample Q edges.
    %%% data_edge=edge_det(data_hist(1:period+2),period); %%% Take two more samples than the current period to realize page 435/436 algorithm.
    %%% Determine if the current frequency is running slow or fast relative to the data.
    [freq_out,freq_mem,freq_delta,phb,phc]=freq_det(data_edge,clkIph,clkQph,clkIbph,clkQbph,reset_sys(loop_i));
    %%% Determine if the current phase is ahead or behind the data phase.
    [phase_delta]=phase_det(data,clkIph,clkIbphB,reset_sys(loop_i));
    phase_delta_counter=phase_delta_counter+phase_delta;
   
    %%% Update the state - same as dumping charge onto the capacitor of an analog PLL.
    
    %state_next=state+phase_delta+8*freq_out;
    
    state_next=state+phase_delta+2*freq_delta;
    
    %%% Perform saturation limiting of the state.
    if(state_next >= 2^9)
      state=2^9-1;
    elseif(state_next < -2^9)
      state=-2^9;
    else
      state=state_next;
    endif
    
    %%% Add the PLL zero for loop stability.
    
  if(counter==period)
    
    tank_lsb=phase_delta_counter*1024+state*64;
    %%% Utilize the effective RC tank "voltage" to drive a first-order SDM which selects the instantaneous period.
    period=period_sdm(tank_lsb,loop_i,reset_in_clkI);
  
    phase_delta_counter=0;
    %%% Clear any resets that may have been held for this clkI period.
    reset_in_clkI=0;
    counter=0;
    data_edge_block=0;
  
  endif
   
  %counter=mod(counter+1,period);
  counter=counter+1;                  %%% Switched this on 111115 in order to make explanation more intuitive. Didn't make a difference.
  state_out(loop_i)=state;            %%% Populate debug vector.
  freq_out_vec(loop_i)=freq_out;      %%% Populate debug vector.
  freq_mem_vec(loop_i)=phase_delta;   %%% Populate debug vector.
  freq_delta_vec(loop_i)=freq_delta;  %%% Populate debug vector.
  tank_lsb_out(loop_i)=tank_lsb; 
endfor 
  
endfunction

function period=period_sdm(tank_lsb,loop_i,reset_sys)
  
  %%% This is a first-order sigma delta modulator used to choose the instantaneous period
  %%% of the PLL.
  
  persistent intg=0;                    %%% Integrator register.
  persistent fb=0;                      %%% Feedback register.
  
  if(reset_sys)
    intg=0;
    fb=0;
  endif
  
  error=tank_lsb-fb;                    %%% Error into SDM loop.
  intg_next=intg+error;                 %%% Integrate the error so that near DC error is 0 input-referred.
  
  if(intg_next >= 2^15)                 %%% Saturate the integrator.
    intg=2^15-1;
  elseif(intg_next < -2^15)
    intg=-2^15;
  else
    intg=intg_next;
  endif

  if(intg > 18431)                      %%% Utilize 5 - level quantization in order to.
    period=14;                          %%% span the required range of periods.
    fb=24576;
  elseif(intg > 6143)
    period=13;
    fb=12288;
  elseif(intg > -6145)
    period=12;
    fb=0;
  elseif(intg > -18433)
    period=11;
    fb=-12288;
  else
    period=10;
    fb=-24576;
  endif
  
endfunction

function [phase_delta]=phase_det(data,clkIph,clkIbph,reset_sys)
  %%% Implement a 4-register Modified Hogge phase detector like in the 1991 DeVito CDR PLL paper.
  %%% Later we determined that this wasn't necessary for a digital phase detector implementation.
  persistent  data1=0;
  persistent  data2=0;
  persistent  data3=0;
  persistent  data4=0;
 
  if(reset_sys)
    data1=0;
    data2=0;
    data3=0;
    data4=0;
  endif
 
  if(clkIbph)
    data3=data2;
    data1=data;
  endif
  
  if(clkIph)
    data4=data3;
    data2=data1; 
  endif
 
  %phase_delta=-(xor(data,data1) - xor(data1,data2) - xor(data2,data3) + xor(data3,data4));
  phase_delta=-(xor(data,data1) - xor(data1,data2)); 
   
endfunction

function [out,mem,delta,phI_gen_reg,phQ_gen_reg]=freq_det(data_edge,phI_edge,phQ_edge,phIb_edge,phQb_edge,reset_sys)

  %Note that these sort of rotational frequency detectors were covered in the Messerschmitt 1979 IEEE Trans.
  %Communications paper.

  persistent reg_q1=0;
  persistent reg_q2=0;
  persistent reg_q3=0;
  persistent reg_q4=0;
  persistent phI_gen_reg=0;
  persistent phQ_gen_reg=0;
  persistent out=0;

  if(reset_sys)
    reg_q1=0;
    reg_q2=0;
    reg_q3=0;
    reg_q4=0;
    phI_gen_reg=0;
    phQ_gen_reg=0;
  end
  
   if(data_edge)
    reg_q3=reg_q1;
    reg_q4=reg_q2;
    reg_q1=phI_gen_reg;
    reg_q2=phQ_gen_reg;
   endif
  
   if(phI_edge)
    phI_gen_reg=1;
   endif
   
   if(phQ_edge)
    phQ_gen_reg=1;
   endif
   
   if(phIb_edge)
    phI_gen_reg=0;   
   endif
   
   if(phQb_edge)
    phQ_gen_reg=0;
   endif
  
   plus=(reg_q1 && reg_q3 && reg_q2 && !reg_q4);
   minus=(reg_q1 && reg_q3 && !reg_q2 && reg_q4);
   
   delta=(plus-minus);
   out=delta;
   mem=delta;
   
endfunction
 
function [out,mem,delta,phb_gen_reg,phc_gen_reg]=freq_det_devito(data_edge,phI_edge,phQ_edge,phIb_edge,phQb_edge,reset_sys)
   
   %%% Implement a 4-quadrant frequency detector like in the 1991 DeVito CDR PLL paper.
   %%% This circuit is augmented by a memory circuit which minimizes chatter during 
   %%% steady state lock, but which permits fast convergence while acquiring lock.
   %%% Also mote that these sort of rotational frequency detectors were covered in 
   %%% the Messerschmitt 1979 IEEE Trans. Communications paper.

   persistent phb_reg2=0; %%% memory is required because we check to see if an edge occurred.
   persistent phb_reg1=0; %%% in quadrant 2 then quadrant 3 (or vice versa).
   persistent phb_reg0=0; %%% A slip in quadrant either way is an indicator that the frequency 
   persistent phc_reg2=0; %%% is not correct.
   persistent phc_reg1=0;
   persistent phc_reg0=0;
   persistent phb_gen_reg=0;
   persistent phc_gen_reg=0;
   persistent out=0;
   
   if(reset_sys)          %%% Permit clearing of the state variables via microcontroller reset.
    phb_reg2=0;
    phb_reg1=0;
    phb_reg0=0;
    phc_reg2=0;
    phc_reg1=0;
    phc_reg0=0;
    phb_gen_reg=0;
    phc_gen_reg=0;
    out=0;
   end
   
   if(phI_edge)
    phb_reg2=phb_reg1;
    phb_reg1=phb_reg0;
    phc_reg2=phc_reg1;
    phc_reg1=phc_reg0;
   endif
   
   if(phQ_edge)
    phb_gen_reg=1;
   endif
   
   if(phIb_edge)
    phb_gen_reg=0;
    phc_gen_reg=1;     
   endif
   
   if(phQb_edge)
    phc_gen_reg=0;
   endif
   
   %%% At every clock period on phaseI, shift the registers and compute where there
   %%% might have been a quadrant slip.
   
   if(data_edge)
    phb_reg0=phb_gen_reg;
    phc_reg0=phc_gen_reg;
   endif
    
   delta=-(and(phb_reg1,phc_reg2)-and(phb_reg2,phc_reg1));
   out=delta;
   mem=delta;
   
 endfunction
  

function [bits_out, magnitude_out, angle_out, magI_out, magQ_out, exit_code_out, irq, exit_code, bit_ctr_out, crc_out]=sampler(dataI,dataQ,sampleI,sampleQ,reset_sys)

  %%% This is the data recovery circuit.
  %%% It takes in both recovered clocks and uses them both during some states and only one during other state.
  %%% This function assumes that events are clocked at the baseband data rate; i.e. 2.25MHz.
 
  %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
  %                                                                                              %
  % Section 0 - Input check                                                                      %
  %                                                                                              %
  %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
 
  if(length(dataI)!=length(dataQ))
    error("Data I is not equal length to Data Q");
  elseif(length(sampleI)!=length(sampleQ))
    error("Sample I is not equal length to Sample Q");
  elseif(length(dataI)!=length(sampleI))
    error("Data length is not equal to Sample length");
  endif
  
  %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
  %                                                                                              %
  % Section 1 - Function initialization                                                          %
  %                                                                                              %
  %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% 
  
  next_state="reset";
  locked_timer_limit1=16*12;  %Wait for approximately 16 clock cycles to determine input slope - we apply this limit twice.
  locked_timer_limit2=32*12;  %Wait for approximately 16 clock cycles to determine input slope - we apply this limit twice.
  watchdog_timer_limit=32768; %Wait in any given state besides idle for about 10ms before going to "done" with an error.
  packet_counter=1;
  packet_bits=[16 128 16 128 16];
  bits_out=cell(1,5); %Hold enough bits and magnitude to process 5 packets - we think this is why simulation is slow for this part.
  magnitude_out=zeros(5,1);
  angle_out=zeros(5,1);
  magI_out=zeros(5,1);
  magQ_out=zeros(5,1);
  exit_code_out=zeros(5,1);
  align_vec=[0 1 2 3 4 5 6 7 8 9 10 11 12 13 14 15];
  irq=zeros(1,length(dataI));
  exit_code=-1*ones(1,length(dataI));
  bit_ctr_out=zeros(1,length(dataI));
  acq_sub_out=zeros(1,length(dataI));
  crc_out=zeros(1,length(dataI));

  %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
  %                                                                                              %
  % Section 2 - Run the data recovery circuit                                                    %
  %                                                                                              %
  %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% 
  
  for loop_a=1:length(dataI)
  
    if(reset_sys(loop_a)==1)
      next_state="reset";
        magI=0;                                               %%% The integrated value of integrated winning-bit amplitudes on the I channel.
        magQ=0;                                               %%% The integrated value of integrated winning-bit amplitudes on the Q channel.
        magnitude=0;                                          %%% A dummy magnitude computed by CORDIC.
        angle=0;                                              %%% A dummy angle computed by CORDIC.
        integI_0=0;                                           %%% 0 bit integrator I state.
        integQ_0=0;                                           %%% 0 bit integrator Q state.
        prev_integI=0;                                        %%% Used to calculate slope.
        prev_integQ=0;
        watchdog_timer=0;                                     %%% watchdog timer - when it expires, the state machine resets.
        locked_timer=0;                                       %%% locked timer - it is used to define an interval over which the integrator slope is measured.
        sym_counter=0;                                        %%% symbol counter - it counts the chips in a modulo 16 fashion.
        bit_counter=0;                                        %%% bit counter - it keeps track of how many bits have been received and checks the crc at the end of the expected packet.
        sqwv=1;                                               %%% The square wave state.
        sqwvI=1;                                              %%% The square wave state for channel I.
        sqwvQ=1;                                              %%% The square wave state for channel Q.
        flip=1;                                               %%% The flip wave state.
        useI=1;                                               %%% The flag to use CR I output // 032416 - To minimize chages to code today, we force this to 1.
        useQ=0;                                               %%% The flag to use CR Q output.
        bits=[];                                              %%% An empty array to hold the bits that are received.
        crc_ccitt16_rx(0,1);                                  %%% Reset the CRC.
        next_state="idle";                                    %%% Set next state to be idle.
        peak_space_vec=zeros(1,3);                            %%% Vector to hold spacings between preamble peaks during preamble sync.
        sym_counter_vec=zeros(1,3);                           %%% Vector to hold the value of the symbol counter at each preamble peak.
        space_counter=0;                                      %%% Counter for spacings between preamble peaks.
        glue=0;  
      
    end
  
    state=next_state;                                         %%% Populate the next state.
    irq(loop_a)=0;                                            %%% Set output IRQ to zero unless one happens later.
    exit_code(loop_a)=-1;                                     %%% Set the output code to -1 until it is set again later.
                   
    if(loop_a==length(dataI)-3000 && packet_counter==1)       %%% If one packet is missed, at the end of the sim issue error code 6.
      state="done";
      exit_code(loop_a)=6;
      exit_code_out(packet_counter)=6;
    elseif(loop_a==length(dataI)-3000 && packet_counter==2)   %%% If both packets in the simulation are missed, at the end of the sim issue error code 7.
      state="done";
      exit_code(loop_a)=7;
      exit_code_out(packet_counter)=7;
    end

    switch state
      case {"reset"}                                          %%% Reset every state variable in the simulation.
    
        magI=0;                                               %%% The integrated value of integrated winning-bit amplitudes on the I channel.
        magQ=0;                                               %%% The integrated value of integrated winning-bit amplitudes on the Q channel.
        magnitude=0;                                          %%% A dummy magnitude computed by CORDIC.
        angle=0;                                              %%% A dummy angle computed by CORDIC.
        integI_0=0;                                           %%% 0 bit integrator I state.
        integQ_0=0;                                           %%% 0 bit integrator Q state.
        prev_integI=0;                                        %%% Used to calculate slope.
        prev_integQ=0;
        watchdog_timer=0;                                     %%% watchdog timer - when it expires, the state machine resets.
        locked_timer=0;                                       %%% locked timer - it is used to define an interval over which the integrator slope is measured.
        sym_counter=0;                                        %%% symbol counter - it counts the chips in a modulo 16 fashion.
        bit_counter=0;                                        %%% bit counter - it keeps track of how many bits have been received and checks the crc at the end of the expected packet.
        sqwv=1;                                               %%% The square wave state.
        sqwvI=1;                                              %%% The square wave state for channel I.
        sqwvQ=1;                                              %%% The square wave state for channel Q.
        flip=1;                                               %%% The flip wave state.
        useI=1;                                               %%% The flag to use CR I output // 032416 - To minimize chages to code today, we force this to 1.
        useQ=0;                                               %%% The flag to use CR Q output.
        bits=[];                                              %%% An empty array to hold the bits that are received.
        crc_ccitt16_rx(0,1);                                  %%% Reset the CRC.
        next_state="idle";                                    %%% Set next state to be idle.
        peak_space_vec=zeros(1,3);                            %%% Vector to hold spacings between preamble peaks during preamble sync.
        sym_counter_vec=zeros(1,3);                           %%% Vector to hold the value of the symbol counter at each preamble peak.
        space_counter=0;                                      %%% Counter for spacings between preamble peaks.
        glue=0;                                               %%% Glue flag for preamble sync acquisition.

      case {"idle"}                                           %%% IDLE state.
      
        integI_0+=dataI(loop_a)*sqwvI;                        %%% Keep integrating correlation of input signal against square wave.
        integI_0=bitroll(integI_0,32);        
        integQ_0+=dataQ(loop_a)*sqwvQ;                        %%% square waves derived from the CR circuits 
        integQ_0=bitroll(integQ_0,32);                        %%% No bit saturating, we really need to save LUT plus we can guarantee no overflow.
        
        if(sampleI(loop_a))
          sqwvI=-sqwvI;                                       %%% Generate the I square wave.
        endif
  
        if(sampleQ(loop_a))
          sqwvQ=-sqwvQ;                                       %%% Generate the Q square wave.
        endif
      
        if((abs(integI_0)>4095) || abs(integQ_0)>4095)        %%% Oddly, reducing threshold appears to cause more problems than it solves.
          next_state="locked";                                %%% Once the "IDLE" threshold is passed, move to the next state.
        endif
      
      case {"locked"}                                         %%% "LOCKED" state.
        integI_0+=dataI(loop_a)*sqwvI;                        %%% Keep integrating correlation of input signal against
        integI_0=bitroll(integI_0,32);                        %%% square waves derived from the CR circuits.
        integQ_0+=dataQ(loop_a)*sqwvQ;                        %%% Roll bits over - we must do without saturator.
        integQ_0=bitroll(integQ_0,32);
      
        watchdog_timer+=1;                                    %%% We introduce the watchdog timer.
      
        if(watchdog_timer==watchdog_timer_limit)              %%% If we get stuck in this state, go to the done
          next_state="done";                                  %%% state and send an IRQ to the microcontroller.
          exit_code(loop_a)=3;                                %%% Burn the packet as well.
          exit_code_out(packet_counter)=3;
          irq(loop_a)=1; 
          watchdog_timer=0;
          bits_out(packet_counter)=zeros(1,packet_bits(packet_counter));
          magnitude_out(packet_counter)=0;
          angle_out(packet_counter)=0;
          packet_counter+=1;
        endif
      
        if(sampleI(loop_a))
          sqwvI=-sqwvI;                                       %%% Generate the I square wave.
        end
  
        if(sampleQ(loop_a))
          sqwvQ=-sqwvQ;                                       %%% Generate the Q square wave.
        end 
  
        if(locked_timer==locked_timer_limit1)
          integI_0=0;                                         %%% After waiting for some time, we measure the slope of both I and Q paths.
          integQ_0=0;
        endif
  
        if(locked_timer==locked_timer_limit2)                 %%% At the end of the interval, measure the slope.
          prev_slopeI=sign(integI_0);                         %%% Set up initial variables for SYNC (acq_sym) state.
          prev_slopeQ=sign(integQ_0);
        
          if((abs(integI_0) >= abs(integQ_0)) && abs(integI_0) >= 2048) %%% Oddly, reducing threshold appears to cause more problems than it solves.
            useI=1;                                           %%% If slope is large enough and I is bigger then use clk I
            next_state="acq_sym";
            watchdog_timer=0;
            integI_0=0;
            integQ_0=0;
          elseif((abs(integQ_0) > abs(integI_0)) && abs(integQ_0) >= 2048) %%% Oddly, reducing threshold appears to cause more problems than it solves.
            useI=1;                                           %%% If slope is  large enough and Q is bigger then use clk Q - 032416 - Force this to Use I to minimize code change.
            next_state="acq_sym";
            watchdog_timer=0;
            integI_0=0;
            integQ_0=0;
          else
            next_state="reset";                               %%% If slope is not large enough - we've probably entered this state.
            watchdog_timer=0;                                 %%% due to a bad reason, so reset and try again later.
          endif
        
        endif
      
        locked_timer+=1;                                      %%% Keep running the locked timer unless it is time to check the slope.
      
        case {"acq_sym"}                                      %%% ACQ_SYM (SYNC) state.
    
          integI_0+=dataI(loop_a)*sqwv;                       %%% Keep integrating correlation of input signal against
          integI_0=bitroll(integI_0,32);                      %%% square waves derived from the CR circuits.
          integQ_0+=dataQ(loop_a)*sqwv;                       %%% This time we use a unified "sqwv" driven by the best clock source.
          integQ_0=bitroll(integQ_0,32);
        
          watchdog_timer+=1;                                  %%% We introduce the watchdog timer.
      
          if(watchdog_timer==watchdog_timer_limit)            %%% If we get stuck in this state, go to the done
            next_state="done";                                %%% state and send an IRQ to the microcontroller.
            exit_code(loop_a)=4;                              %%% Burn the packet as well.
            exit_code_out(packet_counter)=4;
            irq(loop_a)=1; 
            bits_out(packet_counter)=zeros(1,packet_bits(packet_counter));
            magnitude_out(packet_counter)=0;
            angle_out(packet_counter)=0;
            packet_counter+=1;
          endif
              
          if(sampleI(loop_a)*useI || sampleQ(loop_a)*useQ)    %%% When a sample from the good CR clock comes in:
          
            sqwv=-sqwv;                                       %%% Toggle the square wave.
            sym_counter=mod((sym_counter+1),16);              %%% Increment the symbol counter modulo 16 (the number of chips in a bit symbol).
            space_counter+=1;                                 %%% Increment the peak spacing counter.
          
            if(space_counter>=64)                             %%% Put a cap on the peak spacing counter.
              space_counter=64;                               %%% We choose 64 because it is 2x that of the maximum spacing we are looking for (32).
            endif
          
            slopeI=sign(integI_0-prev_integI);                %%% Measure the sign of the instantaneous slope.
            slopeQ=sign(integQ_0-prev_integQ);
          
            slope_deltaI=slopeI-prev_slopeI;                  %%% See if there is a change in sign of slope.
            slope_deltaQ=slopeQ-prev_slopeQ;
          
            prev_integI=integI_0;                             %%% Retain memory of previous integrator value.
            prev_integQ=integQ_0;
          
            prev_slopeI=slopeI;                               %%% Retain memory of previous slope value.
            prev_slopeQ=slopeQ;
          
            if((useI && slope_deltaI) || (useQ && slope_deltaQ)) %%% A peak is detected.
              
              if(space_counter<4)                             %%% Peak filtering technique.
                peak_space_vec(3)+=space_counter;             %%% If the spacing is too small - it's caused by noise.
                glue=1;                                       %%% Keep adding onto last spacing value until the small values stop.
              elseif(glue==1)
                peak_space_vec(3)+=space_counter;
                glue=0;
              else
                peak_space_vec=[peak_space_vec(2:3) space_counter]; %%% Retain a vector of spacing values between peaks.
                sym_counter_vec=[sym_counter_vec(2:3) sym_counter]; %%% Also store the symbol numbers where the peak was seen.
              endif                                                 %%% This is used in aligning the integrate-and-dump to the symbol bounds.

              if(loop_a > 14000 && loop_a < 16000 && 0)             %%% Debugging statement.
              useI
              useQ
                loop_a
                peak_space_vec
              endif
              
              if(loop_a > 35000 && loop_a < 37000 && 0)             %%% Debugging statement.
              useI
              useQ
                loop_a
                peak_space_vec
              endif
              
              space_counter=0;                                      %%% Restart the spacing counter since we just went over a peak.
              
              %%% If we see a spacing pattern similar to 32 16 16, then we assume that we have seen the preamble.
              if((peak_space_vec(1)>=29 && peak_space_vec(1)<=34) && (peak_space_vec(2)>=13 && peak_space_vec(2)<=18) && (peak_space_vec(3)>=13 && peak_space_vec(3)<=18))
                 
                  %%% This new code, requiring less hardware, now determines to value of the sym counter that the symbol boundary occurs at:
                  if(1)
                  
                     align_val           = mod(sym_counter_vec(3)+7,16);
                  
                  endif
                  %%% Move to the next state and reset the integrators and watchdog timer.
                  next_state="bit_decisions";
                  watchdog_timer=0;
                  integI_0=0;
                  integQ_0=0;
                
                  burn=1; %%% We will burn the first sample computed here - it should be a partial of the final preamble bit.

              endif
           endif          
        endif
        
        case {"bit_decisions"}
        
          %%% This section needs to be rewritten in order to support the RE/FE scheme to be implemented in FPGA.
        
          watchdog_timer+=1;                        %%% Keep running the watchdog timer.
      
          if(watchdog_timer==watchdog_timer_limit)  %%% Exit to done and throw an interrupt if the watchdog timer expires.
            next_state="done";
            exit_code(loop_a)=5;
            exit_code_out(packet_counter)=5;
            irq(loop_a)=1;
            bits_out(1)=zeros(1,packet_bits(1));    %%% If the packet 1 fails, we must have
            bits_out(2)=zeros(1,packet_bits(2));    %%% data for comparison in packet 2.
            magnitude_out(packet_counter)=0;
            angle_out(packet_counter)=0;
            packet_counter+=1;
          endif
        
          %%% Run matched filtering for both 0 and 1 symbols on both I and Q.
        
          integI_0=integI_0+dataI(loop_a)*sqwv;       %%% This happens on the clock FE.
          integQ_0=integQ_0+dataQ(loop_a)*sqwv;
          integI_0=bitroll(integI_0,32);              %%% Roll over the integrator outputs.
          integQ_0=bitroll(integQ_0,32);
    
          if (sampleI(loop_a)*useI || sampleQ(loop_a)*useQ)
            sqwv=-sqwv;                             %%% At each sample of the best clock, toggle the sqwv.
            sym_counter=mod(sym_counter+1,16);      %%% Also increment the symbol counter so that we know where we are in the symbol.
          
            if(mod(sym_counter-align_val,16)==8)    %%% We are moving into the middle sample of the symbol.
              if(burn>0)
                %%% If it's the first symbol after SYNC, toss it, because it is the final '1' of the preamble.
                integI_0=integQ_0=0;  %%% Dump the integrator values when burning.
              else
  
                prev_half_bit = integI_0 >= 0;
  
                if(useI)
                  sgn_intg=sign(integI_0);        %%% Figure out what the sign of the output was.
                else                              %%% In this fashion, the dominant signal will multiply itself.
                  sgn_intg=sign(integQ_0);        %%% The nondominant signal will be multiplied by the sign of the dominant signal.
                endif                             %%% So, we will be able to average out the noise of the non dominant signal since the sign of the bit symbol keeps changing.
                magI+=sgn_intg*integI_0;          %%% We continue accumulating because in the end we are looking for an average value here.
                magI=bitroll(magI,32);            %%% Bit roll.
                magQ+=sgn_intg*integQ_0;          %%% 
                magQ=bitroll(magQ,32);
                integI_0=integQ_0=0;              %%% Dump integrator value.
                 
              endif
            elseif(sym_counter==align_val)          %%% We are done with the final sample of the symbol.
              if(burn>0)
                burn-=1;                            %%% If it's the first symbol after SYNC, toss it, because it is the final '1' of the preamble.
                integI_0=integQ_0=0;                %%% Dump the integrator values when burning.
              else
                bit_counter=bit_counter+1;          %%% We've received a bit - so increment the bit counter.
  
                if((integI_0 >= 0 && prev_half_bit == 0) || (integI_0 < 0 && prev_half_bit == 1))
                  bits=[bits 1];                    %%% A 1 bit was detected.
                else
                  bits=[bits 0];
                endif  
                  
                if(useI)
                  sgn_intg=sign(integI_0);        %%% Figure out what the sign of the output was.
                else                              %%% In this fashion, the dominant signal will multiply itself.
                  sgn_intg=sign(integQ_0);        %%% The nondominant signal will be multiplied by the sign of the dominant signal.
                endif                             %%% So, we will be able to average out the noise of the non dominant signal since the sign of the bit symbol keeps changing.
                magI+=sgn_intg*integI_0;          %%% We continue accumulating because in the end we are looking for an average value here.
                magI=bitroll(magI,32);            %%% Bit roll.
                magQ+=sgn_intg*integQ_0;          %%% 
                magQ=bitroll(magQ,32);
                integI_0=integQ_0=0;              %%% Dump integrator value.
              
                %%% Compute the CRC.
              
                crc=crc_ccitt16_rx(bits(end),0); 
                crc_out(loop_a)=crc; 
              
                if(packet_bits(packet_counter)==bit_counter) %%% Checking for bit counter length protects us against wacky stuff like attackers playing back long patterns.
                  magI
                  magQ
                  [magnitude,angle]=cordic(floor(magI/packet_bits(packet_counter)),floor(magQ/packet_bits(packet_counter)));  %%% Run CORDIC to estimate RSSI and phase angle.
                  next_state="done";                %%% A successful packet reception!
                  irq(loop_a)=1;                    %%% Send IRQ code shoing successful packet reception.
                  bits_out(packet_counter)=bits;    %%% Send out the bits.
                  magnitude_out(packet_counter)=magnitude;
                  angle_out(packet_counter)=angle;  %%% Also send out the CORDIC information.
                  magI_out(packet_counter)=magI;
                  magQ_out(packet_counter)=magQ;
                  if(crc==1 || packet_bits(packet_counter)==16)
                    exit_code(loop_a)=0;
                    exit_code_out(packet_counter)=0;
                  else
                    exit_code(loop_a)=1;
                    exit_code_out(packet_counter)=1;
                  endif
                  packet_counter+=1;                %%% Increment the packet counter.
                endif
              
                if(bit_counter>=255)                %%% If we ran over the bit counter, then something went wrong.
                  magnitude=0;
                  angle=0;
                  next_state="done";
                  bits_out(packet_counter)=zeros(1,packet_bits(packet_counter));
                  magnitude_out(packet_counter)=0;
                  angle_out(packet_counter)=0;
                  exit_code(loop_a)=2;
                  exit_code_out(packet_counter)=2;
                  irq(loop_a)=1;
                  packet_counter+=1;
                endif
              endif
            endif
          endif
         
          bit_ctr_out(loop_a)=bit_counter;          %%% Send out the bit counter for debugging.
         
          case {"done"}
         
            watchdog_timer=0;
            exit_code(loop_a)=exit_code(loop_a-1); 
         
            %%% Wait for MCU to acknowledge
            if(reset_sys(loop_a)==1)
              next_state="reset";
            end
          
          otherwise
            error("Invalid case value");
      endswitch 
 
  endfor
 
endfunction         

function crc=crc_ccitt16_rx(bit,reset_sys)

%090716 - Finally fix this as per Appendix F in the UHF RFID specification.

persistent shift_reg=ones(1,16);

if(reset_sys==1)
  shift_reg=ones(1,16);
  crc=0;
else
  fb_bit=xor(bit,shift_reg(16));
  shift_reg=[fb_bit shift_reg(1:15)];
  shift_reg(6)=xor(fb_bit,shift_reg(6));
  shift_reg(13)=xor(fb_bit,shift_reg(13));

  if(any(fliplr(shift_reg)!=[0 0 0 1 1 1 0 1 0 0 0 0 1 1 1 1]))  %%CRC=0 unless the CRC out is 1D0Fh.
    crc=0;
  else
    crc=1;
  endif
endif

endfunction
  
function [magnitude,angle]=cordic(magI,magQ)
  
  %Implement CORDIC vectoring operation - that is: figure out the angle and magnitude by applying
  %rotations until the Q component is as close to zero as the specified number of iterations allow.
  
  num_iters=16;
  
  cordic_angles=round(atan(2.^-(0:27))*(2^13));
  cordic_kvals=round(cumprod(1./abs(1+j*2.^-(0:27)))*(2^13));
  
  if(magQ < 0) % Then the angle is in the lower two quadrants, perform a +90 degree rotation.
    cum_angle=-round(0.5*pi*(2^13));
    vec=[-magQ,magI];
  else         % Then the angle is in the upper two quadrants, perform a -90 degree rotation.
    cum_angle=round(0.5*pi*(2^13));
    vec=[magQ,-magI];
  endif
  
  for loop_i=1:num_iters
    vec1temp=vec(1);
    vec2temp=vec(2);
    vec(1)-=floor(vec2temp*(-sign(vec2temp))*(2^-(loop_i-1)));
    vec(2)+=floor(vec1temp*(-sign(vec2temp))*(2^-(loop_i-1)));
    cum_angle+=sign(vec2temp)*cordic_angles(loop_i);
  end
  
  angle=cum_angle;
  magnitude=vec(1); %ignore scaling factor for now.
  
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

%%% We define this function because we cannot afford saturators in many locations.
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