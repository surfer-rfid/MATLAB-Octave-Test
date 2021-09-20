%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%                                                                              %
% txCancelAlgorithmFast                                                        %
%                                                                              %
% Filename: txCancelAlgorithmFast.m                                            %
% Creation Date: 11/04/2015                                                    %
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
% This is the same algorithm as from testMatchingNetwork25Algo100615H.         %
% It requires that the step matrix and the cap matrix be loaded onto the       %
% next highest level of the simulation.                                        %
%                                                                              %
% 110515 - Add gain adjustment.                                                %
% 111715 - Clean up and add comments while placing in the release directory    %
%                                                                              %
% This file was created to be part of the closed loop SDM-TX cancellation      %
% simulation. As such, it has a persistent memory of the capacitor settings    %
% The intent is to call this file once per baseband sampling period            %
% and the function will spit out the next capacitor value setting.             %
% In real life, the output of this circuit will need to connect to an SPI      %
% controller.                                                                  %
%                                                                              %
% 123015 - Altered to reflect the minimal-LUT implementation that was          %
% eventually adopted.                                                          %
% 022116 - Modified to support the 2-bit Jacobian Matrix                       %
% 031516 - Modified to support multiplication by simple LUTs.                  %
% 100817 - Modified to match current digital implementation, support for       %
% data-driven gain changes, etc.                                               %
%                                                                              %
% 112517 - Match what was implemented in verilog                               %
% 120117 - Slow convergence constant. Update measured gain values.             %
% Put initial state back to where it was for this data set. (13376, 15488)     %
%                                                                              %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function cap_vec_return=txCancelAlgorithmFast(dc_i,dc_q,step_mat_A_qtz,step_mat_B_qtz,step_mat_C_qtz,step_mat_D_qtz,reset)
  
    persistent  cap1_state=13376; %%% Keep the capacitor settings as persistent variables so that we can update them with the next information sample.
    persistent  cap3_state=15488;
    persistent  prev_error=0;
    persistent  mag_delta_c1=0;
    persistent  loop_mode=0;
    persistent  burn=0;
    persistent  c1_flag=0;
    persistent  c3_flag=0;
    persistent  fail_flag=0;
    persistent  fail_ctr=0;
    persistent  lna_gain_state=0;
    persistent  gain_bits=0;
  
    %%% Add provision to reset the capacitor vector before running the algorithm.
  
    if(reset)
      cap1_state=13376; %%% Keep the capacitor settings as persistent variables so that we can update them with the next information sample.
      cap3_state=15488;
      prev_error=0;
      mag_delta_c1=0;
      loop_mode=0;
      burn=0;
      c1_flag=0;
      c3_flag=0;
      fail_ctr=0;
      fail_flag=0;
      lna_gain_state=0;
      gain_bits=0;
      cap_vec_return=[13 2 15 4 10^(-23/20)]; %Must manually update this if we change the op point
      return;
    endif
    
    %%% Perform the gradient descent calculation.
  
    curr_error=max(abs(dc_i),abs(dc_q))+floor(min(abs(dc_i),abs(dc_q))/4);
    
    gain_bits_temp=gain_bits;
    
    while (gain_bits_temp)
      curr_error=floor(curr_error/2); %This may be too much for gain_bits=7
      gain_bits_temp-=1;
    end
    
    cap_vec(1)=bitand(31,floor(cap1_state/(2^10)));
    cap_vec(2)=bitand(31,floor(cap1_state/(2^5)));
    cap_vec(3)=bitand(31,floor(cap3_state/(2^10)));
    cap_vec(4)=bitand(31,floor(cap3_state/(2^5)));
 
    divisor=2; %Update to 2 to reflect actual hardware implementation.
 
    step_vec=[0 0].';
      
    if(burn==1) 
      if(loop_mode==0)
        prev_error=curr_error;
        if(c1_flag==0)
          if(gain_bits==7)
            step_vec(1)=32;
          else
            step_vec(1)=max(floor(prev_error/divisor),32);
          endif
        else
          if(gain_bits==7)
            step_vec(1)=-32;
          else
            step_vec(1)=-max(floor(prev_error/divisor),32);
          endif
        endif
      elseif(loop_mode==1) 
        if(curr_error >= prev_error)
          fail_ctr+=1;
          if(c1_flag==0)
            if(gain_bits==7)
              step_vec(1)=-32;
            else
              step_vec(1)=-max(floor(prev_error/divisor),32);
            endif
          else
            if(gain_bits==7)
              step_vec(1)=32;
            else
              step_vec(1)=max(floor(prev_error/divisor),32);
            endif
          endif
          c1_flag=!c1_flag;
        else
          fail_ctr=0;
        endif
      elseif(loop_mode==2)
        prev_error=curr_error;
        if(c3_flag==0)
          if(gain_bits==7)
            step_vec(2)=32;
          else
            step_vec(2)=max(floor(prev_error/divisor),32);
          endif
        else
          if(gain_bits==7)
            step_vec(2)=-32;
          else
            step_vec(2)=-max(floor(prev_error/divisor),32);
          endif
        endif
      elseif(loop_mode==3) 
        if(curr_error >= prev_error)
          fail_ctr+=1;
          if(c3_flag==0)
            if(gain_bits==7)
              step_vec(2)=-32;
            else
              step_vec(2)=-max(floor(prev_error/divisor),32);
            endif
          else
            if(gain_bits==7)
              step_vec(2)=32;
            else
              step_vec(2)=max(floor(prev_error/divisor),32);
            endif
          endif
          c3_flag=!c3_flag;
        else
          fail_ctr=0;
        endif       
      endif
    endif
    
    %disp(sprintf("LM: %d CE: %d\t DCI:%d\t DCQ:%d\t C1:%d\t C2:%d\t C3:%d\t C4:%d\t GB:%d\t FC:%d\t",loop_mode,curr_error,dc_i,dc_q,cap_vec(1),cap_vec(2),cap_vec(3),cap_vec(4),gain_bits,fail_ctr));
 
    
    %if(fail_ctr >= 8)
    %     if(!fail_flag || loop_mode==0)
    %      step_vec=2*step_vec;
    %      fail_ctr=0;
    %      fail_flag=~fail_flag;
    %     elseif
    %      (fail_flag || loop_mode==2)
    %      step_vec=2*step_vec;
    %      fail_ctr=0;
    %      fail_flag=~fail_flag;
    %     endif
    %end
    
    if(fail_ctr >= 7)
         step_vec=2*step_vec;
         fail_ctr=0;
    end
    
    if(!((lna_gain_state==2) && !(curr_error > 23))) %This is a bit hokey. Best idea may be to stop when we reach the minimum at highest gain? Same as curr err < 9
      cap1_state+=floor(step_vec(1));
      cap3_state+=floor(step_vec(2));
    end
    
    lna_gain_state_new=lna_gain_state;
    
    if(loop_mode==3)
      if(lna_gain_state==0)
        if(curr_error > 16383)
          cap1_state=16384;
          cap2_state=16384;
        elseif(!(curr_error > 255))
          lna_gain_state_new=1;
        endif
      elseif(lna_gain_state==1)
        if(curr_error > 2047)
          lna_gain_state_new=0;
        elseif(!(curr_error > 31))
          lna_gain_state_new=2;
        endif      
      elseif(lna_gain_state==2)
        if(curr_error > 127)
          lna_gain_state_new=1;
        endif
      else
        error("Problem in tx cancel");
      endif
    endif
    
    lna_gain_state=lna_gain_state_new;
    %%% Make sure that any steps do not exceed the bounds of the capacitor control values.
    
    cap1_state=max(min(cap1_state,32767),0);
    cap3_state=max(min(cap3_state,32767),0);
    
    %%% Break apart the control values for the four capacitors so that they can be individually addressed over SPI
    
    cap_vec(1)=bitand(31,floor(cap1_state/(2^10)));
    cap_vec(2)=bitand(31,floor(cap1_state/(2^5)));
    cap_vec(3)=bitand(31,floor(cap3_state/(2^10)));
    cap_vec(4)=bitand(31,floor(cap3_state/(2^5)));
    if(lna_gain_state==0)
      cap_vec(5)=10^(-23/20);
      gain_bits=0;
    elseif(lna_gain_state==1)
      cap_vec(5)=10^(-3/20);
      gain_bits=3;
    elseif(lna_gain_state==2)
      cap_vec(5)=10^(20/20);
      gain_bits=7;
    else
      error("Someone made a boo boo with LNA gain state transitions");
    endif
    %disp(sprintf("LM: %d CE: %d\t SV1:%d\t SV2:%d\t DCI:%d\t DCQ:%d\t C1:%d\t C2:%d\t C3:%d\t C4:%d\t",loop_mode,curr_error,step_vec(1),step_vec(2),dc_i,dc_q,cap_vec(1),cap_vec(2),cap_vec(3),cap_vec(4)));
 
    if(loop_mode==3 && burn==0)
      burn=1;
    endif
    
    loop_mode=mod(loop_mode+1,4);
 
    cap_vec_return=cap_vec;
 
endfunction