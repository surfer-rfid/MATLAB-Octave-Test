function [v,x0_out] = simulateDSM_oneshotQ(u,arg2,nlev,reset)
%09192021 - This file was modified a bit for the RFID Octave simulations to permit retention of state while run in a larger loop.
%[v,xn,xmax,y] = simulateDSM(u,ABCD,nlev=2,x0=0)
% or
%[v,xn,xmax,y] = simulateDSM(u,ntf,nlev=2,x0=0)
%
%Compute the output of a general delta-sigma modulator with input u,
%a structure described by ABCD, an initial state x0 (default zero) and 
%a quantizer with a number of levels specified by nlev.
%Multiple quantizers are implied by making nlev an array,
%and multiple inputs are implied by the number of rows in u.
%
%Alternatively, the modulator may be described by an NTF.
%The NTF is zpk object. (The STF is assumed to be 1.)
%The structure that is simulated is the block-diagional structure used by
%zp2ss.m.
%
% 110415 - simulateDSM_oneshot
%
% This file permits calling the simulateDSM argument one iteration at a time so that we can put it
% in a feedback loop containing the TX leakage cancellation algorithm.

persistent x0=0;
persistent A=0;
persistent B=0;
persistent C=0;
persistent D1=0;

if nargin<2
    fprintf(1,'Error. simulateDSM needs at least two arguments.\n');
    return
end

if(reset)
  
  ntf=struct();
  ntf.('k') = arg2.k;
  ntf.('zeros') = arg2.z;
  ntf.('poles') = arg2.p;
  order = length(ntf.zeros);
  
  x0=zeros(order,1);
  
    [A,B2,C,D2] = zp2ss(ntf.poles,ntf.zeros,-1);	% A realization of 1/H
    % Transform the realization so that C = [1 0 0 ...]
    Sinv = orth([C' eye(order)])/norm(C); S = inv(Sinv);
    C = C*Sinv;
    if C(1)<0
	S = -S;
	Sinv = -Sinv;
    end
    A = S*A*Sinv; B2 = S*B2; C = [1 zeros(1,order-1)]; % C=C*Sinv; 
    D2 = 0;
    % !!!! Assume stf=1
    B1 = -B2;
    D1 = 1;
    B = [B1 B2];
  
endif

    y = C*x0 + D1*u;
    v = ds_quantize(y,nlev);
    x0 = A * x0 + B * [u;v];

    x0_out=x0;
    
endfunction

function v = ds_quantize(y,n)
%v = ds_quantize(y,n)
%Quantize y to 
% an odd integer in [-n+1, n-1], if n is even, or
% an even integer in [-n, n], if n is odd.
%
%This definition gives the same step height for both mid-rise
%and mid-tread quantizers.

if rem(n,2)==0	% mid-rise quantizer
    v = 2*floor(0.5*y)+1;
else 		% mid-tread quantizer
    v = 2*floor(0.5*(y+1));
end

% Limit the output
for qi=1:length(n)	% Loop for multiple quantizers
    L = n(qi)-1;
    i = v(qi,:)>L; 
    if any(i)
	v(qi,i) = L;
    end
    i = v(qi,:)<-L;
    if any(i)
	v(qi,i) = -L;
    end
end

endfunction