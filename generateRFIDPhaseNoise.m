function [noise_cplx]=generateRFIDPhaseNoise(len_out,ana_Fs)

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%                                                                           %
% 091615 - generateRFIDPhaseNoise                                           %
%                                                                           %
% This file is slightly modified from the pn_generator() function by        %
% Markus Nentwig at www.dsprelated.com/showcode/246.php.                    %
%                                                                           %
% Permission to publish this code was obtained from the original author     %
% provided that the license below is included in the header:                %
%                                                                           %
% Copyright 2021, Markus Nentwig                                            %
% Permission is hereby granted, free of charge, to any person               %
% obtaining a copy of this software and associated documentation files      %
% (the "Software"), to deal in the Software without restriction,            %
% including without limitation the rights to use, copy, modify, merge,      %
% publish, distribute, sublicense, and/or sell copies of the Software,      %
% and to permit persons to whom the Software is furnished to do so,         %
% subject to the following conditions:                                      %
% The above copyright notice and this permission notice shall be            %
% included in all copies or substantial portions of the Software.           %
% THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,           %
% EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF        %
% MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.    %
% IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY      %
% CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT,      %
% TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE         %
% SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.                    %
%                                                                           %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% ****************************************************************
% PN generator configuration
% ****************************************************************

srcPar = struct();
srcPar.n = 2^ceil(log(len_out)/log(2)); % generated number of output samples
srcPar.rate_Hz = ana_Fs; % sampling rate
% The phase noise numbers below are from the STS1TX transmitter IC
%srcPar.f_Hz = [0, 10e3, 100e3, 200e3, 500e3, 1e6, 2e6, 48e6]; % phase noise spectrum, frequencies
%srcPar.g_dBc1Hz = [-90, -94, -99, -100, -107, -116, -119, -156]; % phase noise spectrum, magnitude
% The phase noise numbers below are from the SX1257 IC, with unknown values interpolated with curve of STS1TX transmitter IC
srcPar.f_Hz = [0, 10e3, 100e3, 200e3, 500e3, 1e6, 2e6, 10e6, 20e6]; % phase noise spectrum, frequencies
srcPar.g_dBc1Hz = [-89, -93, -98, -102, -106, -115, -118, -128, -128]; % phase noise spectrum, magnitude - update to reflect what's in the paper, more or less

srcPar.spursF_Hz = []; % discrete spurs (set [] if not needed)
srcPar.spursG_dBc = []; % discrete spurs, power relative to carrier
    
% ****************************************************************
% run PN generator
% ****************************************************************

s = PN_src(srcPar);
noise_cplx=s(1:len_out);

endfunction

function pn_td = PN_src(varargin)
    def = {'includeCarrier', false, ...
           'spursF_Hz', [], ...
           'spursG_dBc', [], ...
           'fMax_Hz', []};
    p = vararginToStruct(def, varargin);
    
    % length of signal in the time domain (after ifft)
    len_s = p.n / p.rate_Hz;

    % FFT bin frequency spacing
    deltaF_Hz = 1 / len_s;
    
    % construct AWGN signal in the frequency domain     
    % a frequency domain bin value of n gives a time domain power of 1
    % for example ifft([4 0 0 0]) => 1 1 1 1 
    % each bin covers a frequency interval of deltaF_Hz
    mag = p.n;
    
    % scale "unity power in one bin" => "unity power per Hz":
    % multiply with sqrt(deltaF_Hz): 
    mag = mag * sqrt(deltaF_Hz);
    
    % Create noise according to mag in BOTH real- and imaginary value
    mag = mag * sqrt(2);

    % both real- and imaginary part contribute unity power => divide by sqrt(2)
    pn_fd = mag / sqrt(2) * (randn(1, p.n) + 1i * randn(1, p.n));
    
    % frequency vector corresponding to the FFT bins (0, positive, negative)
    fb_Hz = FFT_freqbase(p.n, deltaF_Hz);
    
    % interpolate phase noise spectrum on frequency vector
    % note: interpolate dB on logarithmic frequency axis
    H_dB = interp1(log(p.f_Hz+eps), p.g_dBc1Hz, log(abs(fb_Hz)+eps), 'linear');

    % dB => magnitude
    H = 10 .^ (H_dB / 20);
    %    H = 1e-6; % sanity check: enforce flat -120 dBc in 1 Hz
    
    % apply filter to noise spectrum
    pn_fd = pn_fd .* H;

    % set spurs
    for ix = 1:numel(p.spursF_Hz)
        fs = p.spursF_Hz(ix);
        u = abs(fb_Hz - fs);
        ix2 = find(u == min(u), 1);
    
    % random phase
        rot = exp(2i*pi*rand());
        
        % bin value of n: unity (carrier) power (0 dBc)
        % scale with sqrt(2) because imaginary part will be discarded
        % scale with sqrt(2) because the tone appears at both positive and negative frequencies
        smag = 2 * p.n * 10 ^ (p.spursG_dBc(ix) / 20);
        pn_fd(ix2) = smag * rot;
    end
    
    % limit bandwidth (tool to avoid aliasing in an application
    % using the generated phase noise signal)
    if ~isempty(p.fMax_Hz)
        pn_fd(find(abs(fb_Hz) > p.fMax_Hz)) = 0;
    end
    
    % convert to time domain
    pn_td = ifft(pn_fd);

    % discard imaginary part 
    pn_td = real(pn_td);

    % Now pn_td is a real-valued random signal with a power spectral density 
    % as specified in f_Hz / g_dBc1Hz.
    
    % phase-modulate to carrier
    % note: d/dx exp(x) = 1 near |x| = 1
    % in other words, the phase modulation has unity gain for small phase error
    pn_td = exp(i*pn_td);
      
    if ~p.includeCarrier
        % remove carrier
        % returns isolated phase noise component
        pn_td = pn_td - 1;
    end
endfunction

% returns a vector of frequencies corresponding to n FFT bins, when the
% frequency spacing between two adjacent bins is deltaF_Hz
function fb_Hz = FFT_freqbase(n, deltaF_Hz)
    fb_Hz = 0:(n - 1);
    fb_Hz = fb_Hz + floor(n / 2);
    fb_Hz = mod(fb_Hz, n);
    fb_Hz = fb_Hz - floor(n / 2);
    fb_Hz = fb_Hz * deltaF_Hz;
endfunction

% *************************************************************
% helper function: Parse varargin argument list
% allows calling myFunc(A, A, A, ...)
% where A is
% - key (string), value (arbitrary) => result.key = value
% - a struct => fields of A are copied to result
% - a cell array => recursive handling using above rules
% *************************************************************
function r = vararginToStruct(varargin)
% note: use of varargin implicitly packs the caller's arguments into a cell array
% that is, calling vararginToStruct('hello') results in
%   varargin = {'hello'}
    r = flattenCellArray(varargin, struct());
endfunction

function r = flattenCellArray(arr, r)
    ix=1;
    ixMax = numel(arr);
    while ix <= ixMax
        e = arr{ix};
        
        if iscell(e)
            % cell array at 'key' position gets recursively flattened
            % becomes struct
            r = flattenCellArray(e, r);
        elseif ischar(e)
            % string => key.
            % The following entry is a value
            ix = ix + 1;
            v = arr{ix};
            % store key-value pair
            r.(e) = v;
        elseif isstruct(e)
            names = fieldnames(e);
            for ix2 = 1:numel(names)
                k = names{ix2};
                r.(k) = e.(k);
            end
        else
            e
            assert(false)
        end
        ix=ix+1;
    end % while
endfunction