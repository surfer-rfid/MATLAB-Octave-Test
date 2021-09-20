function plotSpectrum(X,fin,Fs,fmt)
% plotSpectrum(X,fin,fmt) Plot a smoothed spectrum
if nargin<4
    fmt='-';
end
[f p] = logsmooth(X,fin,8,3);
semilogx(f*Fs,p,fmt);
