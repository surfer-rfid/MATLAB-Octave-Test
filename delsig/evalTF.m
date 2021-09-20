function h = evalTF(tf,z)
%h = evalTF(tf,z)
%Evaluates the rational function described by the struct tf
% at the point(s) given in the z vector.
% TF must be either a zpk object or a struct containing
%   form		'zp' or 'coeff'
%   zeros,poles,k	if form=='zp'
%   num,den		if form=='coeff'
%
% In Matlab 5, the ss/freqresp() function does nearly the same thing.


	h = tf.k * evalRPoly(tf.z,z) ./ evalRPoly(tf.p,z);
