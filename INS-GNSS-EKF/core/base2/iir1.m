function y = iir1(x, afa)
% yk = (1-afa)*yk_1 + afa*xk.
%
% Prototype: y = irr1(x, afa)
% Inputs: x - input data
%         afa - correlation time
% Outputs: y - output data
%
% Example:
%    x = randn(100,1)+1; figure, plot([x, iir1(x)]); grid on;
%
% See also  markov1, ar1coefs, iir1iir1, ar1filt.

% Copyright(c) 2009-2022, by Gongmin Yan, All rights reserved.
% Northwestern Polytechnical University, Xi An, P.R.China
% 05/10/2022
    if nargin<2,  afa = 0.1;  end
    x0 = x(1,:);
    for k=1:size(x,2), x(:,k)=x(:,k)-x0(k); end
    y = filter(afa, [1 -(1-afa)], x);
    for k=1:size(x,2), y(:,k)=y(:,k)+x0(k); end
