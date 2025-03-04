function [data, idx] = reminct(data)
% remain data strictly monotonic increasing by time-tag(REMain INCreaseing Time)
%
% Prototype:  [data, idx] = monoinc(data)
% Input: data - input data, the last column is time tag
% Outputs: data - output data
%          idx - increasing data index
%
% See also inct, sortt, monoinc.

% Copyright(c) 2009-2021, by Gongmin Yan, All rights reserved.
% Northwestern Polytechnical University, Xi An, P.R.China
% 09/05/2021
    n = length(data);
    t = data(:,end);
    idx = (1:n)';
    for k=2:n
        if t(k)<=t(k-1), idx(k)=0; t(k)=t(k-1)+1e-10; end
    end
    idx = idx(idx>0);  % for delete
    data = data(idx,:);