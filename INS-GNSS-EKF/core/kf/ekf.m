

function kf = ekf(kf, yk, TimeMeasBoth)
% Extended Kalman filter for nonlinear system.
%
% Prototype: kf = ekf(kf, yk)
% Inputs: kf - filter structure array
%         yk - measurement vector
%         TimeMeasBoth - described as follows,
%            TimeMeasBoth='T' (or nargin==1) for time updating only, 
%            TimeMeasBoth='M' for measurement updating only, 
%            TimeMeasBoth='B' (or nargin==2) for both time and 
%                             measurement updating.
% Output: kf - output filter structure array
%
% See also  ekfJcb, ukf, ckf, kfupdate.

% Copyright(c) 2009-2022, by Gongmin Yan, All rights reserved.
% Northwestern Polytechnical University, Xi An, P.R.China
% 27/09/2012, 04/03/2022, 12/06/2024 (Improved)

%  21/2/2025  对严恭敏老师的算法进行优化：增加输入验证、预测状态验证、协方差矩阵对称化等

    % Validate TimeMeasBoth input
    if nargin == 1
        TimeMeasBoth = 'T';
    elseif nargin == 2
        TimeMeasBoth = 'B';
    end
    validFlags = {'T', 'M', 'B'};
    if ~any(strcmp(TimeMeasBoth, validFlags))
        error('Invalid TimeMeasBoth flag. Use ''T'', ''M'', or ''B''.');
    end

    if TimeMeasBoth == 'T' || TimeMeasBoth == 'B'
        if isfield(kf, 'fx')  % Nonlinear state transition
            [kf.Phikk_1, kf.xkk_1] = ekfJcb(kf.fx, kf.xk, kf.px);
            % Ensure predicted state is calculated correctly
            if isempty(kf.xkk_1)
                error('State transition function must return predicted state.');
            end
        else  % Linear state transition
            kf.xkk_1 = kf.Phikk_1 * kf.xk;
        end
        % Symmetrize covariance after prediction
        kf.Pxkk_1 = kf.Phikk_1 * kf.Pxk * kf.Phikk_1' + kf.Gammak * kf.Qk * kf.Gammak';
        kf.Pxkk_1 = 0.5 * (kf.Pxkk_1 + kf.Pxkk_1');
        
        if TimeMeasBoth == 'T'    % Time update only
            kf.xk = kf.xkk_1; 
            kf.Pxk = kf.Pxkk_1;
            return;
        end
    end

    if TimeMeasBoth == 'M' || TimeMeasBoth == 'B'
        if TimeMeasBoth == 'M'    % Measurement update only
            kf.xkk_1 = kf.xk; 
            kf.Pxkk_1 = kf.Pxk;
        end
        if isfield(kf, 'hx')  % Nonlinear measurement
            [kf.Hk, kf.ykk_1] = ekfJcb(kf.hx, kf.xkk_1, kf.py);
            if isempty(kf.ykk_1)
                error('Measurement function must return predicted measurement.');
            end
        else  % Linear measurement
            kf.ykk_1 = kf.Hk * kf.xkk_1;
        end
        
        kf.Pxykk_1 = kf.Pxkk_1 * kf.Hk';
        kf.Pykk_1 = kf.Hk * kf.Pxykk_1 + kf.Rk;
        kf.Pykk_1 = 0.5 * (kf.Pykk_1 + kf.Pykk_1');  % Symmetrize
        
        % Stable Kalman gain calculation using Cholesky decomposition
        [L, flag] = chol(kf.Pykk_1, 'lower');
        if flag ~= 0
            error('Innovation covariance matrix is not positive definite.');
        end
        Kk = (kf.Pxykk_1 / L') / L;
        
        % Update state and covariance
        kf.xk = kf.xkk_1 + Kk * (yk - kf.ykk_1);
        kf.Pxk = kf.Pxkk_1 - Kk * kf.Pykk_1 * Kk';
        kf.Pxk = 0.5 * (kf.Pxk + kf.Pxk');  % Ensure symmetry
    end
end
