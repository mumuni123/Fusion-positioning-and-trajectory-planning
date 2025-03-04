function [Fk, Ft] = kffk(ins, varargin)
% Create Kalman filter system transition matrix.
%
% Prototype: [Fk, Ft] = kffk(ins, varargin)
% Inputs: ins - SINS structure array, if not struct then nts=ins;
%         varargin - if any other parameters
% Outputs: Fk - discrete-time transition matrix, = Phikk_1
%          Ft - continuous-time transition matirx
%
% See also  kfhk, kfinit, kfupdate, kfc2d, insupdate, etm, psinstypedef.

% Copyright(c) 2009-2014, by Gongmin Yan, All rights reserved.
% Northwestern Polytechnical University, Xi An, P.R.China
% 06/08/2012, 01/02/2014, 02/08/2016
global psinsdef
    %% get Ft
    if isstruct(ins),    nts = ins.nts;
    else                 nts = ins;
    end
    switch(psinsdef.kffk)
        case 15,
            Ft = etm(ins);
        case {18,19} % psinsdef.kffkxx, xx=18,19
            Ft = etm(ins);
            Ft(psinsdef.kffk, psinsdef.kffk) = 0;
        case {24}
            Ft = etm(ins);
            Ft(psinsdef.kffk,psinsdef.kffk) = 0;
            % 15+dKg(9)
            Ft(1:3,16:24) = [-ins.wib(1)*ins.Cnb, -ins.wib(2)*ins.Cnb, -ins.wib(3)*ins.Cnb];
        case {33}
            Ft = etm(ins);
            Ft(psinsdef.kffk,psinsdef.kffk) = 0;
            % 15+dKg(9)+dKa(6)+Ka2(3)
            Ft(1:3,16:24) = [-ins.wib(1)*ins.Cnb, -ins.wib(2)*ins.Cnb, -ins.wib(3)*ins.Cnb];
            Ft(4:6,25:33) = [ins.fb(1)*ins.Cnb, ins.fb(2)*ins.Cnb(:,2:3), ins.fb(3)*ins.Cnb(:,3), ins.Cnb*diag(ins.fb.^2)];
        case {34, 37}
            Ft = etm(ins);
            Ft(psinsdef.kffk,psinsdef.kffk) = 0;
            % 19+dKg(9)+dKa(6)
            Ft(1:3,20:28) = [-ins.wib(1)*ins.Cnb, -ins.wib(2)*ins.Cnb, -ins.wib(3)*ins.Cnb];
            Ft(4:6,29:34) = [ins.fb(1)*ins.Cnb, ins.fb(2)*ins.Cnb(:,2:3), ins.fb(3)*ins.Cnb(:,3)];
        otherwise,
%             Ft = feval(psinsdef.typestr, psinsdef.kffktag, {ins, varargin});
            Ft = feval(psinsdef.typestr, psinsdef.kffktag, [{ins},varargin]);
    end
    %% discretization
	Fk = Ft*nts;
    if nts>0.1  % for large time interval, this may be more accurate.
        Fk = expm(Fk);
    else   % Fk = I + Ft*nts + 1/2*(Ft*nts)^2  , 2nd order expension
        Fk = eye(size(Ft)) + Fk;% + Fk*Fk*0.5; 
    end
    
