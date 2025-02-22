function ins = insupdate_grid(ins, imu)
% Grid SINS Updating Alogrithm.
%
% Prototype: ins = insupdate_grid(ins, imu)
% Inputs: ins - SINS structure array created by function 'insinit_grid'
%         imu - gyro & acc incremental sample(s)
% Output: ins - SINS structure array after updating
%
% See also  insinit_grid, insinit_ecef, insinit, insupdate_ecef.

% Copyright(c) 2009-2024, by Gongmin Yan, All rights reserved.
% Northwestern Polytechnical University, Xi An, P.R.China
% 28/11/2024
global glv
    nn = size(imu,1);
    nts = nn*ins.ts;  nts2 = nts/2;  ins.nts = nts;
    [phim, dvbm] = cnscl(imu,0);    % coning & sculling compensation
    ins.wib = phim/nts; ins.fb = dvbm/nts;
    %% earth & angular rate updating
    vG01 = ins.vG+ins.aG*nts2; pe01 = ins.pe+ins.CeG*vG01*nts2;  % extrapolation at t1/2
    [llh, RN, sl] = xyz2llh(pe01);
    RNh = RN+llh(3); RMh = RN*(1-glv.e2)/(1-glv.e2*sl^2)+llh(3);
    [ins.CeG, slat, clat, slon, clon, zeta] = pos2ceg(llh);
    ssigma = slon*slat*zeta; csigma = clon*zeta;
    Rx = ssigma^2/RMh+csigma^2/RNh;  Ry = csigma^2/RMh+ssigma^2/RNh;  % 1/Rx,1/Ry
    tau = (1/RMh-1/RNh)*ssigma*csigma; kappa = slon*clat*zeta;
    wGie = glv.wie*[-clat*ssigma; clat*csigma; slat];
    wGeG = [tau -Ry; Rx -tau; kappa*tau -kappa*Ry]*ins.vG(1:2);
    wGiG = wGie+wGeG;
    g = glv.g0*(1+5.2790414e-3*sl^2+2.32718e-5*sl^4)-3.086e-6*llh(3);
    gG01 = [0;0;-g];
    %% (1)velocity updating
    gcc = -cross(2*wGie+wGeG,vG01) + gG01;
    ins.fG = qmulv(ins.qGb, ins.fb);
    ins.aG = rotv(-wGiG*nts2, ins.fG) + gcc;
    vG1 = ins.vG + ins.aG*nts;
    %% (2)position updating
    ins.pe = ins.pe + ins.CeG*(ins.vG+vG1)*nts2;
    ins.vG = vG1;
    %% (3)attitude updating
    ins.qGb = qupdt2(ins.qGb, phim, wGiG*nts);
    ins.avp = [q2att(ins.qGb); ins.vG; ins.pe];
    
%     CnG= pos2cen(llh)'*ins.CeG;
%     ins.avp = [m2att(CnG*q2mat(ins.qGb)); CnG*ins.vG; llh];

