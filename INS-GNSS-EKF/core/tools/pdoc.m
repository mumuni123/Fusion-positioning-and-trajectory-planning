function pdoc(typ)
% Open PSINS m-function & demo introduction doc
    rpath = psinsenvi();
    if nargin<1, typ=0; end
    if typ==0
        system(['start ', rpath,'\doc\myword.lnk ', rpath,'\doc\PSINS����������.doc']);
%         system(['start WPS Office.exe ', rpath,'\doc\PSINS����������.doc']);
    else
        system(['start word.exe ', rpath,'\doc\PSINS����������.doc']);
    end
