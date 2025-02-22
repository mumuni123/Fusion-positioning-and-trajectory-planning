%% 用于路径回溯的findWayBack函数
 
%findWayBack函数用来进行路径回溯，这个函数的输入参数是终止点goalposind和矩阵fieldpointers，输出参数是P
function p = findWayBack(n,goalposind,fieldpointers)
 
    posind = goalposind;
    [px,py,pz] = ind2sub([n,n n],posind); % 将索引值posind转换为坐标值 [px,py,pz]
    p = [px py pz];
    
    %利用while循环进行回溯，当我们回溯到起始点的时候停止，也就是在矩阵fieldpointers中找到S时停止
    while ~strcmp(fieldpointers{posind},'S')
      switch fieldpointers{posind}
          
        case 'L' % ’L’ 表示当前的点是由左边的点拓展出来的
          px = px - 1;
        case 'R' % ’R’ 表示当前的点是由右边的点拓展出来的
          px = px + 1;
        case 'U' % ’U’ 表示当前的点是由上面的点拓展出来的
          pz = pz + 1;
        case 'D' % ’D’ 表示当前的点是由下边的点拓展出来的
          pz = pz - 1;
        case 'B' % ’B’ 表示当前的点是由后面的点拓展出来的
          py = py - 1;
        case 'F' % ’F’ 表示当前的点是由前面的点拓展出来的
          py = py + 1;
        case 'DB' % ’DB’ 表示当前的点是由下后的点拓展出来的
          py = py - 1; pz = pz - 1;
        case 'DF' % ’DF’ 表示当前的点是由下前的点拓展出来的
          py = py + 1; pz = pz - 1;
        case 'UB' % ’UB’ 表示当前的点是由上后的点拓展出来的
          py = py - 1; pz = pz + 1;
        case 'UF' % ’UF’ 表示当前的点是由上前的点拓展出来的
          py = py + 1; pz = pz + 1;
        case 'RB' % ’RB’ 表示当前的点是由右后的点拓展出来的
          px = px + 1; py = py - 1;
        case 'RF' % ’RF’ 表示当前的点是由右前的点拓展出来的
          px = px + 1; py = py + 1;
        case 'LB' % ’LB’ 表示当前的点是由左后的点拓展出来的
          px = px - 1; py = py - 1;
        case 'LF' % ’RF’ 表示当前的点是由右前的点拓展出来的
          px = px - 1; py = py + 1;
        case 'DR' % ’DR’ 表示当前的点是由下右的点拓展出来的
          px = px + 1; pz = pz - 1;
        case 'DL' % ’DL’ 表示当前的点是由下左的点拓展出来的
          px = px - 1; pz = pz - 1;
        case 'UR' % ’UR’ 表示当前的点是由上右的点拓展出来的
          px = px + 1; pz = pz + 1;
        case 'UL' % ’UL’ 表示当前的点是由上左的点拓展出来的
          px = px - 1; pz = pz + 1;
          
      end
      p = [p; px py pz];
      posind = sub2ind([n n n],px,py,pz);% 将坐标值转换为索引值
    end
end