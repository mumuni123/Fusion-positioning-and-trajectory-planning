%% 用于拓展的findFValue函数，由一个父节点拓展多个子节点，分别计算出costs和heuristic
 
%这个函数的作用就是把输入的点作为父节点，然后进行拓展找到子节点，并且找到子节点的代价，并且把子节点距离终点的代价找到。
%函数的输出量中costs表示拓展的子节点到起始点的代价，heuristics表示拓展出来的点到终止点的距离大约是多少，posinds表示拓展出来的子节点
%拓展方向分别为左、右、前、后、上、下， 上前、上后、下前，下后， 左前、左后、右前、右后， 上左、上右、下左、下右
function [cost,heuristic,posinds] = findFValue(Corner_obstacles,n,posind,costsofar,field,goalind)
    [currentpos(1) currentpos(2) currentpos(3)] = ind2sub([n n n],posind);   %将要进行拓展的点（也就是父节点）的索引值拓展成坐标值
    [goalpos(1) goalpos(2) goalpos(3)] = ind2sub([n n n],goalind);        %将终止点的索引值拓展成坐标值
    cost = Inf*ones(18,1); heuristic = Inf*ones(18,1); pos = ones(18,3); %将矩阵cost和heuristic初始化为4x1的无穷大值的矩阵，pos初始化为4x2的值为1的矩阵
    
    % 拓展方向一，左
    newx = currentpos(1) - 1; newy = currentpos(2); newz =  currentpos(3);
    if newx > 0
      pos(1,:) = [newx newy newz];
%       heuristic(1) = abs(goalpos(1)-newx) + abs(goalpos(2)-newy) + abs(goalpos(3)-newz); % 曼哈顿距离公式
      heuristic(1) = sqrt((goalpos(1)-newx)^2 +(goalpos(2)-newy)^2+(goalpos(3)-newz)^2);  %欧几里得距离
      cost(1) = costsofar + field(newx,newy,newz);
    end
 
    % 拓展方向二，右
    newx = currentpos(1) + 1; newy = currentpos(2); newz =  currentpos(3);
    if newx <= n
      pos(2,:) = [newx newy newz];
%       heuristic(2) = abs(goalpos(1)-newx) + abs(goalpos(2)-newy) + abs(goalpos(3)-newz); % 曼哈顿距离公式
      heuristic(2) = sqrt((goalpos(1)-newx)^2 +(goalpos(2)-newy)^2+(goalpos(3)-newz)^2);  %欧几里得距离
      cost(2) = costsofar + field(newx,newy,newz);
    end
 
    % 拓展方向三，前
    newx = currentpos(1); newy = currentpos(2)+1; newz =  currentpos(3); %向前拓展，地图上y+1，数组中y
    if newy <= n
      pos(3,:) = [newx newy newz];
%       heuristic(3) = abs(goalpos(1)-newx) + abs(goalpos(2)-newy) + abs(goalpos(3)-newz); % 曼哈顿距离公式
      heuristic(3) = sqrt((goalpos(1)-newx)^2 +(goalpos(2)-newy)^2+(goalpos(3)-newz)^2);  %欧几里得距离
      cost(3) = costsofar + field(newx,newy,newz);
    end
 
    % 拓展方向四，后
    newx = currentpos(1); newy = currentpos(2)-1; newz =  currentpos(3);
    if newy > 0
      pos(4,:) = [newx newy newz];
%       heuristic(4) = abs(goalpos(1)-newx) + abs(goalpos(2)-newy) + abs(goalpos(3)-newz); % 曼哈顿距离公式
      heuristic(4) = sqrt((goalpos(1)-newx)^2 +(goalpos(2)-newy)^2+(goalpos(3)-newz)^2);  %欧几里得距离 
      cost(4) = costsofar + field(newx,newy,newz);
    end
    
    % 拓展方向五，上
    newx = currentpos(1); newy = currentpos(2); newz =  currentpos(3)+1;
    if newz <= n
      pos(5,:) = [newx newy newz];
%       heuristic(5) = abs(goalpos(1)-newx) + abs(goalpos(2)-newy) + abs(goalpos(3)-newz); % 曼哈顿距离公式
      heuristic(5) = sqrt((goalpos(1)-newx)^2 +(goalpos(2)-newy)^2+(goalpos(3)-newz)^2);  %欧几里得距离
      cost(5) = costsofar + field(newx,newy,newz);
    end
    
     % 拓展方向六，下
    newx = currentpos(1); newy = currentpos(2); newz =  currentpos(3)-1;
    if newz > 0
      pos(6,:) = [newx newy newz];
%       heuristic(6) = abs(goalpos(1)-newx) + abs(goalpos(2)-newy) + abs(goalpos(3)-newz); % 曼哈顿距离公式
      heuristic(6) = sqrt((goalpos(1)-newx)^2 +(goalpos(2)-newy)^2+(goalpos(3)-newz)^2);  %欧几里得距离
      cost(6) = costsofar + field(newx,newy,newz);
    end
    
    % 拓展方向七，上前
    newx = currentpos(1); newy = currentpos(2)+1; newz =  currentpos(3)+1;
    if (newy <= n)&&(newz <= n)
      pos(7,:) = [newx newy newz];
      heuristic(7) = sqrt((goalpos(1)-newx)^2 +(goalpos(2)-newy)^2+(goalpos(3)-newz)^2);  %欧几里得距离
      
      cost(7) = costsofar + sqrt(2)*field(newx,newy,newz); 
      if Corner_obstacles
         cost(7) =  cost(7) + field(newx,newy,newz-1)-1;   %代价为根号2的拓展节点，添加不可穿越四角障碍的规则,向上前方拓展时，判断新拓展点下方是否有障碍
      end
    end
    
    
    % 拓展方向八，上后
    newx = currentpos(1); newy = currentpos(2)-1; newz =  currentpos(3)+1;
    if (newy > 0)&&(newz <= n)
      pos(8,:) = [newx newy newz];
      heuristic(8) = sqrt((goalpos(1)-newx)^2 +(goalpos(2)-newy)^2+(goalpos(3)-newz)^2);  %欧几里得距离
      cost(8) = costsofar + sqrt(2)*field(newx,newy,newz); 
      if Corner_obstacles
           cost(8) = cost(8)+ field(newx,newy,newz-1)-1;   %向上后方拓展时，判断新拓展点下方是否有障碍
      end
    end
    
    % 拓展方向九，下前
    newx = currentpos(1); newy = currentpos(2)+1; newz =  currentpos(3)-1;
    if (newy <= n)&&(newz > 0)
      pos(9,:) = [newx newy newz];
      heuristic(9) = sqrt((goalpos(1)-newx)^2 +(goalpos(2)-newy)^2+(goalpos(3)-newz)^2);  %欧几里得距离
      
      cost(9) = costsofar + sqrt(2)*field(newx,newy,newz); 
      if Corner_obstacles
           cost(9) = cost(9)+ field(newx,newy-1,newz)-1;   %向下前方拓展时，判断新拓展点后方是否有障碍
      end
    end
    
    % 拓展方向十，下后
    newx = currentpos(1); newy = currentpos(2)-1; newz =  currentpos(3)-1;
    if (newy > 0)&&(newz > 0)
      pos(10,:) = [newx newy newz];
      heuristic(10) = sqrt((goalpos(1)-newx)^2 +(goalpos(2)-newy)^2+(goalpos(3)-newz)^2);  %欧几里得距离
      
      cost(10) = costsofar + sqrt(2)*field(newx,newy,newz); 
      if Corner_obstacles
           cost(10) = cost(10)+ field(newx,newy+1,newz)-1;    %向下后方拓展时，判断新拓展点前方是否有障碍
      end
    end
 
    % 拓展方向十一，左前
    newx = currentpos(1)-1; newy = currentpos(2)+1; newz =  currentpos(3);
    if (newx > 0)&&(newy <= n)
      pos(11,:) = [newx newy newz];
      heuristic(11) = sqrt((goalpos(1)-newx)^2 +(goalpos(2)-newy)^2+(goalpos(3)-newz)^2);  %欧几里得距离
      
      cost(11) = costsofar + sqrt(2)*field(newx,newy,newz); 
      if Corner_obstacles
           cost(11) = cost(11)+ field(newx+1,newy,newz)+field(newx,newy-1,newz)-2;    %向左前方拓展时，判断新拓展点右方及后方是否有障碍
      end
    end
    
    % 拓展方向十二，左后
    newx = currentpos(1)-1; newy = currentpos(2)-1; newz =  currentpos(3);
    if (newx > 0)&&(newy > 0)
      pos(12,:) = [newx newy newz];
      heuristic(12) = sqrt((goalpos(1)-newx)^2 +(goalpos(2)-newy)^2+(goalpos(3)-newz)^2);  %欧几里得距离
      
      cost(12) = costsofar + sqrt(2)*field(newx,newy,newz); 
      if Corner_obstacles
           cost(12) = cost(12)+ field(newx+1,newy,newz)+field(newx,newy+1,newz)-2;    %向左后方拓展时，判断新拓展点右方及前方是否有障碍
      end
    end
    
    % 拓展方向十三，右前 
    newx = currentpos(1)+1; newy = currentpos(2)+1; newz =  currentpos(3);
    if (newx <= n)&&(newy <= n)
      pos(13,:) = [newx newy newz];
      heuristic(13) = sqrt((goalpos(1)-newx)^2 +(goalpos(2)-newy)^2+(goalpos(3)-newz)^2);  %欧几里得距离
      
      cost(13) = costsofar + sqrt(2)*field(newx,newy,newz); 
      if Corner_obstacles
           cost(13) = cost(13)+field(newx-1,newy,newz)+field(newx,newy-1,newz)-2;    %向右前方拓展时，判断新拓展点左方及后方是否有障碍
      end
    end
    
    % 拓展方向十四，右后
    newx = currentpos(1)+1; newy = currentpos(2)-1; newz =  currentpos(3);
    if (newx <= n)&&(newy > 0)
      pos(14,:) = [newx newy newz];
      heuristic(14) = sqrt((goalpos(1)-newx)^2 +(goalpos(2)-newy)^2+(goalpos(3)-newz)^2);  %欧几里得距离
      
      cost(14) = costsofar + sqrt(2)*field(newx,newy,newz); 
      if Corner_obstacles
           cost(14) = cost(14)+ field(newx-1,newy,newz)+field(newx,newy+1,newz)-2;    %向右后方拓展时，判断新拓展点左方及前方是否有障碍
      end
    end
    
    % 拓展方向十五，上左
    newx = currentpos(1)-1; newy = currentpos(2); newz =  currentpos(3)+1;
    if (newx > 0)&&(newz <= n)
      pos(15,:) = [newx newy newz];
      heuristic(15) = sqrt((goalpos(1)-newx)^2 +(goalpos(2)-newy)^2+(goalpos(3)-newz)^2);  %欧几里得距离
      
      cost(15) = costsofar + sqrt(2)*field(newx,newy,newz); 
      if Corner_obstacles
           cost(15) = cost(15)+  field(newx,newy,newz-1)-1;    %向上左方拓展时，判断新拓展点下方是否有障碍
      end
    end
    
    % 拓展方向十六，上右
    newx = currentpos(1)+1; newy = currentpos(2); newz =  currentpos(3)+1;
    if (newx <= n)&&(newz <= n)
      pos(16,:) = [newx newy newz];
      heuristic(16) = sqrt((goalpos(1)-newx)^2 +(goalpos(2)-newy)^2+(goalpos(3)-newz)^2);  %欧几里得距离
      
      cost(16) = costsofar + sqrt(2)*field(newx,newy,newz); 
      if Corner_obstacles
           cost(16) = cost(16)+ field(newx,newy,newz-1)-1;    %向上右方拓展时，判断新拓展点下方是否有障碍
      end
    end
    
      % 拓展方向十七，下左
    newx = currentpos(1)-1; newy = currentpos(2); newz =  currentpos(3)-1;
    if (newx > 0)&&(newz > 0)
      pos(17,:) = [newx newy newz];
      heuristic(17) = sqrt((goalpos(1)-newx)^2 +(goalpos(2)-newy)^2+(goalpos(3)-newz)^2);  %欧几里得距离
      
      cost(17) = costsofar + sqrt(2)*field(newx,newy,newz); 
      if Corner_obstacles
           cost(17) = cost(17)+field(newx+1,newy,newz)-1;    %向下左方拓展时，判断新拓展点右方是否有障碍
      end
    end
    
      % 拓展方向十八，下右
    newx = currentpos(1)+1; newy = currentpos(2); newz =  currentpos(3)-1;
    if (newx <= n)&&(newz > 0)
      pos(18,:) = [newx newy newz];
      heuristic(18) = sqrt((goalpos(1)-newx)^2 +(goalpos(2)-newy)^2+(goalpos(3)-newz)^2);  %欧几里得距离
      
      cost(18) = costsofar + sqrt(2)*field(newx,newy,newz); 
      if Corner_obstacles
           cost(18) = cost(18)+ field(newx-1,newy,newz)-1;    %向下右方拓展时，判断新拓展点左方是否有障碍
      end
    end
    
     posinds = sub2ind([n n n],pos(:,1),pos(:,2),pos(:,3)); % 将拓展出来的所以子节点的坐标值转换为索引值，储存在数组posinds里
end