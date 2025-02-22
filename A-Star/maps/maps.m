 function [field, startposind, goalposind, costchart, fieldpointers] = maps(n)
    
    field=ones(n,n,n);  %地图上的0-n，按方格计数，一共n*n个方格
    field(:)=1;
%     stem3(field,'--.y');
    field_x = 0:n:n;
    field_y = 0:n:n;
    field_z = 0:n:n;
    stem3(field_x,field_y,field_z,'--.y');
    hold on;
   
i=11;   %要创建n个面积为1的障碍建筑物
% x,y,z想要改成随机可将数组成员改成rand*20
x=1:1:i; x=[2 4 3 5 7 10 13 14 16 9 8]; %创建建筑的x轴坐标分别为x-x+1；
y=1:1:i; y=[4 17 14 3 11 13 8 16 6 8 18];%创建建筑的y轴坐标分别为y-y+1；
z=1:1:i; z=[5 7 7 12 6 11 4 18 3 15 6];%创建建筑的高度分别为z；  
 
for a=1:i        % 创建第a个面积为1的建筑，将[x(a),y(a)]右上角的方块设为障碍物
 
    x1=x(a):0.01:x(a)+1; y1=y(a)+0.99:0.0001:y(a)+1; z1=z(a)-0.01:0.0001:z(a);
    stem3(x1,y1,z1,'.k');
    x2=x(a)+0.99:0.0001:x(a)+1; y2=y(a):0.01:y(a)+1; z2=z(a)-0.01:0.0001:z(a);
    stem3(x2,y2,z2,'.k');
    x3=x(a):0.0001:x(a)+0.01; y3=y(a):0.01:y(a)+1; z3=z(a)-0.01:0.0001:z(a);
    stem3(x3,y3,z3,'.k');
    x4=x(a):0.01:x(a)+1; y4=y(a):0.0001:y(a)+0.01; z4=z(a)-0.01:0.0001:z(a);
    stem3(x4,y4,z4,'.k');    
    
    for c=1:z(a)     
        field(x(a)+1,y(a)+1,c)=inf; 
    end
end
 
j=9;   %要创建n个面积为1的障碍建筑物
x=1:1:j; x=[14 17 11 3 6 10 17 7 0]; %创建建筑的x轴坐标分别为x-x+2；
y=1:1:j; y=[8 11 2 6 4 16 7 12 1];%创建建筑的y轴坐标分别为y-y+2；
z=1:1:j; z=[2 9 13 11 4 18 5 7 6];%创建建筑的高度分别为z； 
 
for b=1:j        % 创建第m个面积为4的建筑
 
    x1=x(b):0.01:x(b)+2; y1=y(b)+2:0.0001:y(b)+2.02; z1=z(b)-0.02:0.0001:z(b);
    stem3(x1,y1,z1,'.b');
    x2=x(b):0.0001:x(b)+0.02; y2=y(b):0.01:y(b)+2; z2=z(b)-0.02:0.0001:z(b);
    stem3(x2,y2,z2,'.b');
    x3=x(b)+2:0.0001:x(b)+2.02; y3=y(b):0.01:y(b)+2; z3=z(b)-0.02:0.0001:z(b);
    stem3(x3,y3,z3,'.b');
    x4=x(b):0.01:x(b)+2; y4=y(b):0.0001:y(b)+0.02; z4=z(b)-0.02:0.0001:z(b);
    stem3(x4,y4,z4,'.b');    
    
    for d=1:z(b)
        field(x(b)+1,y(b)+1,d)=inf; 
        field(x(b)+2,y(b)+1,d)=inf; 
        field(x(b)+1,y(b)+2,d)=inf;
        field(x(b)+2,y(b)+2,d)=inf;   
    end
end
    % 随机生成起始点和终止点
startposind = sub2ind([n,n,n],[ceil(n.*rand),ceil(n.*rand),ceil(n.*rand)]); %随机生成起始点的索引值，n在【1,20】
     while field(startposind(1),startposind(2),startposind(3))==inf
          startposind = sub2ind([n,n,n],[ceil(n.*rand),ceil(n.*rand),ceil(n.*rand)]);  %随机生成起始点
          if field(startposind(1),startposind(2),startposind(3))==1
              break;
          end
     end
     plot3(startposind(1)-0.5,startposind(2)-0.5,startposind(3)-0.5,'r.','markersize',20);
     startposind = sub2ind([n,n,n],startposind(1),startposind(2),startposind(3));
      
     goalposind = sub2ind([n,n,n],[ceil(n.*rand),ceil(n.*rand),ceil(n.*rand)]);  %随机生成终止点的索引值
      while field(goalposind(1),goalposind(2),goalposind(3))==inf
          goalposind = sub2ind([n,n,n],[ceil(n.*rand),ceil(n.*rand),ceil(n.*rand)]);  %随机生成起始点
          if field(goalposind(1),goalposind(2),goalposind(3))==1
              break;
          end
      end     
     plot3(goalposind(1)-0.5,goalposind(2)-0.5,goalposind(3)-0.5,'y.','markersize',20);
     goalposind = sub2ind([n,n,n],goalposind(1),goalposind(2),goalposind(3)); 
    field(startposind) = 0; field(goalposind) = 0;  %把矩阵中起始点和终止点处的值设为0
    
    costchart = NaN*ones(n,n,n);%生成一个nxn的矩阵costchart，每个元素都设为NaN。就是矩阵初始NaN无效数据
    costchart(startposind) = 0;%在矩阵costchart中将起始点位置处的值设为0
    
    % 生成元胞数组
    fieldpointers = cell(n,n,n);%生成元胞数组n*n
    fieldpointers{startposind} = 'S'; fieldpointers{goalposind} = 'G'; %将元胞数组的起始点的位置处设为 'S'，终止点处设为'G'
    fieldpointers(field==inf)={0};
    
   
 end
