clear;clc;
% set customer
MaxSearchTime = 10000;
Metric = 15;

% init graph
A = imread('graph.png');
MaxRange = 1000;
imshow(A);
hold on;
B = A(:,:,1);
% imshow(B);
% choose start & goal point
% start = 500*rand(1,2,'double');
% goal = 500*rand(1,2,'double');
start = [4,490];
goal = [480,480];
plot(start(1),start(2),'b.');
plot(goal(1),goal(2),'y.');

node.x = start(1);
node.y = start(2);
node.father = 0; % no child is needed!!!

time=0;
scale=1; % tree scale
ends=false;
while (time < MaxSearchTime && ~ends)
    time=time+1;
    x=500*rand(1,'double');
    y=500*rand(1,'double');
    MinDistance=MaxRange;
    nearestNode=0;
    for i=1:scale  % find nearest node
        EulicDistance=abs(x-node(i).x)+abs(y-node(i).y); % get Eulic Distance
        if (EulicDistance < MinDistance) % update nearest node
            MinDistance = EulicDistance;
            nearestNode = i;
        end
    end
    % add new node
    if (nearestNode > 0)
%         if (B(ceil(node(nearestNode).x),ceil(node(nearestNode).y)) == 255 && B(floor(node(nearestNode).x),ceil(node(nearestNode).y)) == 255 && B(ceil(node(nearestNode).x),floor(node(nearestNode).y)) == 255 && B(floor(node(nearestNode).x),floor(node(nearestNode).y)) == 255)
        tempx = node(nearestNode).x + (x-node(nearestNode).x) /MinDistance *Metric;
        tempy = node(nearestNode).y + (y-node(nearestNode).y) /MinDistance *Metric;
        if (1<=tempx && tempx<=500 && tempy>=1 && tempy<=500 && B(ceil(tempy),ceil(tempx))==255 && B(ceil(tempy),floor(tempx))==255 && B(floor(tempy),ceil(tempx))==255 && B(floor(tempy),floor(tempx))==255)
            node(scale+1).x = tempx;
            node(scale+1).y = tempy;
            node(scale+1).father = nearestNode;
            plot(node(scale+1).x,node(scale+1).y,'m.')
            plot([node(scale+1).x,node(nearestNode).x],[node(scale+1).y,node(nearestNode).y],'b-')
            scale = scale+1;
            ends = hit_the_target(node(scale),goal,Metric);
        end
    end
end

% trajectory
if ends
    plot(node(scale).x,node(scale).y,'k.') % change the near goal point color
    i = scale;
    father = node(i).father;
    
    while (father > 0)
        plot(node(father).x,node(father).y,'k.') % father is in the traj., so change color
        plot([node(i).x,node(father).x],[node(i).y,node(father).y],'r-')
        i = father;
        father = node(i).father;        
    end
end