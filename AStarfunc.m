function [ path] = AStarfunc()
%AStar path planner
%path is returned

global state; 
global destination;

%Converting to grid points
init = translator([state(1)+100, state(2)+100]); % +100 to account for initial drift
ngl = translator(destination);

%Possible moves at each node
delta = [-1,  0;  % go up
          0, -1; % go left
          1,  0;  % go down
          0,  1;  % go right
         -1,  1; % go diag up left
         -1, -1; % go diag up right
          1,  1; % go diag down right 
          1, -1];% go diag down left

%Initialize
closed = init;
node = init;
g = 0.0;
path = init;
f = g + heuristic(node, ngl);
Fringe = struct('f',f,'g',g,'node',node,'path',path);

found = false;% flag set when search is complete
resign = false;% flag set if no more nodes to expand
    
while(~found && ~resign)
     
    if (size(Fringe,2) == 0)
        resign = true;
        display 'Failed to find path';
        continue;
    end
    
    [~,I]= sort([Fringe(:).f],'descend');
    Fringe = Fringe(I);        
    next = Fringe(end);
    Fringe = Fringe(1:end-1);            
    
    node = next.node;    
    g = next.g;
    path = next.path;
    
    if (isdestination(node, ngl))
        found = true;
        display 'Path obtained';
        continue;
    end
    
    for i = 1:size(delta,1)
        newnode = node + delta(i,:);
        if (ismember(newnode,closed,'rows') == 0) && (clsnchk(node,newnode) == 0)
            newg = g + 1;% + 10*floor((i-1)/4); uncomment to penalize turns
            newf = newg + heuristic(newnode,ngl);
            newpath = [path;newnode];
            Fringe(end+1) = struct('f',newf,'g',newg,'node',newnode,'path',newpath);                               
            closed = [closed;newnode];                
        end
    end
end

if(found)
    path = rtranslator(path);
end





