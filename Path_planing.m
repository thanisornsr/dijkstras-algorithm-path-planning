clc; clear;

%% Give start position, goal position, and path to map in this section
%this are in positions
start_x = 4.5;
start_y = 4.5;

goal_x = 2.6;
goal_y = 5.5;

simulation_map = 'mysimulation-map.txt';


%% Dealing with map.txt and make map
map_text = readtable(simulation_map);
size_text  = size(map_text);
text_array = zeros(size_text);

for i = 1:size_text(1)
    temp = zeros(0);
    for j = 1:size_text(2)
        if j == 4
            temp_j = map_text{i,j};
            temp_j = temp_j{1};
            temp_j = strsplit(temp_j,';');
            temp_j = str2double(temp_j(1));
            temp(j) = temp_j;
        else
            temp(j) = map_text{i,j};
        end
    end
    text_array(i,:) = temp;
end

x0 = text_array(:,1);
y0 = text_array(:,2);
x1 = text_array(:,3);
y1 = text_array(:,4);

max_x = max(max(x0,x1));
min_x = min(min(x0,x1));
max_y = max(max(y0,y1));
min_y = min(min(y0,y1));
map_size_x = max_x - min_x;
map_size_y = max_y - min_y;
gridSize = 0.2;


h_map = round(map_size_y/gridSize) + 1;
w_map = round(map_size_x/gridSize) + 1;

%map position to array
map_pos = zeros(h_map,w_map,2);
pos_x = 0;
for i = 1:h_map
    pos_y = 0;
    for j = 1:h_map
        map_pos(j,i,:) = [pos_x,pos_y];
        pos_y = pos_y+gridSize;
    end
    pos_x = pos_x + gridSize;
end
n_nodes = h_map*w_map;

wall_color = 250;
start_color = 100;
goal_color = 200;
path_color = 130;
non_visited_color = 50; 

map = zeros(h_map,w_map);

%convert x0 x1 y0 y1 to grid
x0_g = x0/gridSize + 1;
x1_g = x1/gridSize + 1;
y0_g = y0/gridSize + 1;
y1_g = y1/gridSize + 1;

wall_map = map;

for i = 1:length(x0)
    temp_x0 = x0_g(i);
    temp_y0 = y0_g(i);
    temp_x1 = x1_g(i);
    temp_y1 = y1_g(i);
    if temp_x1 > temp_x0
        temp_y = temp_y0;
        for j = temp_x0:temp_x1
            wall_map(temp_y,j) = 1;
        end
    elseif temp_y1 > temp_y0
        temp_x = temp_x0;
        for j = temp_y0:temp_y1
            wall_map(j,temp_x) = 1;
        end
    else
       disp('Something wrong!') 
    end
end
map(wall_map == 1) = wall_color;


%% Path finding


% here

node_names = [1:n_nodes];
node_names = reshape(node_names,h_map,w_map);

% Store distance
nodes = Inf(h_map,w_map);

unexplored_nodes = node_names;
parents = NaN(h_map,w_map);
startX = round(start_x/gridSize) + 1 ;
startY = round(start_y/gridSize) + 1 ;
start_node = node_names(startY,startX);
nodes(startY, startX) = 0;

goalX = round(goal_x/gridSize) + 1 ;
goalY = round(goal_y/gridSize) + 1 ;
goal_node = node_names(goalY,goalX);


if map(startY,startX) == wall_color
    error('The start position is in the wall')

elseif map(goalY,goalX) == wall_color
    error('The destination position is in the wall')

else
    disp('We are good to go!')
 
end


while ~all(all(unexplored_nodes == Inf))
   % get current node >> in the unexploredSet and smallest distance
   current_node = NaN;
   current_node_dis = Inf;
   current_x = NaN;
   current_y = NaN;
   temp_node = NaN;
   temp_node_dis_min = Inf;

   for i = 1:n_nodes
       if any(any(unexplored_nodes == i))
           [temp_node_y, temp_node_x] = find(node_names == i);
           temp_node_dis = nodes(temp_node_y,temp_node_x);
           if temp_node_dis<temp_node_dis_min
               temp_node = i;
               temp_node_dis_min = temp_node_dis;
           end    
       end
   end
   
   current_node = temp_node;
   current_node_dis = temp_node_dis_min;

   [current_y,current_x] = find(node_names == current_node);
   unexplored_nodes(current_y,current_x) = Inf;
   % if current Node is goal >> break;
   if current_node == goal_node
       break;
   end
   
   %gett all neightbors in unexploredSet and not wall
   
   neighborName = NaN(1,1);
   neighborDistance = Inf(1,1);
   index_neighbor = 1;
   
   % Check all 4 direction x+ x- y+ y-
   %%x+
   temp_n_x = current_x + 1;
   temp_n_y = current_y;
   
   
   % Check if it is in map
   if temp_n_x > 0 && temp_n_y > 0 && temp_n_x <= w_map && temp_n_y <= h_map 
       temp_n_dis = nodes(temp_n_y,temp_n_x);
       temp_n_name = node_names(temp_n_y,temp_n_x);
       % Check if it is in unexplored and not wall
       if any(any(unexplored_nodes == temp_n_name)) && map(temp_n_y,temp_n_x) ~= wall_color
           neighborName(index_neighbor) = temp_n_name;
           neighborDistance(index_neighbor) = nodes(temp_n_y,temp_n_x);
           index_neighbor = index_neighbor + 1;
       end
   end
   
   %%x-
   temp_n_x = current_x - 1;
   temp_n_y = current_y;
   
   % Check if it is in map
   if temp_n_x > 0 && temp_n_y > 0 && temp_n_x <= w_map && temp_n_y <= h_map 
       temp_n_dis = nodes(temp_n_y,temp_n_x);
       temp_n_name = node_names(temp_n_y,temp_n_x);
       % Check if it is in unexplored and not wall
       if any(any(unexplored_nodes == temp_n_name)) && map(temp_n_y,temp_n_x) ~= wall_color
           neighborName(index_neighbor) = temp_n_name;
           neighborDistance(index_neighbor) = nodes(temp_n_y,temp_n_x);
           index_neighbor = index_neighbor + 1;
       end
   end
   
   %%y+
   temp_n_x = current_x;
   temp_n_y = current_y+1;
   
   % Check if it is in map
   if temp_n_x > 0 && temp_n_y > 0 && temp_n_x <= w_map && temp_n_y <= h_map 
       temp_n_dis = nodes(temp_n_y,temp_n_x);
       temp_n_name = node_names(temp_n_y,temp_n_x);
       % Check if it is in unexplored and not wall
       if any(any(unexplored_nodes == temp_n_name)) && map(temp_n_y,temp_n_x) ~= wall_color
           neighborName(index_neighbor) = temp_n_name;
           neighborDistance(index_neighbor) = nodes(temp_n_y,temp_n_x);
           index_neighbor = index_neighbor + 1;
       end
   end
   
   %%y-
   temp_n_x = current_x;
   temp_n_y = current_y-1;
   
   % Check if it is in map
   if temp_n_x > 0 && temp_n_y > 0 && temp_n_x <= w_map && temp_n_y <= h_map 
       temp_n_dis = nodes(temp_n_y,temp_n_x);
       temp_n_name = node_names(temp_n_y,temp_n_x);
       % Check if it is in unexplored and not wall
       if any(any(unexplored_nodes == temp_n_name)) && map(temp_n_y,temp_n_x) ~= wall_color
           neighborName(index_neighbor) = temp_n_name;
           neighborDistance(index_neighbor) = nodes(temp_n_y,temp_n_x);
           index_neighbor = index_neighbor + 1;
       end
   end
   
   if isnan(neighborName)
       disp('No Neighbor')
   else
       for i = 1:length(neighborName)
           temp_n_name = neighborName(i);
           temp_n_dis = neighborDistance(i);
           temp_new_distance = current_node_dis + 1; 

           if temp_new_distance < temp_n_dis
               [temp_n_y,temp_n_x] = find(node_names == temp_n_name);
               nodes(temp_n_y,temp_n_x) = temp_new_distance;
               parents(temp_n_y,temp_n_x) = current_node;
           end
       end
   end
    
end


% to here
%% Plot the path

temp_perental = parents(goalY,goalX);

%non-visited plot to 
map(unexplored_nodes == Inf) = non_visited_color;
map(startY,startX) = start_color;
map(goalY,goalX) = goal_color;
map(wall_map == 1) = wall_color;

while temp_perental ~= start_node
   [temp_y,temp_x] = find(node_names == temp_perental);
   map(temp_y,temp_x) = path_color;
   temp_perental = parents(temp_y,temp_x);
end

map = flip(map);
image(map)

