clear all;

% rosinit;

%% point cloud
sub_map = rossubscriber('/mock_map');
pointcd = receive(sub_map,10);
map_full = readXYZ(pointcd);

z_select = [3.5 3.5];
map_select = map_full(map_full(:,3)<=(z_select(2))...
    & map_full(:,3)>=(z_select(1)),:); %#ok<NODEF> % x-y plane for a given z coordinate and tolerance


% scatter3(map_select(:,1),map_select(:,2),map_select(:,3));

%% read vias
sub_vias = rossubscriber('/quad_marker');
vias = receive(sub_vias,5);

%%
% rosshutdown;