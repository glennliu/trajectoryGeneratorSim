clear all;

sub_map = rossubscriber('/mock_map');
pointcd = receive(sub_map,10);
map_full = readXYZ(pointcd);

z_select = [3.5 3.5];
sliceZ = map_full(map_full(:,3)<=(z_select(2))...
    & map_full(:,3)>=(z_select(1)),:); %#ok<NODEF> % x-y plane for a given z coordinate and tolerance


map_select = sliceZ;
scatter3(map_select(:,1),map_select(:,2),map_select(:,3));


