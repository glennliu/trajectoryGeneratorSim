function path = create_3d_qp_trajecotry(segments_3d,transit_pose_3d,params)

path_x = unconstrained_qp_generator([segments_3d(1,1:4);segments_3d(end,1:4)],transit_pose_3d(:,1)',params);  % x
path_y = unconstrained_qp_generator([segments_3d(1,5:8);segments_3d(end,5:8)],transit_pose_3d(:,2)',params);  % y
path_z = unconstrained_qp_generator([segments_3d(1,9:12);segments_3d(end,9:12)],transit_pose_3d(:,3)',params);  % z

path =[path_x path_y(:,2) path_z(:,2)];

end