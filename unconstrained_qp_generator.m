function path = unconstrained_qp_generator(pose_fixed,pose_transit,params)

%% set parameters here

    t0 = params.t0;
    T = params.T;
    kr = params.kr;
    frame  =params.frame;
    num_st = params.num_st;
    
    % function check
    if T>=10
       warning('Segment duration cannot exceed 10 s!'); 
    end
          

%%  matrix initial

    % induced params
    num_seg = size(pose_fixed,1)+size(pose_transit,2)-1;
    num_point = num_seg+1;
    segments0 = zeros(num_seg,kr*2);
    for is = 1:size(pose_fixed,1)-1
       segments0(is,1:kr) = pose_fixed(is,:);
       segments0(is,kr+1:2*kr) = pose_fixed(is+1,:);
    end
    
    N = 2*kr -1;
    dim = 1;
    ts = t0:T:T*num_seg;

    % Q
    Q0 = zeros(N+1,N+1);
    Ad0 = zeros((num_st*2)*num_seg*dim,(N+1)*num_seg);   % mellinger method
    Ad1 = zeros((num_st*2)*num_seg*dim,(N+1)*num_seg);    % unconstarined QP method
    f = zeros((N+1)*num_seg,1);
    poly0 = zeros(num_seg*(N+1),1);

    for i=kr+1:N+1
        for j = kr+1:N+1
            Q0(i,j) = ((i-1)*(i-2)*(i-3)*(i-4)*(j-1)*(j-2)*(j-3)*(j-4)/(i+j-9))*T^(i+j-9);
        end
    end
    
    %find constraint matrix Ad
    for is = 1:num_seg
        for k=0:3
            for i=k:N
                Ad1(1+k +(is-1)*2*kr,i+1+(is-1)*(N+1)) = (factorial(i)/factorial(i-k))*ts(is)^(i-k);    % start states
                Ad1(1+k +(is-0.5)*2*kr,i+1+(is-1)*(N+1)) = (factorial(i)/factorial(i-k))*ts(is+1)^(i-k);    % end states
            end
        end
        segments1(1,(N+1)*(is-1)+1:(N+1)*is) = segments0(is,1:end);  % just for x now
    end
    
    %% unconstained method
    Q2 = kron(eye(num_seg),Q0); 
    Ad2 = Ad1;
    segments2 = segments1;
    path = zeros(frame*num_seg+1,2);
    
    % find d
    for ip = 1:size(pose_fixed,1)
       df0((ip-1)*kr +1:ip*kr,1) = pose_fixed(ip,:); 
    end 
    df = [df0;pose_transit'];
    
    % selection matrix C
    num_trans = size(pose_transit,2);
    Ct = zeros(num_seg*2*kr,(num_seg+1)*kr);
    Ct(1:kr,1:kr) = eye(4);
    Ct(end-kr+1:end,kr+1:2*kr) = eye(4);
    
    Ct_f0 = [1 zeros(1,kr-1) 1 zeros(1,kr-1)]';
    Ct_p0 = [zeros(1,3);eye(3);zeros(1,3);eye(3)];
    for it = 1:num_trans
        Ct((it-1)*2*kr +kr+1:(it-1)*2*kr +kr*3,it-1 +kr*2+1) = Ct_f0;
        Ct((it-1)*2*kr +kr+1:(it-1)*2*kr +kr*3,2*kr+num_trans+ (it-1)*(kr-1)+1:2*kr+num_trans+ it*(kr-1)) = Ct_p0;
    end
    
    % find R
    length_df = size(df,1);
    length_dp = num_trans*(kr-1);
    
    R = Ct.'*inv(Ad2).'*Q2*inv(Ad2)*Ct;
    R_pp = R(end-length_dp+1:end,end-length_dp+1:end);
    R_fp = R(1:length_df,end-length_dp+1:end);


    % solver
    dp_star = -inv(R_pp)*R_fp'*df;
    dp = conj(dp_star);
    d_st = Ct *[df;dp];
    poly2 = inv(Ad2) * d_st;
    
    % calculate trajectory
    for is = 1:num_seg
        for index=1+(is-1)*frame:frame+(is-1)*frame
            t = ts(is) +(index-1-(is-1)*frame)*T/frame;
            for i=0:N   % x path
                path(index,1) = t;
                path(index,2) = poly2(i+1+(is-1)* (N+1))*t^(i) + path(index,2);  %x
            end
        end
    end
    path(frame*num_seg+1,:) = [t+T/frame pose_fixed(end,1)];
    
    %% plot tools
%     plot(path(:,1),path(:,2));
%     grid on;

    