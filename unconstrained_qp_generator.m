clear all;

%% set parameters here
    t0 = 0;
    T = 2;   % segment duration
    kr = 4; % derivative order
    frame = 100;    % number of frames
    num_st = 4; % number of constrained states
            
    % [states_start states_end]
    segments0 = [0 0 0 0 4 3 0 0;
                  4 3 0 0 11 0 0 0
                  11 0 0 0 22 0 0 0
                  ];
            
    num_seg = size(segments0,1);
    dim = size(segments0,2)/(num_st*2);
    ts = t0:T:T*num_seg;

%%  matrix initial

    % induced params
    N = 2*kr -1;
    path = zeros(frame*num_seg+1,4);

    % Q
    Q0 = zeros(N+1,N+1);
    Ad0 = zeros((num_st*2)*num_seg*dim,(N+1)*num_seg);   % mellinger method
    Ad1 = zeros((num_st*2)*num_seg*dim,(N+1)*num_seg);    % unconstarined QP method
    f = zeros((N+1)*num_seg,1);
    poly0 = zeros(num_seg*(N+1),1);
    %poly = zeros(frame*num_seg+1,dim+1);

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
    
    %% mellinger method
%     Ad1 = Ad0; 
%     Q1 = kron(eye(num_seg),Q0); 
%     poly1 = poly0; segments1 =
%     zeros(1,num_st*2*dim*num_seg); path1 = zeros(frame*num_seg+1,2);
% 
%     for is = 1:num_seg
%         for k=0:3
%             for i=k:N
%                 Ad1(1+k +(is-1)*2*kr,i+1+(is-1)*(N+1)) =
%                 (factorial(i)/factorial(i-k))*ts(is)^(i-k);    % start
%                 states Ad1(1+k +(is-0.5)*2*kr,i+1+(is-1)*(N+1)) =
%                 (factorial(i)/factorial(i-k))*ts(is+1)^(i-k);    % end
%                 states
%             end
%         end
%                 segments1(1,(N+1)*(is-1)+1:(N+1)*is) =
%                 segments0(is,1:end);  % just for x now
% 
%     end
%     
%     poly1 = quadprog(Q1,f,[],[],Ad1,segments1');
%     
%     for is = 1:num_seg
%         for index=1+(is-1)*frame:frame+(is-1)*frame
%             t = ts(is) +(index-1-(is-1)*frame)*T/frame; for i=0:N   % x
%             path
%                 path1(index,1) = t; path1(index,2) = poly1(i+1+(is-1)*
%                 (N+1))*t^(i) + path1(index,2);  %x
%             end
%         end
%     end path1(frame*num_seg+1,:) = [t+T/frame segments1(1,end-num_st+1)];
    
    %% unconstained method
    Q2 = kron(eye(num_seg),Q0); 
    Ad2 = Ad1;
    segments2 = segments1;
    path2 = zeros(frame*num_seg+1,2);
    
    % find d
    df = [0 0 0 0 22 0 0 0]';
    
    % selection matrix C
    Ct = zeros(num_seg*2*kr,(num_seg+1)*kr);
    Ct(1:4,1:4) = eye(4);
    Ct(end-3:end,5:8) = eye(4);
    
    Ct_0 = [eye(4);eye(4)];
    Ct(5:end-4,9:end) = kron(eye(2),Ct_0);
    
%     Ct(13:16,5:8) =eye(4);
    
    % find R
    R = Ct.'*inv(Ad2).'*Q2*inv(Ad2)*Ct;
    R_pp = R(9:end,9:end);
    R_fp = R(1:8,9:end);


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
                path2(index,1) = t;
                path2(index,2) = poly2(i+1+(is-1)* (N+1))*t^(i) + path2(index,2);  %x
            end
        end
    end
    path2(frame*num_seg+1,:) = [t+T/frame segments1(1,end-num_st+1)];
    
    %% plot tools
    
%     plot(path1(:,1),path1(:,2));
%     hold on;
    plot(path2(:,1),path2(:,2));
    grid on;

    