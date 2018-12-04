function s_des=minimum_snap_generator()   

%% set parameters here
    t0 = 0;
    T = 2;   % segment duration
    kr = 4; % derivative order
    frame = 100;    % number of frames
    
    % derivative constraint
    % Notice: set acce st without vel st doesn't perform well
    % [[x0;xt;vx0;vxt;ax0;axt];[y0;yt;vy0;vyt;ay0;ayt];[z0;zt;vz0;vzt;az0;azt]]
    segments = [-1 0 0 0 0 0 0 1 0 0 0 0 1 0 0 0 0 0
                0 2 0 0 0 0 1 0 0 0 0 0 0 -1 0 0 0 0
                2 3 0 0 0 0 0 -2 0 0 0 0 -1 2 0 0 0 0];
    num_seg = size(segments,1);
    dim = size(segments,2)/6;
    ts = t0:T:T*num_seg;

    % c_select = [p0 pt v0 vt a0 at]
    % 1 for choose, 0 for none
    % the same selection matrix for x,y,z dim
    c_select = [1 1 1 1 1 1]
    

    %% calculation
    % induced params
    num_st = size(segments,2)/3;  %number of constraint
    N = 2*kr -1;
    segments_ = zeros(1,6*num_seg*dim);

    % build matrix
    Q0 = zeros(N+1,N+1);
    Q1 = zeros(N+1,N+1);
    Ad = zeros(num_st*num_seg,(N+1)*num_seg);
    Ac = zeros(num_st,(N+1)*num_seg);
    f = zeros((N+1)*dim*num_seg,1);
    s_des = zeros(frame*num_seg+1,dim+1);

    for i=kr+1:N+1
        for j = kr+1:N+1
            Q1(i,j) = ((i-1)*(i-2)*(i-3)*(i-4)*(j-1)*(j-2)*(j-3)*(j-4)/(i+j-9))*T^(i+j-9);
        end
    end
    
    Q2 = kron(eye(num_seg),Q1);

%   derivative constraint and continuity constraint
    for is = 1:num_seg
        for k=0:2
            for i=k:N
                Ad(k*2+1+(is-1)*6,i+1+(is-1)*(N+1)) = (factorial(i)/factorial(i-k))*ts(is)^(i-k);
                Ad(k*2+2+(is-1)*6,i+1+(is-1)*(N+1)) = (factorial(i)/factorial(i-k))*ts(is+1)^(i-k);
                
                Ac(k*2+1,i+1+(is-1)*(N+1)) = (factorial(i)/factorial(i-k))*ts(is)^(i-k);
                Ac(k*2+2,i+1+(is-1)*(N+1)) = (factorial(i)/factorial(i-k))*ts(is+1)^(i-k);                
            end
            
            if c_select(1,k*2+1) == 0
                Ad(k*2+1,:) = zeros(1,N+1);
            end
            
            if c_select(1,k*2+2) == 0
                Ad(k*2+2,:) = zeros(1,N+1);
            end
        end
    
        for i_dim = 1:dim
            segments_(1,1+((is-1)+(i_dim-1)*num_seg)*num_st:(is+num_seg*(i_dim-1))*num_st) = segments(is,1+(i_dim-1)*num_st:i_dim*num_st); %x
        end
    
    end
    
    
    
%   duplicate in dimension            
    A0 = zeros(num_st,N+1);
    Qtotal = kron(eye(dim),Q2);
    Ad_total = kron(eye(dim),Ad);
    Ac_total = [Ac Ac Ac];
    
    [x,fval,exitflag,output,lamda] = quadprog(Qtotal,f,[],[],Ad_total,segments_');
    
    % create output path
    disp('debug');
    
    for is = 1:num_seg
        for index=1+(is-1)*frame:frame+(is-1)*frame
            t = ts(is) +(index-1-(is-1)*frame)*T/frame;
            for i=0:N   % x path
                s_des(index,1) = t;
                s_des(index,2) = x(i+1+(is-1)* (N+1))*t^(i) + s_des(index,2);  %x
                s_des(index,3) = x(i+1+(is-1 + num_seg)* (N+1))*t^(i) + s_des(index,3);    %y
                s_des(index,4) = x(i+1+(is-1 + num_seg*2)* (N+1))*t^(i) + s_des(index,4);  %z
            end
        end
    end
    s_des(frame*num_seg+1,:) = [t+T/frame segments(end,2) segments(end,8) segments(end,14)];
        
    disp('break');
    
end