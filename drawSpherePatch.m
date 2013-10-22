function h_p = drawSpherePatch(R,t,dim,varargin)
    %
    % h_p = drawSpherePatch(R, t, dim,...)
    %
    % R,t are orientation and center of the sphere
    % dim is 1 dimensional vector giving radius [r]
    % possible additional parameters are:
    %       'FaceColor'  default: [1;1;1]
    %       'FaceAlpha'  default: 1
    %       'LineWidth'  default: 0.5
    %       'EdgeColor'  default: [0;0;0]
    %       'EdgeAlpha'  default: 1
    %
    % can accept existing handle to sphere patch using 'Handle'
    %
    % h_p is a struct containing handles to the patch objects
    %       h_p.h_sides
    
    % Walk through varargin
    for i=1:2:(nargin-3)
        if strcmp(varargin{i},'FaceColor')
            fc = varargin{i+1};
        elseif strcmp(varargin{i},'FaceAlpha')
            fa = varargin{i+1};
        elseif strcmp(varargin{i},'LineWidth')
            lw = varargin{i+1};
        elseif strcmp(varargin{i},'EdgeColor')
            ec = varargin{i+1};
        elseif strcmp(varargin{i},'EdgeAlpha')
            ea = varargin{i+1};
        elseif strcmp(varargin{i},'Handle')
            h_p = varargin{i+1};
        else
            error(['Parameter not recognized: ' varargin{i}]);
        end
    end
    % Set defaults if not already established
    if ~exist('fc','var')
        fc = [1;1;1];
    end
    if ~exist('fa','var')
        fa = 1;
    end
    if ~exist('lw','var')
        lw = 0.5;
    end
    if ~exist('ec','var')
        ec = [0;0;0];
    end
    if ~exist('ea','var')
        ea = 1;
    end
    
    r = dim(1);
    N = 20;
    
    theta = 0:pi/N:2*pi;
    phi = 0:pi/N:pi;
    
    c_th = cos(theta);
    s_th = sin(theta);
    c_phi = cos(phi);
    s_phi = sin(phi);
    
    x1 = r*c_th(1:2*N);
    x2 = r*c_th(2:2*N+1);
    y1 = r*s_th(1:2*N);
    y2 = r*s_th(2:2*N+1);
    
    faces = zeros(3,8*N*N);
    for i=1:N
        
        z1 = r*c_phi(i)*ones(1,2*N);
        z2 = r*c_phi(i+1)*ones(1,2*N);
        faces(:,(i-1)*8*N+1:4:i*8*N) = [x1*s_phi(i);y1*s_phi(i);z1];
        faces(:,(i-1)*8*N+2:4:i*8*N) = [x2*s_phi(i);y2*s_phi(i);z1];
        faces(:,(i-1)*8*N+3:4:i*8*N) = [x2*s_phi(i+1);y2*s_phi(i+1);z2];
        faces(:,(i-1)*8*N+4:4:i*8*N) = [x1*s_phi(i+1);y1*s_phi(i+1);z2];
    end
    
    faces = t*ones(1,8*N*N) + R*faces;
    faces_X = reshape(faces(1,:),[4 2*N*N]);
    faces_Y = reshape(faces(2,:),[4 2*N*N]);
    faces_Z = reshape(faces(3,:),[4 2*N*N]);
    if exist('h_p','var')
        set(h_p,'XData',faces_X,'YData',faces_Y,'ZData',faces_Z);
    else
        h_p = patch(faces_X,faces_Y,faces_Z,1, ...
                            'FaceColor',fc, ...
                            'FaceAlpha',fa, ...
                            'LineWidth',lw, ...
                            'EdgeColor',ec, ...
                            'EdgeAlpha',ea);
    end
end
    