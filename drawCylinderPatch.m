function h_p = drawCylinderPatch(R,t,dim,varargin)
    %
    % h_p = drawCylinderPatch(R, t, dim,...)
    %
    % R,t are orientation and center of the cylinder
    % dim is 2 dimensional vector giving radius [r] and height [h]
    % possible additional parameters are:
    %       'FaceColor'  default: [1;1;1]
    %       'FaceAlpha'  default: 1
    %       'LineWidth'  default: 0.5
    %       'EdgeColor'  default: [0;0;0]
    %       'EdgeAlpha'  default: 1
    %
    % can accept existing handle to cylinder patch using 'Handle'
    %
    % h_p is a struct containing handles to the patch objects
    %       h_p.h_sides
    %       h_p.h_caps
    
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
    
    r = dim(1); h = dim(2);
    n = 20;
    X = r*cos(0:2*pi/n:2*pi);
    Y = r*sin(0:2*pi/n:2*pi);
    Z = 1/2*h*[1 -1];
    
    side_faces = zeros(3,4*n);

    for i=1:n
        side_faces(:,(i-1)*4+1:i*4) = [X(i:i+1) X(i+1:-1:i); ...
                                       Y(i:i+1) Y(i+1:-1:i); ...
                                       Z(1) Z(1) Z(2) Z(2)];
    end

    side_faces_rot = t*ones(1,4*n) + R*side_faces;
    sides_X = reshape(side_faces_rot(1,:),[4 n]);
    sides_Y = reshape(side_faces_rot(2,:),[4 n]);
    sides_Z = reshape(side_faces_rot(3,:),[4 n]);

    cap_faces = [X(1:n) X(1:n);Y(1:n) Y(1:n); Z(1)*ones(1,n) Z(2)*ones(1,n)];
    cap_faces_rot = t*ones(1,2*n) + R*cap_faces;
    caps_X = reshape(cap_faces_rot(1,:),[n 2]);
    caps_Y = reshape(cap_faces_rot(2,:),[n 2]);
    caps_Z = reshape(cap_faces_rot(3,:),[n 2]);

    if exist('h_p','var')
        set(h_p.h_sides,'XData',sides_X,'YData',sides_Y,'ZData',sides_Z);
        set(h_p.h_caps,'XData',caps_X,'YData',caps_Y,'ZData',caps_Z);
    else
        h_p.h_sides = patch(sides_X,sides_Y,sides_Z,1);
        h_p.h_caps = patch(caps_X,caps_Y,caps_Z,1);
        
        set(h_p.h_sides,'FaceColor',fc, ...
                        'FaceAlpha',fa, ...
                        'LineWidth',lw, ...
                        'EdgeColor',ec, ...
                        'EdgeAlpha',ea);
        set(h_p.h_caps,'FaceColor',fc, ...
                        'FaceAlpha',fa, ...
                        'LineWidth',lw, ...
                        'EdgeColor',ec, ...
                        'EdgeAlpha',ea);
    end
end