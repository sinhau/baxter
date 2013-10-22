function h_p = drawCuboidPatch(R, t, dim, varargin)
    %
    % h_p = drawCuboidPatch(R, t, dim,...)
    %
    % R,t are rotation and translation of the block
    % dim is 3 dimensional vector giving width [w], height [h], length [l]
    % possible additional parameters are:
    %       'FaceColor'  default: [1;1;1]
    %       'FaceAlpha'  default: 1
    %       'LineWidth'  default: 0.5
    %       'EdgeColor'  default: [0;0;0]
    %       'EdgeAlpha'  default: 1
    %
    % can accept existing handle to cuboid patch using 'Handle'
    %
    % h_p is handle to the patch object
    
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
    
    % X associated with width
    % Y associated with height
    % Z associated with length
    w = dim(1); h = dim(2); l = dim(3);
    
    left_face = 0.5*[-w -w -w -w; -h h h -h; l l -l -l];
    right_face = 0.5*[w w w w; -h h h -h; l l -l -l];
    top_face = 0.5*[-w -w w w; h h h h; l -l -l l];
    bottom_face = 0.5*[-w -w w w; -h -h -h -h; l -l -l l];
    front_face = 0.5*[-w -w w w; h -h -h h; l l l l];
    back_face = 0.5*[-w -w w w; h -h -h h; -l -l -l -l];

    faces_rot = t*ones(1,24) + ...
        R*[left_face right_face top_face bottom_face ...
            front_face back_face];

    faces_X = reshape(faces_rot(1,:),[4 6]);
    faces_Y = reshape(faces_rot(2,:),[4 6]);
    faces_Z = reshape(faces_rot(3,:),[4 6]);
    
    if exist('h_p','var')
        set(h_p,'XData',faces_X,'YData',faces_Y,'ZData',faces_Z);
    else
        h_p = patch(faces_X,faces_Y,faces_Z,1);
        set(h_p,'FaceColor',fc, ...
                'FaceAlpha',fa, ...
                'LineWidth',lw, ...
                'EdgeColor',ec, ...
                'EdgeAlpha',ea);
    end
end