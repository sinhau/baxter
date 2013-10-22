function h_p = drawGripperPatch(R, t, dim, varargin)
    % 
    % h_p = drawGripperPatch(R, t, dim,...)
    %
    % R,t are rotation and translation of the block
    % dim is 2 dimensional vector giving width [w] and height [h]
    % possible additional parameters are:
    %       'Color'      default: [0;0;0]
    %       'LineWidth'  default: 2
    %
    % can accept existing handle to gripper patch using 'Handle'
    %
    
    % Walk through varargin
    for i=1:2:(nargin-3)
        if strcmp(varargin{i},'Handle')
            h_p = varargin{i+1};
        elseif strcmp(varargin{i},'Color')
            c = varargin{i+1};
        elseif strcmp(varargin{i},'LineWidth')
            lw = varargin{i+1};
        else
            error(['Parameter not recognized: ' varargin{i}]);
        end
    end
    if ~exist('c','var')
        c = [0;0;0];
    end
    if ~exist('lw','var')
        lw = 2;
    end
    w = dim(1); h = dim(2);
    
    X = [0 0 0 0];
    Y = 0.5*[-w -w w w];
    Z = [ h 0 0 h];
    
    gripper = t*ones(1,4) + R*[X;Y;Z];
    X = gripper(1,:)';
    Y = gripper(2,:)';
    Z = gripper(3,:)';
    if exist('h_p','var')
        set(h_p,'XData',X,'YData',Y,'ZData',Z);
    else
        h_p = line(X,Y,Z);
        set(h_p,'Color',c,'LineWidth',lw);
    end
end