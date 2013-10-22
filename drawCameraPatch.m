function h_p = drawCameraPatch(camconst, R, t,varargin)
    %
    % h_p = drawCylinderPatch(R, t, dim,...)
    %
    % camconst is struct defining camera constants
    % R,t are orientation and position of the camera (point is tip of lens)
    % possible additional parameters are:
    %       'FaceColor'  default: [0.25;0.25;0.25]
    %       'FaceAlpha'  default: 1
    %       'LineWidth'  default: 1
    %       'EdgeColor'  default: [0;0;0]
    %       'EdgeAlpha'  default: 1
    %
    % can accept existing handle to camera patch using 'Handle'
    %
    % h_p is a struct containing handles to the patch objects
    %       h_p.h_body    (drawCuboidPatch handle)
    %       h_p.h_lens    (drawCylinderPatch handle)
    
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
        fc = [0.25;0.25;0.25];
    end
    if ~exist('fa','var')
        fa = 1;
    end
    if ~exist('lw','var')
        lw = 1;
    end
    if ~exist('ec','var')
        ec = [0;0;0];
    end
    if ~exist('ea','var')
        ea = 1;
    end
    
    LENS_HEIGHT = 0.044;
    LENS_R = 0.02;
    BODY_T = t-R*[0;0;LENS_HEIGHT+camconst.dim(3)/2];
    LENS_T = t-R*[0;0;LENS_HEIGHT/2];
    if exist('h_p','var')
        h_body = h_p.h_body;
        h_lens = h_p.h_lens;
    end
    
    if exist('h_body','var')
        h_body = drawCuboidPatch(R,BODY_T,camconst.dim,'Handle',h_body);
    else
        h_body = drawCuboidPatch(R,BODY_T,camconst.dim,...
            'FaceColor',fc,'FaceAlpha',fa,'LineWidth',lw,...
            'EdgeColor',ec,'EdgeAlpha',ea);
    end
    if exist('h_lens','var')
        h_lens = drawCylinderPatch(R,LENS_T,[LENS_R;LENS_HEIGHT],...
                        'Handle',h_lens);
    else
    	h_lens = drawCylinderPatch(R,LENS_T,[LENS_R;LENS_HEIGHT],...
            'FaceColor',fc,'FaceAlpha',fa,'LineWidth',lw,...
            'EdgeColor',ec,'EdgeAlpha',ea);
    end
    h_p.h_body = h_body;
    h_p.h_lens = h_lens;
end