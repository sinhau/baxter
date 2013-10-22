function h_p = drawFramePatch(R, t, scale, varargin)
    %
    % h_p = drawFramePatch(R, t, scale, ...)
    %
    % R, t are rotation and translation of the frame
    % scale defines the length of frame vectors
    %
    % Additional Parameters include:
    %          'LineWidth': default 0.03*scale
    %
    % can accept existing handle to frame patch using 'Handle'
    %
    % h_p is handle to struct containing patch objects
    %       h_p.h_x
    %       h_p.h_y
    %       h_p.h_z
    
    % Walk through varargin
    for i=1:2:(nargin-3)
        if strcmp(varargin{i},'Handle')
            h_p = varargin{i+1};
        elseif strcmp(varargin{i},'LineWidth')
            lw = varargin{i+1};
        else
            error(['Parameter not recognized: ' varargin{i}]);
        end
    end
    % Set default values
    if ~exist('lw','var')
        lw = 0.03*scale;
    end
    
    
    Rx = R*rot([0;1;0],pi/2);
    tx = t + Rx*[0;0;scale/2];
    Ry = R*rot([1;0;0],-pi/2);
    ty = t + Ry*[0;0;scale/2];
    Rz = R;
    tz = t + Rz*[0;0;scale/2];
    
    if ~exist('h_p','var')
        h_x = drawCylinderPatch(Rx,tx,[lw scale], ...
                                'FaceColor',[1;0;0], ...
                                'EdgeAlpha', 0);
        h_y = drawCylinderPatch(Ry,ty,[lw scale], ...
                                'FaceColor',[0;1;0], ...
                                'EdgeAlpha', 0);
        h_z = drawCylinderPatch(Rz,tz,[lw scale], ...
                                'FaceColor',[0;0;1], ...
                                'EdgeAlpha', 0);
    else
        h_x = drawCylinderPatch(Rx,tx,[lw scale], ...
                                'Handle',h_p.h_x);
        h_y = drawCylinderPatch(Ry,ty,[lw scale], ...
                                'Handle',h_p.h_y);
        h_z = drawCylinderPatch(Rz,tz,[lw scale], ...
                                'Handle',h_p.h_z);
    end
    h_p.h_x = h_x;
    h_p.h_y = h_y;
    h_p.h_z = h_z;
end
    