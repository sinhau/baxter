function h_p = drawGenericGripperPatch(R, t, dim, varargin)
    % 
    % h_p = drawGenericGripperPatch(R, t, dim,...)
    %
    % Draws a generic parallel jaw gripper.  No actuation just yet (may be
    % added in future)
    %
    % R:    orientation of the GenericGripper (3 x 3 rotation matrix)
    % t:    position of center of middle bar
    % dim:  2 dimensional vector giving width [w] and height [h]
    %       optional 3rd dimension giving block-width [bw]
    %                   -> default: 0.1*min(width,height)
    % 
    % possible additional parameters are:
    %       'Color'       default: [0;0;0]
    %
    % can accept existing handle to GenericGripper patch object 
    % using 'Handle'.  Object consists of
    %           h_left_jaw
    %           h_right_jaw
    %           h_middle_bar
    
    % Walk through varargin
    for i=1:2:(nargin-3)
        if strcmp(varargin{i},'Handle')
            h_p = varargin{i+1};
        elseif strcmp(varargin{i},'Color')
            c = varargin{i+1};
        else
            error(['Parameter not recognized: ' varargin{i}]);
        end
    end
    if ~exist('c','var')
        c = [0;0;0];
    end
    
    w = dim(1); h = dim(2);
    if length(dim)==3
        bw = dim(3);
    else
        bw = 0.1*min(dim);
    end
    
    t_lj = t + R*[0;-w/2+bw/2;h/2];
    t_rj = t + R*[0;w/2-bw/2;h/2];
    R_mj = R*rot([1;0;0],pi/2);
    
    if ~exist('h_p','var')
        h_left_jaw = drawCuboidPatch(R,t_lj,[bw,bw,h], ...
                                    'FaceColor', c, ...
                                    'EdgeColor', c);
        h_right_jaw = drawCuboidPatch(R,t_rj,[bw,bw,h], ...
                                    'FaceColor', c, ...
                                    'EdgeColor', c);
        h_middle_bar = drawCuboidPatch(R_mj,t,[bw,bw,w], ...
                                    'FaceColor', c, ...
                                    'EdgeColor', c);
    else
        h_left_jaw = drawCuboidPatch(R,t_lj,[bw,bw,h], ...
                                    'Handle', h_p.h_left_jaw);
        h_right_jaw = drawCuboidPatch(R,t_rj,[bw,bw,h], ...
                                    'Handle', h_p.h_right_jaw);
        h_middle_bar = drawCuboidPatch(R_mj,t,[bw,bw,w], ...
                                    'Handle', h_p.h_middle_bar);
    end
    h_p.h_left_jaw = h_left_jaw;
    h_p.h_right_jaw = h_right_jaw;
    h_p.h_middle_bar = h_middle_bar;
end