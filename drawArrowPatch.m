function h_p = drawArrowPatch(pos, vec, dim, varargin)
    %
    % h_p = drawArrowPatch(pos, vec, dim, ...)
    %
    % pos: x,y,z position of the arrow (bottom of arrow)
    % vec: u,v,w vector giving direction and magnitude of arrow
    % dim is 2-dimensional vector denoting width [w] of arrow shaft 
    %       and size of head as percentage of body (0.85 - 0.99)
    %           -> if left empty defaults are [0.03*norm(vec), 0.9]
    %
    % possible additional parameters are:
    %           'Color'     default: [0;0;1]
    %           'Alpha'     default: 1
    %
    % can accept existing handle to arrow patch object using 'Handle' flag
    %
    % returns struct of patch object handles containing
    %       h_shaft
    %       h_head
    
    % Walk through varargin
    for i=1:2:(nargin-3)
        if strcmp(varargin{i},'Handle')
            h_p = varargin{i+1};
        elseif strcmp(varargin{i},'Color')
            c = varargin{i+1};
        elseif strcmp(varargin{i},'Alpha')
            a = varargin{i+1};
        else
            error(['Parameter not recognized: ' varargin{i}]);
        end
    end
    % Set defaults if not already established
    if ~exist('c','var')
        c = [0;0;1];
    end
    if ~exist('a','var')
        a = 1;
    end
    if isempty(dim)
        dim = [0.03*norm(vec) 0.9];
    end
    
    if dim(2) < 0.7; dim(2) = 0.7; end;
    if dim(2) > 0.99; dim(2) = 0.99; end;
    
    % Preparation for drawing the shaft
    z = [0;0;1];    
    shaft_dim = [dim(1)/2 dim(2)*norm(vec)];
    if norm(vec) ~= 0
        k = vec / norm(vec);
        if dot(k,z)==1
            shaft_R = eye(3);
        elseif dot(k,z)==-1
            shaft_R = rot([1;0;0],pi);
        else
            phi = subproblem1(hat(k)*z,k,z);
            shaft_R = rot(hat(k)*z,-phi(1));
        end
    else
        shaft_R = eye(3);
    end
    shaft_t = pos + shaft_R*[0;0;shaft_dim(2)/2];
    
    % Prepping for drawing the head.  tip of head is reference point
    head_t = pos + shaft_R*[0;0;norm(vec)];
    w = 3*dim(1); d = dim(1); h = (1-dim(2))*norm(vec); 
    front_face = [0 w/2 -w/2 -w/2; d/2 d/2 d/2 d/2; 0 -h -h -h];
    back_face = [0 w/2 -w/2 -w/2; -d/2 -d/2 -d/2 -d/2; 0 -h -h -h];
    left_face = [0 -w/2 -w/2 0; d/2 d/2 -d/2 -d/2; 0 -h -h 0];
    right_face = [0 w/2 w/2 0; d/2 d/2 -d/2 -d/2; 0 -h -h 0];
    bottom_face = [-w/2 w/2 w/2 -w/2; -d/2 -d/2 d/2 d/2; -h -h -h -h];
    faces_rot = head_t*ones(1,20) + ...
            shaft_R*[front_face back_face left_face ...
                        right_face bottom_face];
    faces_X = reshape(faces_rot(1,:),[4 5]);
    faces_Y = reshape(faces_rot(2,:),[4 5]);
    faces_Z = reshape(faces_rot(3,:),[4 5]);
    
    if exist('h_p','var')
        h_p.h_shaft = drawCylinderPatch(shaft_R,shaft_t,shaft_dim, ...
                        'Handle', h_p.h_shaft);
        set(h_p.h_head,'XData',faces_X,'YData',faces_Y,'ZData',faces_Z);
    else
        h_p.h_shaft = drawCylinderPatch(shaft_R,shaft_t,shaft_dim, ...
                            'FaceColor', c, ...
                            'FaceAlpha', a, ...
                            'EdgeAlpha', 0);
        h_p.h_head = patch(faces_X,faces_Y,faces_Z,1);
        set(h_p.h_head,'FaceColor',c, ...
                    'FaceAlpha',a, ...
                    'EdgeAlpha',0);
    end
end

    function [theta]=subproblem1(k,p,q)

        if norm(p-q)<sqrt(eps);theta=0;return;end

        k=k/norm(k);
        pp=p-(p'*k)*k;
        qp=q-(q'*k)*k;
        epp=pp/norm(pp);
        eqp=qp/norm(qp);

        theta=subproblem0(epp,eqp,k);

        tol=1e-2;
        if (abs(norm(p)-norm(q))>tol);
          disp('*** Warning *** ||p|| and ||q|| must be the same!!!');
        end
    end
    
    function [theta]=subproblem0(p,q,k)

        if ((k'*p)>sqrt(eps)||(k'*q)>sqrt(eps))
          error('k must be perpendicular to p and q');
        end

        ep=p/norm(p);
        eq=q/norm(q);

        theta=2*atan2(norm(ep-eq),norm(ep+eq));

        if k'*(cross(p,q))<0
          theta=-theta;
        end
    end