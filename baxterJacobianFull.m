function j = baxterJacobianFull(q)

    % Rotation vectors
    x = [1;0;0];y = [0;1;0];z = [0;0;1];
    h1 = z;
    h2 = -x;
    h3 = y;
    h4 = -x;
    h5 = y;
    h6 = -x;
    h7 = y;
    
    % Rotation matrices
    r01 = rot(h1,q(1));
    r12 = rot(h2,q(2));
    r23 = rot(h3,q(3));
    r34 = rot(h4,q(4));
    r45 = rot(h5,q(5));
    r56 = rot(h6,q(6));
    r67 = rot(h7,q(7));
        
    % Position vectors
    p01 = [0;0;0];
    p12 = [0;0.1;0.235];
    p23 = [0;0.1;0];
    p34 = [0;0.24;-0.095];
    p45 = [0;0.09;0];
    p56 = [0;0.28;-0.02];
    p67 = [0;0.18;0];
    p7T = [0;0.17;0];
    
    % Calculate jacobian
    j1 = hat(r01*h1)*(r01*(p12+r12*(p23+r23*(p34+r34*(p45+r45*(p56+r56*(p67+r67*(p7T))))))));
    j2 = hat(r01*r12*h2)*(r01*r12*(p23+r23*(p34+r34*(p45+r45*(p56+r56*(p67+r67*(p7T)))))));
    j3 = hat(r01*r12*r23*h3)*(r01*r12*r23*(p34+r34*(p45+r45*(p56+r56*(p67+r67*(p7T))))));
    j4 = hat(r01*r12*r23*r34*h4)*(r01*r12*r23*r34*(p45+r45*(p56+r56*(p67+r67*(p7T)))));
    j5 = hat(r01*r12*r23*r34*r45*h5)*(r01*r12*r23*r34*r45*(p56+r56*(p67+r67*(p7T))));
    j6 = hat(r01*r12*r23*r34*r45*r56*h6)*(r01*r12*r23*r34*r45*r56*(p67+r67*(p7T)));
    j7 = hat(r01*r12*r23*r34*r45*r56*r67*h7)*(r01*r12*r23*r34*r45*r56*r67*(p7T));
    j = [[r01*h1;j1],[r01*r12*h2;j2],[r01*r12*r23*h3;j3],[r01*r12*r23*r34*h4;j4],[r01*r12*r23*r34*r45*h5;j5],[r01*r12*r23*r34*r45*r56*h6;j6],[r01*r12*r23*r34*r45*r56*r67*h7;j7]];

end