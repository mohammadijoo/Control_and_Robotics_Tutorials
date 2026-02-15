function cooperative_planar_lab()
    % Contact points on a bar
    contacts = [0.5, 0.0;
                -0.5, 0.0];

    G = planarGraspMatrix(contacts);
    m = size(contacts, 1);
    W = eye(2 * m);

    m_obj = 2.0;
    g = 9.81;
    w_des = [0.0; m_obj * g; 0.0];

    GWG = G / W * G';
    lambda_vec = GWG \ w_des;
    f_c_star = (W \ G') * lambda_vec;

    disp('Grasp matrix G = ');
    disp(G);
    disp('Optimal contact forces f_c* = ');
    disp(f_c_star);

    % Task allocation example
    C = [4.0, 2.0, 3.5;
         2.5, 3.0, 2.0;
         3.0, 4.0, 1.5];

    % matchpairs solves a linear assignment problem
    [pairs, totalCost] = matchpairs(C, 1e3);
    disp('Assignments [robot, task]:');
    disp(pairs);
    disp(['Total cost = ', num2str(totalCost)]);
end

function G = planarGraspMatrix(contacts)
    m = size(contacts, 1);
    G = zeros(3, 2 * m);
    for i = 1:m
        x_i = contacts(i, 1);
        y_i = contacts(i, 2);
        col = 2 * (i - 1) + 1;
        G(1, col)     = 1.0;
        G(2, col + 1) = 1.0;
        G(3, col)     = -y_i;
        G(3, col + 1) = x_i;
    end
end
      
