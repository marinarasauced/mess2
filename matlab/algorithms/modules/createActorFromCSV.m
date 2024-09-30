function actor = createActorFromCSV(path_file)
    % This function reads a CSV file and returns an instance of MyClass
    % where the second row corresponds to the properties of the class.

    % Read the CSV file
    data = readmatrix(path_file); % Read numeric data from the CSV

    % Extract properties from the second row
    k_ang      = data(1, 1); % First column
    k_lin      = data(1, 2); % Second column
    x_tol_ang  = data(1, 3); % Third column
    x_tol_lin  = data(1, 4); % Fourth column
    u_max_ang  = data(1, 5); % Fifth column
    u_max_lin  = data(1, 6); % Sixth column
    radius     = data(1, 7); % Seventh column
    t_wait     = data(1, 8); % Eigth column

    % Create an instance of MyClass using the extracted properties
    actor = actorTurtleBot3(k_ang, k_lin, x_tol_ang, x_tol_lin, u_max_ang, u_max_lin, radius, t_wait);
end