close all; clear variables; clc;

% Define paths
path_actors  = "config/actors/";
path_goals  = "config/goals/";
path_edges   = "config/edges.csv";
path_vertices = "config/vertices.csv";
path_threat  = "config/threat.csv";

%
dt = 0.001;

% Get the list of files in the actors directory
list_actors = dir(path_actors);

% Filter out non-file entries (e.g., '.' and '..')
list_actors = list_actors(~ismember({list_actors.name}, {'.', '..'}));

% Initialize a string array to store actor file names
paths_actors = strings(length(list_actors), 1);

% Loop through the list and store the file names
for iter = 1:length(list_actors)
    paths_actors(iter) = list_actors(iter).name; % Store the name in the string array
end

% Initialize an array to store instances of actorTurtleBot3
actors = cell(length(paths_actors), 1); 

% Create an instance of actorTurtleBot3 for each actor file
for iter = 1:length(paths_actors)
    % Construct the full path to the CSV file
    path_file = fullfile(path_actors, paths_actors(iter));

    % Create an actor instance from the CSV file
    try
        actors{iter} = createActorFromCSV(path_file);
        fprintf('Created actor from file: %s\n', path_file);
    catch ME
        fprintf('Failed to create actor from file: %s\nError: %s\n', path_file, ME.message);
    end
end

% Load goals into actorTurtleBot3 for each actor
for iter = 1:length(paths_actors)
    % Construct the path to the CSV file
    path_file = fullfile(path_goals, paths_actors(iter));

    try
        actors{iter} = actors{iter}.loadGoals(path_file);
        fprintf('Loaded goals for actor %d from file: %s\n', iter, path_file);
    catch ME
        fprintf('Failed to load goals for actor %d from file: %s\nError: %s\n', iter, goals_filename, ME.message);
    end
end

% Simulate goal trajectories numerically for each actor
steps = 0;
for iter = 1:length(paths_actors)
    try
        actors{iter} = actors{iter}.simulateToGoals(dt);
        fprintf('Simulated goals for actor %d\n', iter);
        if (length(actors{iter}.states) > steps)
            steps = length(actors{iter}.states);
        end
    catch ME
        fprintf('Failed to simulate goals for actor %d: %s\nError: %s\n', iter, ME.message);

        % Print the error identifier and stack trace
        fprintf('Error occurred in function: %s\n', ME.stack(1).name);
        fprintf('Line number: %d\n', ME.stack(1).line);
        
        % Optionally, print more details from the stack if necessary
        for k = 1:length(ME.stack)
            fprintf('In file: %s (line %d)\n', ME.stack(k).file, ME.stack(k).line);
        end
    end
end


% Load threat field
threat = readmatrix(path_threat);
fprintf('Loaded threat from file: %s\n', path_threat);

% Define figure for plotting
figure;

% Now simulate and update the plot at each time step
scale = 100;
for step = 2:steps / scale
    % Clear the current figure content
    clf;
    
    % Replot the threat field
    scatter(threat(:, 1), threat(:, 2), 64, threat(:, 3), 'filled');
    colormap('parula');
    hold on;
    axis equal;
    xlim([-6, 6]);
    ylim([-6, 6]);
    
    % Update and plot actor positions
    for iter = 1:length(actors)
        try
            if (step * scale > length(actors{iter}.states))
                pos = actors{iter}.states(end, :); % If out of bounds, use the last state
            else
                pos = actors{iter}.states(step * scale, :); % Use the current state
            end
            radius = actors{iter}.radius;

            % Plot the actor's circle at the updated position
            viscircles(pos(1:2), radius, 'EdgeColor', '#E01F38');
        catch ME
            fprintf('Failed to update actor %d at step %d: %s\n', iter, step, ME.message);
        end
    end
    
    % Add a small pause to control the animation speed
    pause(0.01);  % Adjust the time for smoother or faster animation
end
