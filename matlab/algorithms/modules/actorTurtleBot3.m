classdef actorTurtleBot3
    properties
        k_ang      % Angular gain
        k_lin      % Linear gain
        x_tol_ang  % Angular tolerance
        x_tol_lin  % Linear tolerance
        u_max_ang  % Maximum angular control input
        u_max_lin  % Maximum linear control input
        radius     % Radius
        t_wait     % Wait time for wait transitions
        goals      % Property to store goals
        states     % Property to store state (x, y, theta)
        times
        error
    end

    methods
        % Constructor method to initialize properties
        function obj = actorTurtleBot3(k_ang, k_lin, x_tol_ang, x_tol_lin, u_max_ang, u_max_lin, radius, t_wait)
            obj.k_ang = k_ang;
            obj.k_lin = k_lin;
            obj.x_tol_ang = x_tol_ang;
            obj.x_tol_lin = x_tol_lin;
            obj.u_max_ang = u_max_ang;
            obj.u_max_lin = u_max_lin;
            obj.radius = radius;
            obj.t_wait = t_wait;
            obj.goals = [];
            obj.states = [];
            obj.times = [];
            obj.error = [inf, inf, inf];
        end

        function obj = loadGoals(obj, path_file)
            % Read goals from the CSV file
            data = readmatrix(path_file);
            
            % Check if there are enough rows
            if size(data, 1) < 1
                error('The CSV file must have at least one row of goals.');
            end
            
            % Store goals (assuming columns are [x, y, theta])
            obj.goals = [data(:, 1:2), data(:, 4)]; % Store the entire matrix as goals
        end

        function obj = simulateToGoals(obj, dt)
             % Initialize state
             obj.states(end + 1, :) = obj.goals(1, :) + [0.01, 0.01, 0.0];
             obj.times(end + 1, 1) = 0.0;

             % Iterate through goals
             for iter = 2:size(obj.goals, 1)
                init = obj.goals(iter - 1, :);
                init(1, 3) = wrapToPi(init(1, 3));
                trgt = obj.goals(iter, :);
                trgt(1, 3) = wrapToPi(trgt(1, 3));

                is_same_x = (init(1, 1) == trgt(1, 1));
                is_same_y = (init(1, 2) == trgt(1, 2));
                is_same_theta = (init(1, 3) == trgt(1, 3));

                is_wait = (is_same_x && is_same_y && is_same_theta);
                is_rotate = (is_same_x && is_same_y && ~is_same_theta);
                is_translate = (~is_wait && ~is_rotate && is_same_theta);

                % disp([init;trgt])
                % disp([is_wait, is_rotate, is_translate])

                if is_wait
                    obj = obj.executeWait(dt);
                elseif is_rotate
                    obj = obj.executeRotate(init, trgt, dt);
                elseif is_translate
                    obj = obj.executeTranslation(init, trgt, dt);
                end
             end
        end

        function obj = executeWait(obj, dt)
            t_init = obj.times(end, 1);
            t_term = t_init + obj.t_wait;
            while (obj.times(end, 1) < t_term)
                obj.states(end + 1, :) = obj.states(end, :);
                obj.times(end + 1, 1) = obj.times(end, 1) + dt;
            end
            %
            % t_curr = obj.states(end, 1);
            % t_stop = t_curr + obj.t_wait;
            % steps = round((t_stop - t_curr) / dt);
            % state_curr = obj.states(end, :);
            % for iter = 1:steps
            %     state_curr(1, 1) = state_curr(1, 1) + dt;
            %     obj.states(end + 1, :) = state_curr;
            % end
        end

        function obj = executeRotate(obj, init, trgt, dt)
            obj.error(1, :) = [inf, inf, inf];
            while (abs(obj.error(1, 3)) > obj.x_tol_ang)
                obj = obj.setError(init, trgt);
                t_curr = obj.times(end, 1);
                x_curr = obj.states(end, :)';
                u_lin = 0.0;
                u_ang = -obj.k_ang * obj.error(1, 3);
                u_curr = [u_lin; u_ang];
                if (abs(u_curr(1, 1)) > obj.u_max_lin)
                    u_curr(1, 1) = sign(u_curr(1, 1)) * obj.u_max_lin;
                end
                if (abs(u_curr(2, 1)) > obj.u_max_ang)
                    u_curr(2, 1) = sign(u_curr(2, 1)) * obj.u_max_ang;
                end

                k1 = obj.getSysDyn(x_curr, u_curr) * dt;
                k2 = obj.getSysDyn(x_curr + k1 / 2, u_curr) * dt;
                k3 = obj.getSysDyn(x_curr + k2 / 2, u_curr) * dt;
                k4 = obj.getSysDyn(x_curr + k3, u_curr) * dt;

                t_curr = t_curr + dt;
                x_curr = x_curr + k1 / 6 + k2 / 3 + k3 / 3 + k4 / 6;
                obj.times(end + 1, 1) = t_curr;
                obj.states(end + 1, :) = x_curr';
            end
        end

        function obj = executeTranslation(obj, init, trgt, dt)
            obj.error(1, :) = [inf, inf, inf];
            while (abs(obj.error(1, 1)) > obj.x_tol_lin)
                obj = obj.setError(init, trgt);
                t_curr = obj.times(end, 1);
                x_curr = obj.states(end, :)';
                u_lin = obj.u_max_lin;
                u_ang = -obj.k_ang * obj.error(1, 3) -obj.k_lin * obj.error(1, 2);
                u_curr = [u_lin; u_ang];
                if (abs(u_curr(1, 1)) > obj.u_max_lin)
                    u_curr(1, 1) = sign(u_curr(1, 1)) * obj.u_max_lin;
                end
                if (abs(u_curr(2, 1)) > obj.u_max_ang)
                    u_curr(2, 1) = sign(u_curr(2, 1)) * obj.u_max_ang;
                end

                k1 = obj.getSysDyn(x_curr, u_curr) * dt;
                k2 = obj.getSysDyn(x_curr + k1 / 2, u_curr) * dt;
                k3 = obj.getSysDyn(x_curr + k2 / 2, u_curr) * dt;
                k4 = obj.getSysDyn(x_curr + k3, u_curr) * dt;

                t_curr = t_curr + dt;
                x_curr = x_curr + k1 / 6 + k2 / 3 + k3 / 3 + k4 / 6;
                x_curr(3, 1) = wrapToPi(x_curr(3, 1));
                obj.times(end + 1, 1) = t_curr;
                obj.states(end + 1, :) = x_curr';
            end
        end

        function obj = setError(obj, init, trgt)

            is_same_x = (init(1, 1) == trgt(1, 1));
            is_same_y = (init(1, 2) == trgt(1, 2));
            curr = obj.states(end, :);
            if (is_same_x && is_same_y)
                % Rotate
                obj.error(1, 1) = 0.0;
                obj.error(1, 2) = 0.0;
                obj.error(1, 3) = wrapToPi(curr(1, 3) - trgt(1, 3));
            elseif (is_same_x && ~is_same_y)
                % Translate along vertical
                obj.error(1, 1) = trgt(1, 2) - curr(1, 2);
                obj.error(1, 2) = trgt(1, 1) - curr(1, 1);
                obj.error(1, 3) = wrapToPi(curr(1, 3) - trgt(1, 3));
            else
                A = [trgt(1, 1) - init(1, 1); trgt(1, 2) - init(1, 2)];
                B = [trgt(1, 1) - curr(1, 1); trgt(1, 2) - curr(1, 2)];
                C = [curr(1, 1) - init(1, 1); curr(1, 2) - init(1, 2)];
                a = norm(A);
                b = norm(B);
                alpha = acos(dot(A, B) / (a * b));
                beta = atan2(C(2, 1), C(1, 1));
                theta = atan2(A(2, 1), A(1, 1));
                obj.error(1, 1) = b * cos(alpha);
                obj.error(1, 2) = b * sin(alpha) * sign(beta - theta);
                obj.error(1, 3) = wrapToPi(curr(1, 3) - theta);
            end
        end

        function x_dot = getSysDyn(obj, x_curr, u_curr)
            x3 = x_curr(3, 1);
            u1 = u_curr(1, 1);
            u2 = u_curr(2, 1);

            x_dot = zeros(3, 1);
            x_dot(1, 1) = u1 * cos(x3);
            x_dot(2, 1) = u1 * sin(x3);
            x_dot(3, 1) = u2;
        end
        
        % Example method to display properties
        function displayProperties(obj)
            fprintf('Angular Gain (k_ang): %f\n', obj.k_ang);
            fprintf('Linear Gain (k_lin): %f\n', obj.k_lin);
            fprintf('Angular Tolerance (x_tol_ang): %f\n', obj.x_tol_ang);
            fprintf('Linear Tolerance (x_tol_lin): %f\n', obj.x_tol_lin);
            fprintf('Max Angular Input (u_max_ang): %f\n', obj.u_max_ang);
            fprintf('Max Linear Input (u_max_lin): %f\n', obj.u_max_lin);
            fprintf('Radius: %f\n', obj.radius);
            fprintf('Wait Time: %f\n', obj.t_wait);
        end
    end
end
