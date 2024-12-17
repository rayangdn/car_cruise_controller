function result = simulate(params)

function ref = get_ref(car, i, t, x, xOther)
    if isfield(car, 'ref')
        if isnumeric(car.ref)
            if size(car.ref, 2) == 1
                ref = car.ref;
            else
                ref = car.ref(:, i);
            end
        elseif isa(car.ref, 'function_handle')
            ref = car.ref(t, x, xOther);
        else
            error(['car.ref must be a static input or a control ' ...
                   'function handle ref_fcn = @(t, x, xOther).']);
        end
    else
        ref = zeros(2, 1);
    end
end

function u = get_u(car, i, x, ref, xOther)
    if isfield(car, 'u')
        if isnumeric(car.u)
            if size(car.u, 2) == 1
                u = car.u;
            else
                u = car.u(:, i);
            end
        elseif isa(car.u, 'function_handle')
            u = car.u(x, ref, xOther);
        else
            error(['car.u must be a static input or a control ' ...
                   'function handle ctrl_fcn = @(x, xOther, ref).']);
        end
    else
        u = zeros(car.model.dimu, 1);
    end
end

function xp = simulate_step(f, x0, u, dt)
    % Integrate forward to next time step
    [~, xout] = ode45(@(t_, x_) f(x_, u), [0, dt], x0);
    xp = xout(end,:)';
end

Tf = params.Tf;
myCar = params.myCar;
Ts = myCar.model.Ts;
if isfield(params, 'otherCar') && params.otherCar.model.Ts ~= Ts
    error('myCar.model.Ts and otherCar.model.Ts must match.');
end

nx = myCar.model.dimx;
nu = myCar.model.dimu;

nSteps = ceil(Tf / Ts);
result.T = linspace(0, nSteps * Ts, nSteps + 1);

result.myCar.X = zeros(nx, nSteps + 1);
result.myCar.U = zeros(nu, nSteps + 1);
result.myCar.Ref = zeros(2, nSteps + 1);
result.myCar.X(:,1) = myCar.x0;
if isfield(myCar, 'est_fcn')
    result.myCar.Z_hat = zeros(nx + length(myCar.est_dist0), nSteps + 1);
    result.myCar.Z_hat(:,1) = [myCar.x0; myCar.est_dist0];
end

if isfield(params, 'otherCar')
    otherCar = params.otherCar;
    result.otherCar.X = zeros(otherCar.model.dimx, nSteps + 1);
    result.otherCar.U = zeros(otherCar.model.dimu, nSteps + 1);
    result.otherCar.Ref = zeros(2, nSteps + 1);
    result.otherCar.X(:,1) = otherCar.x0;
end

fprintf(['  > Simulating to Tf = ' num2str(Tf) ':\n    ( ']); tic;

sim_success = true;
for iStep = 1:nSteps+1
    % Print simulation time
    if mod(result.T(iStep), 2) == 0, fprintf([num2str(result.T(iStep)) ' ']); end
    
    if isfield(params, 'otherCar')
        otherCar = params.otherCar;
        result.otherCar.Ref(:, iStep) = get_ref(otherCar, iStep, result.T(iStep), result.otherCar.X(:, iStep), result.myCar.X(:, iStep));
        result.otherCar.U(:, iStep) = get_u(otherCar, iStep, result.otherCar.X(:, iStep), result.otherCar.Ref(:, iStep), result.myCar.X(:, iStep));
        if any(isnan(result.otherCar.U(:, iStep)))
            sim_success = false;
            break;
        end
        if iStep <= nSteps
            result.otherCar.X(:, iStep+1) = simulate_step(@(x_, u_) otherCar.model.f(x_, u_), result.otherCar.X(:, iStep), result.otherCar.U(:, iStep), Ts);
        end

        otherCarXi = result.otherCar.X(:, iStep);
    else
        otherCarXi = zeros(size(result.myCar.X(:, iStep)));
    end

    if isfield(myCar, 'est_fcn')
        myCarZ_hati = result.myCar.Z_hat(:, iStep);
    else
        myCarZ_hati = result.myCar.X(:, iStep);
    end

    result.myCar.Ref(:, iStep) = get_ref(myCar, iStep, result.T(iStep), myCarZ_hati, otherCarXi);
    result.myCar.U(:, iStep) = get_u(myCar, iStep, myCarZ_hati, result.myCar.Ref(:, iStep), otherCarXi);

    if any(isnan(result.myCar.U(:, iStep)))
        sim_success = false;
        break;
    end

    if any(result.myCar.U(:, iStep) < myCar.model.lbu - 1e-4) || any(result.myCar.U(:, iStep) > myCar.model.ubu + 1e-4)
        fprintf('Input violation');
        sim_success = false;
        break;
    end

    if iStep <= nSteps
        result.myCar.X(:, iStep+1) = simulate_step(@(x_, u_) myCar.model.f(x_, u_), result.myCar.X(:, iStep), result.myCar.U(:, iStep), Ts);

        if isfield(myCar, 'est_fcn')
            indx = myCar.model.indx;
            indu = myCar.model.indu;
            z_hat_sub = [result.myCar.Z_hat(indx.V, iStep); result.myCar.Z_hat(end+1-length(myCar.est_dist0):end, iStep)];
            z_hat_next_sub = myCar.est_fcn(z_hat_sub, result.myCar.U(indu.u_T, iStep), result.myCar.X(indx.V, iStep));
            result.myCar.Z_hat(1:nx, iStep+1) = result.myCar.X(:, iStep+1);
            result.myCar.Z_hat(indx.V, iStep+1) = z_hat_next_sub(1);
            result.myCar.Z_hat(end+1-length(myCar.est_dist0):end, iStep+1) = z_hat_next_sub(2:end);
        end
    end
end

if sim_success
    if mod(result.T(end), 2) ~= 0
        fprintf([num2str(result.T(end)) ' ']);
    end
    fprintf(') Done (%.2fs)\n', toc);

    if isfield(myCar, 'u') && isa(myCar.u, 'function_handle')
        for i = 1:nSteps+1
            if any(result.myCar.X(:, i) < myCar.model.lbx - 1e-4) || any(result.myCar.X(:, i) > myCar.model.ubx + 1e-4)
                fprintf('State constraint violation at time %.2fs\n', result.T(i));
                break;
            end
        end
    end
else
    fprintf([num2str(result.T(iStep)) ' ']);
    fprintf(')  Abort (%.2fs)\n', toc);
    fprintf('Problem when computing control from x = [ ');
    fprintf('%g ', result.myCar.X(:,iStep));
    if isfield(params, 'otherCar')
        fprintf('] xOther = [ ');
        fprintf('%g ', result.otherCar.X(:,iStep));
    end
    fprintf(']\n');
    
    % Remove remaining part of trajectory
    result.T(:,iStep+1:end) = [];
    result.myCar.X(:,iStep+1:end) = [];
    result.myCar.U(:,iStep+1:end) = [];
    result.myCar.Ref(:,iStep+1:end) = [];
    if isfield(myCar, 'est_fcn')
        result.myCar.Z_hat(:,iStep+1:end) = [];
    end
    if isfield(params, 'otherCar')
        result.otherCar.X(:,iStep+1:end) = [];
        result.otherCar.U(:,iStep+1:end) = [];
        result.otherCar.Ref(:,iStep+1:end) = [];
    end
end

end

