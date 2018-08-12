
function init_ped_grid(env::UrbanEnv, pos_res::Float64, vel_res::Float64)
    d = Dict{LaneTag, RectangleGrid{3}}()
    phi_space = SVector{2, Float64}(0., float(pi))
    v_space = get_ped_vspace(env, vel_res)
    for lane in env.crosswalks
        l_space = get_discretized_lane(lane.tag, env.roadway, pos_res)
        grid = RectangleGrid(l_space, v_space,  phi_space)
        d[lane.tag] = grid
    end
    return d
end

function init_car_grid(env::UrbanEnv, pos_res::Float64, vel_res::Float64)
    d = Dict{LaneTag, RectangleGrid{2}}()
    v_space = get_car_vspace(env, vel_res)
    for i=1:length(env.roadway.segments)
        for lane in env.roadway.segments[i].lanes
            l_space = get_discretized_lane(lane.tag, env.roadway, pos_res)
            grid = RectangleGrid(l_space, v_space)
            d[lane.tag] = grid
        end
    end
    return d
end
function init_l_grid(env::UrbanEnv, pos_res::Float64)
    d = Dict{LaneTag, RectangleGrid{1}}()
    for i=1:length(env.roadway.segments)
        for lane in env.roadway.segments[i].lanes
            l_space = get_discretized_lane(lane.tag, env.roadway, pos_res)
            grid = RectangleGrid(l_space)
            d[lane.tag] = grid
        end
    end
    return d
end

function init_v_grid(env::UrbanEnv, vel_res::Float64)
    v_space = get_car_vspace(env, vel_res)
    return RectangleGrid(v_space)
end

# a bunch of interpolation helpers
function AutomotivePOMDPs.interpolate_state(mdp::PedCarMDP, state::VehicleState)
    # interpolate longitudinal position and velocity
    if state.posG == mdp.off_grid
        return VehicleState[state], Float64[1.0]
    end
    lane = get_lane(mdp.env.roadway, state)
    l_space = get_discretized_lane(lane.tag, mdp.env.roadway, mdp.pos_res)
    grid = mdp._car_grid[lane.tag]
    real_state = SVector{2, Float64}(state.posF.s, state.v)
    idx, weights = interpolants(grid, real_state)
    n_pts = length(idx)
    states = Vector{VehicleState}(n_pts)
    probs = zeros(n_pts)
    for i=1:n_pts
        sg, vg = ind2x(grid, idx[i])
        states[i] = VehicleState(Frenet(lane, sg), mdp.env.roadway, vg)
        probs[i] = weights[i]
    end
    return states, probs
end

# take into account heading as well
function AutomotivePOMDPs.interpolate_pedestrian(mdp::PedCarMDP, state::VehicleState)
    # interpolate longitudinal position and velocity
    if state.posG == mdp.off_grid
        return (state,), (1.0,)
    end
    lane = get_lane(mdp.env.ped_roadway, state)
    grid = mdp._ped_grid[lane.tag]
    real_state = SVector{3, Float64}(state.posF.s, state.v, state.posF.ϕ)
    idx, weights = interpolants(grid, real_state)
    n_pts = length(idx)
    states = Vector{VehicleState}(n_pts)
    probs = zeros(n_pts)
    for i=1:n_pts
        sg, vg, phig = ind2x(grid, idx[i])
        states[i] = VehicleState(Frenet(lane, sg, 0., phig), mdp.env.ped_roadway, vg)
        probs[i] = weights[i]
    end
    return states, probs
end