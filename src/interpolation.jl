
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
    if state == mdp.off_grid
        return VehicleState[state], Float64[1.0]
    end
    lane = get_lane(mdp.env.roadway, state)
    l_space = get_discretized_lane(lane.tag, mdp.env.roadway, mdp.pos_res)
    grid = mdp._car_grid[lane.tag]
    real_state = SVector{2, Float64}(state.posF.s, state.v)
    idx, weights = interpolants(grid, real_state)
    n_pts = length(idx)
    states = Vector{VehicleState}(undef, n_pts)
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
    if state == mdp.off_grid
        return (state,), (1.0,)
    end
    lane = get_lane(mdp.env.ped_roadway, state)
    grid = mdp._ped_grid[lane.tag]
    real_state = SVector{3, Float64}(state.posF.s, state.v, state.posF.ϕ)
    idx, weights = interpolants(grid, real_state)
    n_pts = length(idx)
    states = Vector{VehicleState}(undef, n_pts)
    probs = zeros(n_pts)
    for i=1:n_pts
        sg, vg, phig = ind2x(grid, idx[i])
        states[i] = VehicleState(Frenet(lane, sg, 0., phig), mdp.env.ped_roadway, vg)
        probs[i] = weights[i]
    end
    return states, probs
end

function AutomotivePOMDPs.interpolate_state(mdp::PedCarMDP, s::PedCarMDPState)
    # interpolate s in the grid
    vspace = get_car_vspace(mdp.env, mdp.vel_res)
    v_ped_space = get_ped_vspace(mdp.env, mdp.vel_ped_res)
    itp_ped, itp_ped_w = interpolate_pedestrian(mdp, s.ped)
    itp_car, itp_car_w = interpolate_state(mdp, s.car)
    itp_ego, itp_ego_w = interpolate_state(mdp, s.ego)
    itp_states = Vector{PedCarMDPState}(undef, length(itp_ego)*length(itp_car)*length(itp_ped))
    itp_w = Vector{Float64}(undef, length(itp_states))
    l = 1
    for (i, p) in enumerate(itp_ped)
        for (j, c) in enumerate(itp_car)
            for (k, e) in enumerate(itp_ego)
                itp_states[l] = PedCarMDPState(crash(mdp, s), e, p, c, s.route)
                itp_w[l] = itp_ped_w[i]*itp_car_w[j]*itp_ego_w[k]
                l += 1
            end
        end
    end
    @assert sum(itp_w) ≈ 1.
    return itp_states, itp_w
end

function AutomotivePOMDPs.get_mdp_state(mdp::PedCarMDP, pomdp::UrbanPOMDP, s::Scene, ped_id, car_id)
    car_i = findfirst(s, car_id)
    car = Vehicle(mdp.off_grid, mdp.car_type, car_id)
    if car_i != 0
        car = s[car_i]
    end
    ped_i = findfirst(s, ped_id)
    ped = Vehicle(mdp.off_grid, mdp.ped_type, ped_id)
    if ped_i != 0
        ped = s[ped_i]
    end
    ego = get_ego(s)
    # find route 
    sroute = nothing
    if haskey(pomdp.models, car_id) && car_i != 0
        # find the exact route from the list of routes
        curr_route = [l.tag for l in pomdp.models[car_id].navigator.route]
        for route in get_car_routes(mdp.env)
            tags = intersect(Set(curr_route), Set(route))
            if length(tags) >= 2
                sroute = SVector{2, LaneTag}(route[1], route[end])
            elseif length(curr_route) == 1 && curr_route[1] ∈ route
                sroute = SVector{2, LaneTag}(route[1], route[end])                
            end
        end
        if sroute == nothing
            println(curr_route)
        end       
    else 
        sroute = OFF_ROUTE
    end
    e_state = VehicleState(ego.state.posG, car_roadway(mdp.env), ego.state.v)
    p_state = VehicleState(ped.state.posG, ped.state.posF, ped.state.v)
    c_state = VehicleState(car.state.posG, car_roadway(mdp.env), car.state.v)
    return PedCarMDPState(is_colliding(ego, car), e_state, p_state, c_state, sroute)
end
