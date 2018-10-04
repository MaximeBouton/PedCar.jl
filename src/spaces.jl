POMDPs.actions(mdp::PedCarMDP) = [PedCarMDPAction(-4.0), PedCarMDPAction(-2.0), PedCarMDPAction(0.0), PedCarMDPAction(2.0)]
POMDPs.n_actions(mdp::PedCarMDP) = 4

function POMDPs.actionindex(mdp::PedCarMDP, action::PedCarMDPAction)
    if action.acc == -4.0
        return 1
    elseif action.acc == -2.0
        return 2
    elseif action.acc == 0.
        return 3
    else
        return 4
    end
end

function POMDPs.states(mdp::PedCarMDP)
    state_space = PedCarMDPState[]
    ego_states = get_ego_states(mdp.env, mdp.pos_res, mdp.vel_res)
    ped_states = get_ped_states(mdp.env, mdp.pos_res, mdp.vel_ped_res)
    push!(ped_states, mdp.off_grid)
    routes = get_car_routes(mdp.env)
    for route in routes
        for ego in ego_states
            for ped in ped_states
                car_states = get_car_states(mdp.env, route, mdp.pos_res, mdp.vel_res)
                for car in car_states
                    collision = crash(mdp, ego, ped, car)
                    # enumerate all possible routes
                    lane = get_lane(mdp.env.roadway, car)
                    push!(state_space, PedCarMDPState(collision, ego, ped, car, SVector{2, LaneTag}(route[1], route[end])))
                end
            end
        end
    end
    for ego in ego_states
        for ped in ped_states
            # add absent states
            collision = collision_checker(ego, ped, mdp.ego_type, mdp.ped_type)
            push!(state_space, PedCarMDPState(collision, ego, ped, mdp.off_grid, SVector{2, LaneTag}(LaneTag(0,0), LaneTag(0, 0))))
        end
    end
    return state_space
end

function POMDPs.n_states(mdp::PedCarMDP)
    n_ego = n_ego_states(mdp.env, mdp.pos_res, mdp.vel_res)
    n_ped = n_ped_states(mdp.env, mdp.pos_res, mdp.vel_ped_res)
    routes = get_car_routes(mdp.env)
    n_cars = 0
    for route in routes
        n_cars += n_car_states(mdp.env, route, mdp.pos_res, mdp.vel_res)
    end
    return n_ego*(n_cars + 1)*(n_ped + 1) # do not forget absent state
end


function POMDPs.stateindex(mdp::PedCarMDP, s::PedCarMDPState)
    n_ego = n_ego_states(mdp.env, mdp.pos_res, mdp.vel_res)
    n_ped = n_ped_states(mdp.env, mdp.pos_res, mdp.vel_ped_res)
    routes = get_car_routes(mdp.env)
    # step 1: find ego_index
    ego_i = ego_state_index(mdp.env, s.ego, mdp.pos_res, mdp.vel_res)
    # step 2: find ped index 
    if s.ped == mdp.off_grid
        ped_i = n_ped + 1
    else
        ped_i = ped_state_index(mdp.env, s.ped, mdp.pos_res, mdp.vel_ped_res)
    end

    # step 3 find route index 
    route_i = 0
    for (i, route) in enumerate(routes)
        if s.route[end] == route[end] && s.route[1] == route[1]
            route_i = i
        end
    end

    # handle off the grid case
    if s.car == mdp.off_grid || s.route == OFF_ROUTE
        si = 0
        for route in routes
            si += n_ego * (n_ped + 1) * n_car_states(mdp.env, route, mdp.pos_res, mdp.vel_res)
        end
        si += LinearIndices((n_ped + 1, n_ego))[ped_i, ego_i]
    else
        n_car = n_car_states(mdp.env, find_route(mdp.env, s.route), mdp.pos_res, mdp.vel_res)
        # step 2: find route_index
        route_i = 0
        for (i, route) in enumerate(routes)
            if  s.route[end] == route[end] && s.route[1] == route[1]
                route_i = i
            end
        end
        # step 3: find car_index in car states
        car_i = car_state_index(mdp.env, s.car, find_route(mdp.env, s.route), mdp.pos_res, mdp.vel_res)
        # linear/cartesian magic
        si = LinearIndices((n_car, n_ped + 1, n_ego))[car_i, ped_i, ego_i]

        for i=2:route_i
            size_r = n_ego * (n_ped + 1) * n_car_states(mdp.env, routes[i-1], mdp.pos_res, mdp.vel_res)
            si += size_r
        end
    end
    return si
end

function ind2state(mdp::PedCarMDP, si::Int64)
    n_ego = n_ego_states(mdp.env, mdp.pos_res, mdp.vel_res)
    n_ped = n_ped_states(mdp.env, mdp.pos_res, mdp.vel_ped_res)
    routes = get_car_routes(mdp.env)
    n_routes = length(routes)
    car, ped, ego = nothing, nothing, nothing
    # find route first
    ns = 0 
    route_ind = 0
    route_shift = 0
    for (i, route) in enumerate(routes)
        n_cars = n_car_states(mdp.env, route, mdp.pos_res, mdp.vel_res)
        route_shift = ns
        ns += n_cars*n_ego*(n_ped + 1)
        if ns >= si 
            route_ind = i
            break
        end
    end
    # find car, ped, ego
    if route_ind == 0 # route was not found, car is off the grid
        si_ = si - ns # shift by all the states that were added before
        car = mdp.off_grid
        # retrieve ped and ego
        ped_i, ego_i = Tuple(CartesianIndices((n_ped + 1, n_ego))[si_])
        ego = ind2ego(mdp.env, ego_i, mdp.pos_res, mdp.vel_res)
        if ped_i > n_ped
            ped = mdp.off_grid
        else
            ped = ind2ped(mdp.env, ped_i, mdp.pos_res, mdp.vel_ped_res)
        end
        collision = crash(mdp, ego, ped, car)
        return PedCarMDPState(collision, ego, ped, car, SVector{2, LaneTag}(LaneTag(0,0), LaneTag(0, 0)))
    else
        si_ = si - route_shift
        route = routes[route_ind]
        sroute = SVector{2, LaneTag}(route[1], route[end])
        n_cars = n_car_states(mdp.env, route, mdp.pos_res, mdp.vel_res)
        # use cartesian/linear magic
        car_i, ped_i, ego_i = Tuple(CartesianIndices((n_cars, n_ped + 1, n_ego))[si_])
        car = ind2car(mdp.env, car_i, route, mdp.pos_res, mdp.vel_res)
        ego = ind2ego(mdp.env, ego_i, mdp.pos_res, mdp.vel_res)
        if ped_i > n_ped
            ped = mdp.off_grid
        else
            ped = ind2ped(mdp.env, ped_i, mdp.pos_res, mdp.vel_ped_res)
        end
        collision = crash(mdp, ego, ped, car)
        return PedCarMDPState(collision, ego, ped, car, sroute)
    end
end