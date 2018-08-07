
function POMDPs.transition(mdp::PedCarMDP, s::PedCarMDPState, a::PedCarMDPAction)
    ego_dist = mdp._ego_transition_dict[(s.ego, LonAccelDirection(a.acc, 4))]
    if s.car == mdp.off_grid
        act_car = LonAccelDirection(0., 0)
    else
        set_car_model!(mdp, s, a)
        a_lon = round(mdp.car_models[s.route].a.a_lon)
        if a_lon == 0.0; a_lon = 0.0; end # in case it rounds to -0.0
        act_car = LonAccelDirection(a_lon, mdp.car_models[s.route].a.direction)
    end
    car_dist = mdp._car_transition_dict[(s.car, s.route, act_car)]
    ped_dist = mdp._ped_transition_dict[(s.ped, ConstantSpeedDawdling(0., 0))]
    # merge the distribution
    n_next = length(ego_dist.vals)*length(car_dist.vals)*length(ped_dist.vals)
    states = Vector{PedCarMDPState}(n_next)
    probs = Vector{Float64}(n_next)
    l = 1
    for (ego, pego) in weighted_iterator(ego_dist)
        for (car_route, pcar) in weighted_iterator(car_dist)
            car, route = car_route
            for (ped, pped) in weighted_iterator(ped_dist)
                collision = crash(mdp, ego, ped, car)
                # collision = false
                # collision = mdp._collision_checker[(ego, ped, car)]
                states[l] = PedCarMDPState(collision, ego, ped, car, route)
                probs[l] = pego*pcar*pped
                l += 1
            end
        end
    end
    return SparseCat(states, probs)::SparseCat{Vector{PedCarMDPState}, Vector{Float64}}
end

function init_transition!(mdp::PedCarMDP)
    mdp._ego_transition_dict = ego_transition(mdp)
    mdp._car_transition_dict = car_transition(mdp)
    mdp._ped_transition_dict = ped_transition(mdp)
end

using ProgressMeter
function init_collision_checker!(mdp::PedCarMDP)
    @showprogress for i=1:n_states(mdp)
        s = ind2state(mdp, i)
        mdp._collision_checker[(s.ego, s.ped, s.car)] = s.crash 
    end
end

function ego_transition(mdp::PedCarMDP)
    env = mdp.env
    pos_res = mdp.pos_res
    vel_res = mdp.vel_res 
    ego_states = get_ego_states(env, pos_res, vel_res)
    ego_transition_dict = EgoTransitionDict()
    for ego in ego_states 
        for a in actions(mdp)
            act = LonAccelDirection(a.acc, 4)
            ego_p = propagate(ego, act, env.roadway, mdp.ΔT)
            ego_states, ego_probs = ego_interpolation(mdp, mdp._car_grid, ego_p)
            ego_transition_dict[(ego, act)] = SparseCat(ego_states, ego_probs)
        end
    end
    return ego_transition_dict
end

function car_transition(mdp::PedCarMDP)
    env = mdp.env
    pos_res = mdp.pos_res
    vel_res = mdp.vel_res 
    car_action_space = mdp.car_action_space
    routes = get_car_routes(env)
    car_transition_dict = CarTransitionDict()
    for route in routes 
        car_route = CarRoute(route[1], route[end])
        car_states = get_car_states(env, route, pos_res, vel_res)
        for car in car_states 
            for a in car_action_space
                car_states = Vector{Tuple{VehicleState, CarRoute}}()
                car_probs = Vector{Float64}()
                k = 1
                lane = get_lane(env.roadway, car)
                dir = get_direction(lane, [env.roadway[l] for l in route])
                acts_car = SVector(LonAccelDirection(a - mdp.a_noise, dir), LonAccelDirection(a, dir), LonAccelDirection(a + mdp.a_noise, dir))
                for act in acts_car
                    car_p = propagate(car, act, env.roadway, mdp.ΔT)
                    itp_car_ps, itp_car_weights = car_interpolation(mdp, mdp._car_grid, car_p)
                    for (j, car_pss) in enumerate(itp_car_ps)
                        index_itp_state = find(x -> x==(car_pss, car_route), car_states)
                        if !(itp_car_weights[j] ≈ 0.)
                            if isempty(index_itp_state)
                                push!(car_states, (car_pss, car_route))
                                push!(car_probs, itp_car_weights[j]*1/length(acts_car))
                            else
                                car_probs[index_itp_state] += itp_car_weights[j]*1/length(acts_car)
                            end
                        end
                    end
                end
                normalize!(car_probs, 1)
                car_transition_dict[(car, car_route, LonAccelDirection(a, dir))] = SparseCat(car_states, car_probs)
            end
        end
        car_transition_dict[(mdp.off_grid, car_route, LonAccelDirection(0., 0))] = car_reset(mdp)
    end # end route
    car_transition_dict[(mdp.off_grid, OFF_ROUTE, LonAccelDirection(0., 0))] = car_reset(mdp)
    return car_transition_dict
end

function ped_transition(mdp::PedCarMDP)
    env = mdp.env 
    pos_res = mdp.pos_res
    vel_res = mdp.vel_ped_res ## not the same vel res 
    ped_action_space = mdp.ped_action_space 
    ped_transition_dict = PedTransitionDict()
    ped_states = get_ped_states(env, pos_res, vel_res)
    for ped in ped_states 
        for a in ped_action_space
            acts_ped = SVector(ConstantSpeedDawdling(0., 0.), ConstantSpeedDawdling(1., 0.), ConstantSpeedDawdling(2., 0.))
            ped_states = Vector{VehicleState}()
            ped_probs = Vector{Float64}()
            k = 1
            p_a = 1/length(acts_ped) # uniform
            for act in acts_ped
                ped_p = propagate(ped, act, env.roadway, mdp.ΔT)
                itp_ped_ps, itp_ped_weights = ped_interpolation(mdp, mdp._ped_grid, ped_p)
                for (j, ped_pss) in enumerate(itp_ped_ps)
                    index_itp_state = find(x -> x==ped_pss, ped_states)
                    if !(itp_ped_weights[j] ≈ 0.)
                        if isempty(index_itp_state)
                            push!(ped_states, ped_pss)
                            push!(ped_probs, itp_ped_weights[j]*p_a)
                        else
                            ped_probs[index_itp_state] += itp_ped_weights[j]*p_a
                        end
                    end
                end
            end
            normalize!(ped_probs, 1) 
            ped_transition_dict[(ped, ConstantSpeedDawdling(0., 0.))] = SparseCat(ped_states, ped_probs)               
        end
    end
    ped_transition_dict[(mdp.off_grid, ConstantSpeedDawdling(0., 0))] = ped_reset(mdp)
    return ped_transition_dict
end


function ego_interpolation(mdp::PedCarMDP, grids::Dict{LaneTag, RectangleGrid{2}}, ego::VehicleState)
    N_itp = 4
    itp_car_ps, itp_car_weights = interpolate_state(mdp, ego)
    # states_ = Vector{VehicleState}(N_itp)
    # probs_ = Vector{Float64}(N_itp)
    # fill!(states_, itp_car_ps[1])
    # fill!(probs_, itp_car_weights[1]/(N_itp - length(itp_car_ps)+1))
    # for i=2:length(itp_car_ps)
    #     states_[i] = itp_car_ps[i]
    #     probs_[i] = itp_car_weights[i]
    # end
    # normalize!(probs_, 1)
    return itp_car_ps, itp_car_weights
end

function car_interpolation(mdp::PedCarMDP, grids::Dict{LaneTag, RectangleGrid{2}}, car::VehicleState)
    N_itp = 4
    if car.posF.s >= get_end(get_lane(mdp.env.roadway, car)) && isempty(get_lane(mdp.env.roadway, car).exits)
        states = SVector{N_itp}(fill(mdp.off_grid, N_itp))
        probs = SVector{N_itp}(fill(1.0/N_itp, N_itp))
        return [mdp.off_grid], [1.0]
        # return SparseCat(states, probs)::SparseCat{SVector{N_itp, VehicleState}, SVector{N_itp, Float64}}
    else
        # interpolate car_p in discrete space
        itp_car_ps, itp_car_weights = interpolate_state(mdp, car)
        @assert length(itp_car_ps) <= N_itp
        states_ = Vector{VehicleState}(N_itp)
        fill!(states_, mdp.off_grid)
        probs_ = zeros(N_itp)
        # can have a varying number of states (between 1 and 4)
        for i=1:length(itp_car_ps)
            states_[i] = itp_car_ps[i]
            probs_[i] = itp_car_weights[i]
        end
        normalize!(probs_, 1)
        return itp_car_ps, itp_car_weights
        # states = SVector(states_...)
        # probs = SVector(probs_...)
        # return SparseCat(states, probs)::SparseCat{SVector{N_itp, VehicleState}, SVector{N_itp, Float64}}
    end
end


function ped_interpolation(mdp::PedCarMDP, grids::Dict{LaneTag, RectangleGrid{3}}, ped::VehicleState)
    N_itp = 4
    if ped.posG == mdp.off_grid.posG || (ped.posF.s >= get_end(get_lane(mdp.env.roadway, ped)) && isapprox(ped.posF.ϕ, 0.)) ||
               (isapprox(ped.posF.s, 0., atol=0.01) && isapprox(ped.posF.ϕ, float(pi)))
        states = SVector{N_itp}(fill(mdp.off_grid, N_itp))
        probs = SVector{N_itp}(fill(1.0/N_itp, N_itp))
        return [mdp.off_grid], [1.0]
        # return SparseCat(states, probs)::SparseCat{SVector{N_itp, VehicleState}, SVector{N_itp, Float64}}
    else
        # interpolate ped_p in discrete space 
        itp_ped_ps, itp_ped_weights = interpolate_pedestrian(mdp, ped)
        @assert length(itp_ped_ps) <= N_itp
        states_ = Vector{VehicleState}(N_itp)
        fill!(states_, mdp.off_grid)
        probs_ = zeros(N_itp)
        # can have a varying number of states (between 1 and 4)
        for i=1:length(itp_ped_ps)
            states_[i] = itp_ped_ps[i]
            probs_[i] = itp_ped_weights[i]
        end
        normalize!(probs_, 1)
        return itp_ped_ps, itp_ped_weights
        # states = SVector(states_...)
        # probs = SVector(probs_...)
        # return SparseCat(states, probs)::SparseCat{SVector{N_itp, VehicleState}, SVector{N_itp, Float64}}
    end
end


# distribution if the car is off the grid
function car_reset(mdp::PedCarMDP, min_speed::Float64 = 6.0)
    # list of car starting states
    routes = get_car_routes(mdp.env)
    v_space = min_speed:mdp.vel_res:mdp.env.params.speed_limit 
    N_states = 0
    for route in routes 
        N_states += length(v_space)
    end
    car_states = Vector{VehicleState}(N_states)
    start_routes = Vector{SVector{2, LaneTag}}(N_states)
    i = 1
    for route in routes
        lane = route[1]
        for v in v_space
            car_states[i] = VehicleState(Frenet(mdp.env.roadway[lane], 0.), mdp.env.roadway, v)
            start_routes[i] = SVector{2, LaneTag}(route[1], route[end])
            i += 1
        end
    end
    car_states[end] = mdp.off_grid #TODO FIX, it is overwriting the last state
    start_routes[end] = OFF_ROUTE
    car_probs = ones(N_states)
    car_probs[end] = 1-mdp.car_birth
    car_probs[1:end-1] = mdp.car_birth/(length(car_states)-1)
    normalize!(car_probs, 1)
    return SparseCat(collect(zip(car_states, start_routes)), car_probs)
end

function ped_reset(mdp::PedCarMDP)
    # list of pedestrian starting states
    n_headings = 2
    lanes = get_ped_lanes(mdp.env)
    v_space = 1.0:mdp.vel_ped_res:2.0
    N_states = length(lanes)*length(v_space)*n_headings
    ped_states = Vector{VehicleState}(N_states)
    i = 1
    for lane in lanes
        dlane = get_discretized_lane(lane, mdp.env.roadway, mdp.pos_res)
        for v in v_space
            ped_states[i] = VehicleState(Frenet(mdp.env.roadway[lane], 0., 0., 0.), mdp.env.roadway, v)
            i += 1
            ped_states[i] = VehicleState(Frenet(mdp.env.roadway[lane], dlane[end], 0., float(pi)), mdp.env.roadway, v)
            i += 1
        end
    end
    ped_states[end] = mdp.off_grid
    ped_probs = ones(N_states)
    ped_probs[end] = 1-mdp.ped_birth
    ped_probs[1:end-1] = mdp.ped_birth/(length(ped_states)-1)
    normalize!(ped_probs, 1)
    return SparseCat(ped_states, ped_probs)
end


function initial_ego_state(mdp::PedCarMDP)
    lanes = AutomotivePOMDPs.get_ego_route(mdp.env)
    posF = Frenet(mdp.env.roadway[lanes[1]], mdp.ego_start)
    v0 = 0.
    return VehicleState(posF, mdp.env.roadway, v0)
end

function initial_car_state_distribution(mdp::PedCarMDP)
    routes = get_car_routes(mdp.env)
    init_car_routes = []
    init_car_states = VehicleState[]
    for route in routes 
        car_states = get_car_states(mdp.env, route, mdp.pos_res, mdp.vel_res)
        for cs in car_states 
            push!(init_car_states, cs)
            push!(init_car_routes, route)
        end
    end
    push!(init_car_states, mdp.off_grid)
    push!(init_car_routes, SVector{2, LaneTag}(LaneTag(0,0), LaneTag(0,0)))
    return init_car_states, init_car_routes
end

function initial_car_state(mdp::PedCarMDP, rng::AbstractRNG)
    init_car_states, init_car_routes = initial_car_state_distribution(mdp)
    return rand(rng, init_car_states), rand(rng, init_car_routes)
end

function initial_ped_state_distribution(mdp::PedCarMDP)
    init_ped_states = get_ped_states(mdp.env, mdp.pos_res, mdp.vel_ped_res)
    push!(init_ped_states, mdp.off_grid)
    # uniform (maybe add more weights to the states when pedestrians are not there?)
    probs = ones(length(init_ped_states))
    normalize!(probs, 1)
    return SparseCat(init_ped_states, probs)
end

function initial_ped_state(mdp::PedCarMDP, rng::AbstractRNG)
    init_dist = initial_ped_state_distribution(mdp)
    return rand(rng, init_dist)
end

function POMDPs.initial_state(mdp::PedCarMDP, rng::AbstractRNG)
    return rand(rng, initial_state_distribution(mdp))
end


function POMDPs.initial_state_distribution(mdp::PedCarMDP)
    ego = initial_ego_state(mdp)
    init_car_states, init_car_routes = initial_car_state_distribution(mdp)
    init_ped_dist = initial_ped_state_distribution(mdp) 
    init_states = Vector{PedCarMDPState}()
    for i=1:length(init_car_states)
        for j=1:length(init_ped_dist.vals)
            ped = init_ped_dist.vals[j]
            car = init_car_states[i]
            route = init_car_routes[i]
            collision = crash(mdp, ego, car, ped)
            push!(init_states, PedCarMDPState(collision, ego, ped, car, SVector{2, LaneTag}(route[1], route[end])))
        end
    end
    # uniform
    probs = ones(length(init_states))
    normalize!(probs, 1)
    return SparseCat(init_states, probs)
end