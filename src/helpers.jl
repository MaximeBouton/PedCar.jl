function crash(mdp::PedCarMDP, s::PedCarMDPState)
    return crash(mdp, s.ego, s.car, s.ped)
end

function crash(mdp::PedCarMDP, ego::VehicleState, ped::VehicleState, car::VehicleState)
    return collision_checker(ego, car, mdp.ego_type, mdp.car_type) || collision_checker(ego, ped, mdp.ego_type, mdp.ped_type)
end

function POMDPs.discount(mdp::PedCarMDP)
    return mdp.γ
end

function POMDPs.isterminal(mdp::PedCarMDP, s::PedCarMDPState)
    if crash(mdp, s)
        return true
    elseif s.ego.posF.s >= get_end(mdp.env.roadway[mdp.ego_goal]) &&
       get_lane(mdp.env.roadway, s.ego).tag == mdp.ego_goal
       return true
   end
   return false
end

function POMDPs.convert_s(::Type{V}, s::PedCarMDPState, mdp::PedCarMDP) where V<:AbstractArray
    n_routes = 4
    n_features = 4
    z = zeros(n_features*3 + n_routes + 1)
    z[1] = s.ego.posG.x / abs(mdp.env.params.x_min)
    z[2] = s.ego.posG.y / abs(mdp.env.params.y_min)
    z[3] = s.ego.posG.θ / π
    z[4] = s.ego.v / abs(mdp.env.params.speed_limit)
    z[5] = s.ped.posG.x / abs(mdp.env.params.x_min)
    z[6] = s.ped.posG.y / abs(mdp.env.params.y_min)
    z[7] = s.ped.posG.θ / π
    z[8] = s.ped.v / abs(mdp.env.params.speed_limit)
    z[9] = s.car.posG.x / abs(mdp.env.params.x_min)
    z[10] = s.car.posG.y / abs(mdp.env.params.y_min)
    z[11] = s.car.posG.θ / π
    z[12] = s.car.v / abs(mdp.env.params.speed_limit)
    # one hot encoding for the route
    routes = get_car_routes(mdp.env)
    for (i, r) in enumerate(routes)
        if r[1] == s.route[1] && r[end] == s.route[end]
            z[12+i] = 1.
        end
    end
    z[17] = float(s.crash)
    return z
end

function POMDPs.convert_s(::Type{PedCarMDPState}, z::V, mdp::PedCarMDP) where V<:AbstractArray{Float64}
    n_routes = 4
    n_features = 4
    @assert length(z) == n_features*3 + n_routes + 1 
    ego_x = z[1]*abs(mdp.env.params.x_min)
    ego_y = z[2]*abs(mdp.env.params.y_min)
    ego_θ = z[3]*π
    ego_v = z[4]*abs(mdp.env.params.speed_limit)
    ego = VehicleState(VecSE2(ego_x, ego_y, ego_θ), mdp.env.roadway, ego_v)
    ped_x = z[5]*abs(mdp.env.params.x_min)
    ped_y = z[6]*abs(mdp.env.params.y_min)
    ped_θ = z[7]*π
    ped_v = z[8]*abs(mdp.env.params.speed_limit)
    ped = VehicleState(VecSE2(ped_x, ped_y, ped_θ), mdp.env.ped_roadway, ped_v)
    car_x = z[9]*abs(mdp.env.params.x_min)
    car_y = z[10]*abs(mdp.env.params.y_min)
    car_θ = z[11]*π
    car_v = z[12]*abs(mdp.env.params.speed_limit)
    car = VehicleState(VecSE2(car_x, car_y, car_θ), mdp.env.roadway, car_v)
    # one hot encoding for the route
    routes = get_car_routes(mdp.env)
    route = SVector{2, LaneTag}(LaneTag(0, 0), LaneTag(0, 0))
    for (i, r) in enumerate(routes)
        if z[12+i] == 1.
            route = SVector{2, LaneTag}(r[1], r[end])
        end
    end
    collision = Bool(z[17])
    return PedCarMDPState(collision, ego, ped, car, route)
end
