# create a dictionary with all the possible models
function get_car_models(env::UrbanEnv, get_model::Function)
    d = Dict{SVector{2, LaneTag}, DriverModel}()

    # 
    r1 = SVector(LaneTag(1,1), LaneTag(2,1))
    d[r1] = get_model(env, r1)

    r2 = SVector(LaneTag(1,1), LaneTag(5,1))
    d[r2] = get_model(env, r2)

    r3 = SVector(LaneTag(3,1), LaneTag(4,1))
    d[r3] = get_model(env, r3)
    
    r4 = SVector(LaneTag(3,1), LaneTag(5,1)) 
    d[r4] = get_model(env, r4)

    return d
end

function set_car_model!(mdp::PedCarMDP, s::PedCarMDPState, a::PedCarMDPAction)
    reset_hidden_state!(mdp.car_models[s.route])
    observe!(mdp.car_models[s.route], s.ego, s.car, s.ped, mdp.env.roadway)
end

function AutomotiveDrivingModels.observe!(model::UrbanDriver, 
                                          ego::VehicleState, 
                                          car::VehicleState, 
                                          ped::VehicleState,
                                          roadway::Roadway)
    AutomotiveDrivingModels.observe!(model.navigator, ego, car, roadway)
    AutomotiveDrivingModels.observe!(model.intersection_driver, ego, car, roadway)
    for driver in model.crosswalk_drivers
        AutomotiveDrivingModels.observe!(driver, ego, car, ped, roadway)
    end
    a_lon_crosswalks = minimum([driver.a.a_lon for driver in model.crosswalk_drivers])
    a_lon = min(model.intersection_driver.a.a_lon, min(a_lon_crosswalks, model.navigator.a))

    model.a = LonAccelDirection(a_lon, model.navigator.dir)
    return model.a
end


# function AutomotiveDrivingModels.observe!(model::TTCIntersectionDriver, 
#                                           ego::VehicleState, 
#                                           car::VehicleState, 
#                                           roadway::Roadway)

#     model.priority = ttc_check(model, car, ego, roadway)
#     passed = has_passed(model, car, ego, roadway)
#     a_lon_idm = model.navigator.a
#     right_of_way = model.priorities[(model.navigator.route[1].tag,model.navigator.route[end].tag)]
#     if !model.priority && !passed && engaged(model, car, ego, roadway) # emergency break
#         a_lon = -model.navigator.d_max
#     elseif !model.priority && !passed
#         a_lon = min(a_lon_idm, AutomotivePOMDPs.stop_at_end(model, car, roadway))
#     else
#         a_lon = a_lon_idm
#     end
#     # to get out of the Emergency break 
#     if !model.priority && !passed && engaged(model, car, ego, roadway) && right_of_way
#         a_lon = a_lon_idm 
#     end
#     model.a = LonAccelDirection(a_lon, model.navigator.dir)    
# end

function AutomotiveDrivingModels.observe!(model::TTCIntersectionDriver, 
                                          ego::VehicleState, 
                                          car::VehicleState, 
                                          roadway::Roadway)
    a_lon_idm = model.navigator.a
    passed =  has_passed(model, car, ego, roadway)
    is_engaged = engaged(model, car, ego, roadway)
    right_of_way = model.priorities[(model.navigator.route[1].tag,model.navigator.route[end].tag)]
    is_clogged = is_intersection_clogged(model, car, ego, roadway)
    ttc = ttc_check(model, car, ego, roadway)
     
    if isempty(model.intersection) || passed 
        a_lon = a_lon_idm 
    elseif !passed 
        if right_of_way
            if is_clogged && !passed && is_engaged && !model.stop && !isapprox(car.v, 0.)
                # println("Vehicle $egoid : emergency break")
                a_lon = -model.navigator.d_max
            else
                a_lon = a_lon_idm
            end
        else # left turn
            if !ttc && !is_engaged  # before left turn
                a_lon = min(a_lon_idm, AutomotivePOMDPs.stop_at_end(model, car, roadway))
            elseif is_clogged && !passed && is_engaged && !isapprox(car.v, 0.) #!ttc && !passed && is_engaged || (is_clogged && is_engaged)
                # println("Vehicle $egoid : emergency break")
                a_lon = -model.navigator.d_max
            elseif ttc 
                a_lon = a_lon_idm 
            end
        end
    end
end

function AutomotiveDrivingModels.observe!(model::CrosswalkDriver, 
                                          ego::VehicleState, 
                                          car::VehicleState,
                                          ped::VehicleState,
                                          roadway::Roadway)
    a_lon =0.
    a_lon_idm = model.navigator.a_max
    if !model.yield 
        a_lon = a_lon_idm
    else
        model.priority = !is_crossing(ped, model.crosswalk, model.conflict_lanes, roadway) || 
        AutomotivePOMDPs.has_passed(model, car, roadway)
        if !model.priority 
            dist_to_cw = AutomotivePOMDPs.get_distance_to_crosswalk(model, car, roadway, -model.stop_delta)
            a_lon = min(a_lon_idm, AutomotivePOMDPs.stop_at_dist(model, car, dist_to_cw))
            # println("dist to cw $dist_to_cw, crosswalk $(model.crosswalk.tag), $(get_lane(roadway, car) ∈ model.intersection_entrances)")
            # println("priority $(model.priority), crossing $(is_crossing(ped, model.crosswalk, model.conflict_lanes, roadway)), a_lon $a_lon, v $(car.v)")
        else
            a_lon = a_lon_idm # just idm
        end
    end
    model.a = LonAccelDirection(a_lon, model.navigator.dir)
end

function AutomotiveDrivingModels.observe!(model::RouteFollowingIDM, 
                                          ego::VehicleState, 
                                          car::VehicleState,
                                          roadway::Roadway)
    Δv = model.v_des - car.v
    acc = Δv*model.k_spd
    model.a = clamp(acc, -model.d_max, model.a_max)
    cur_lane = get_lane(roadway, car)
    set_direction!(model, cur_lane, roadway)    
    return model
end


function is_crossing(ped::VehicleState, crosswalk::Lane, conflict_lanes::Vector{Lane}, roadway::Roadway)
    ped_lane = get_lane(roadway, ped)
    if ped_lane.tag != crosswalk.tag
        return false 
    end
    if 4.0 <= ped.posF.s <= 10. 
        return true
    end
    # at this point, the pedestrian is not on the road
    # check if the pedestrian is going to cross or not
    if AutomotivePOMDPs.direction_from_center(ped, crosswalk) > 0. && get_lane(roadway, ped).tag == crosswalk.tag
        return true
    end
    return false
end

function ttc_check(model::TTCIntersectionDriver, ego::VehicleState, car::VehicleState, roadway::Roadway)
    inter_width=6.0
    posF = car.posF
    int_x, int_y, int_θ = model.intersection_pos
    Δ = (model.intersection_pos.x - car.posG.x)^2 + (model.intersection_pos.y - car.posG.y)^2
    if Δ < inter_width^2 # vehicle is in the middle
        ttc = 0.
    else
        ttc = sqrt(Δ)/car.v
    end

    if 0 <= ttc < model.ttc_threshold
        return false
    else
        return true
    end

end

function is_intersection_clogged(model::TTCIntersectionDriver, ego::VehicleState, car::VehicleState, roadway::Roadway)
    inter_width=5.0
    posF = car.posF
    int_x, int_y, int_θ = model.intersection_pos
    Δ = (model.intersection_pos.x - car.posG.x)^2 + (model.intersection_pos.y - car.posG.y)^2
    return Δ < inter_width^2 # vehicle is in the middle
end


function has_passed(model::TTCIntersectionDriver, ego::VehicleState, car::VehicleState, roadway::Roadway)
    lane = get_lane(roadway, ego)
    inter_to_car = ego.posG - model.intersection_pos
    car_vec = ego.posG + polar(VehicleDef().length/2, ego.posG.θ) - ego.posG
    has_passed = dot(inter_to_car, car_vec) > 0. || isempty(lane.exits)
    return has_passed
end

function engaged(model::TTCIntersectionDriver, ego::VehicleState, car::VehicleState, roadway::Roadway)
    inter_width = 7.5 #todo parameterized
    if normsquared(VecE2(model.intersection_pos - ego.posG)) < inter_width^2
        return true 
    end
    return false 
end

function is_neighbor_fore_along_lane(ego::VehicleState, car::VehicleState, roadway::Roadway, ego_def::VehicleDef = VehicleDef())
    tag_start = ego.posF.roadind.tag
    s_base = ego.posF.s + ego_def.length/2*cos(ego.posF.ϕ)
    tag_start = ego.posF.roadind.tag
    targetpoint_primary = VehicleTargetPointRear()
    targetpoint_valid = VehicleTargetPointFront()

    is_neighbor_fore_along_lane(ego, car, roadway, tag_start, s_base)
end


function is_neighbor_fore_along_lane(
    ego::VehicleState,
    veh::VehicleState,
    roadway::Roadway,
    tag_start::LaneTag,
    s_base::Float64;
    max_distance_fore::Float64 = 250.0, # max distance to search forward [m]
    )

    best_ind = 0
    best_dist = max_distance_fore
    tag_target = tag_start
    lane = roadway[tag_target]

    dist_searched = 0.0
    while dist_searched < max_distance_fore

        s_adjust = NaN
        if veh.posF.roadind.tag == tag_target
            s_adjust = 0.0

        elseif is_between_segments_hi(veh.posF.roadind.ind, lane.curve) &&
                is_in_entrances(roadway[tag_target], veh.posF.roadind.tag)

            distance_between_lanes = norm(VecE2(roadway[tag_target].curve[1].pos - roadway[veh.posF.roadind.tag].curve[end].pos))
            s_adjust = -(roadway[veh.posF.roadind.tag].curve[end].s + distance_between_lanes)

        elseif is_between_segments_lo(veh.posF.roadind.ind) &&
                is_in_exits(roadway[tag_target], veh.posF.roadind.tag)

            distance_between_lanes = norm(VecE2(roadway[tag_target].curve[end].pos - roadway[veh.posF.roadind.tag].curve[1].pos))
            s_adjust = roadway[tag_target].curve[end].s + distance_between_lanes
        end

        if !isnan(s_adjust)
            s_valid = veh.posF.s + VehicleDef().length/2*cos(veh.posF.ϕ) + s_adjust
            dist_valid = s_valid - s_base + dist_searched
            if dist_valid ≥ 0.0
                s_primary = veh.posF.s - VehicleDef().length/2*cos(veh.posF.ϕ) + s_adjust
                dist = s_primary - s_base + dist_searched
                if dist < best_dist
                    best_dist = dist
                    best_ind = 1
                end
            end
        end
        if best_ind != 0
            break
        end

        if !has_next(lane) ||
           (tag_target == tag_start && dist_searched != 0.0) # exit after visiting this lane a 2nd time
            break
        end
        
        dist_searched += (lane.curve[end].s - s_base)
        s_base = -norm(VecE2(lane.curve[end].pos - next_lane_point(lane, roadway).pos)) # negative distance between lanes
        tag_target = next_lane(lane, roadway).tag
    end
    NeighborLongitudinalResult(best_ind, best_dist)
end