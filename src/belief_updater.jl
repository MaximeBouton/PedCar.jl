abstract type StateEstimator end

mutable struct CarStateEstimator{S <: AbstractSensor} <: StateEstimator
    mdp::PedCarMDP
    id::Int64
    ego::Vehicle
    sensor::S
end

function ParticleFilters.generate_s(model::CarStateEstimator, s::CarState, rng::AbstractRNG)
    car, route = s
    if  car == model.mdp.off_grid
        act_car = LonAccelDirection(0., 0)
    else
        cur_lane = get_lane(model.mdp.env.roadway, car)
        set_direction!(model.mdp.car_models[route].navigator, cur_lane, model.mdp.env.roadway)  
        act_car = LonAccelDirection(0.0, model.mdp.car_models[route].navigator.dir)
    end
    d = model.mdp._car_transition_dict[(car, route, act_car)]
    return rand(rng, d)
end


function ParticleFilters.obs_weight(model::CarStateEstimator, s::CarState, sp::CarState, o::Vector{Vehicle})
    car, route = sp
    car_ind = findfirst(x->x.id==model.id, o)
    if car_ind == 0
        obs = nothing
    else
        obs = o[car_ind].state
    end
    if car == model.mdp.off_grid
        weight = AutomotiveSensors.obs_weight(model.sensor, model.ego.state, obs, nothing, model.mdp.env)
    else
        weight = AutomotiveSensors.obs_weight(model.sensor, model.ego.state, obs, car, model.mdp.env)
    end
end



function init_car_belief(mdp::PedCarMDP, n_particles::Int64, rng::AbstractRNG)
    routes = get_car_routes(mdp.env)
    sampled_full_routes = rand(rng, routes, n_particles)
    sampled_states = [rand(rng, get_car_states(mdp.env, route, mdp.pos_res, mdp.vel_res)) for route in sampled_full_routes]
    sampled_routes = [SVector(route[1], route[end]) for route in sampled_full_routes]
    #TODO sample off the grid states
    return ParticleCollection(collect(zip(sampled_states, sampled_routes)))
end

mutable struct PedStateEstimator{S <: AbstractSensor} <: StateEstimator
    mdp::PedCarMDP
    id::Int64
    ego::Vehicle
    sensor::S
end

function ParticleFilters.generate_s(model::PedStateEstimator, ped::VehicleState, rng::AbstractRNG)    
    d = model.mdp._ped_transition_dict[(ped, ConstantSpeedDawdling(0., 0))]
    return rand(rng, d)
end

function ParticleFilters.obs_weight(model::PedStateEstimator, ped::VehicleState, pedp::VehicleState, o::Vector{Vehicle})
    ped_ind = findfirst(x->x.id==model.id, o)
    if ped_ind == 0
        obs = nothing
    else
        obs = o[ped_ind].state
    end
    if ped == model.mdp.off_grid
        weight = AutomotiveSensors.obs_weight(model.sensor, model.ego.state, obs, nothing, model.mdp.env)
    else
        weight = AutomotiveSensors.obs_weight(model.sensor, model.ego.state, obs, ped, model.mdp.env)
    end
end

function init_ped_belief(mdp::PedCarMDP, n_particles::Int64, rng::AbstractRNG)
    sampled_states = rand(rng, get_ped_states(mdp.env, mdp.pos_res, mdp.vel_ped_res), n_particles)
    return ParticleCollection(sampled_states)
end

function ParticleFilters.update(up::SimpleParticleFilter, b::ParticleCollection{S}, o::Vector{Vehicle}) where S <: Union{CarState, VehicleState}
    ps = particles(b)
    pm = up._particle_memory
    wm = up._weight_memory
    resize!(pm, 0)
    resize!(wm, 0)
    sizehint!(pm, n_particles(b))
    sizehint!(wm, n_particles(b))
    all_terminal = true
    for i in 1:n_particles(b)
        s = ps[i]
        sp = ParticleFilters.generate_s(up.model, s, up.rng)
        push!(pm, sp)
        push!(wm, ParticleFilters.obs_weight(up.model, s, sp, o))
    end
    return resample(up.resample, WeightedParticleBelief{S}(pm, wm, sum(wm), nothing), up.rng)
end


@with_kw mutable struct ParticleOverlay{S <: Union{CarState, VehicleState}} <: SceneOverlay
    mdp::PedCarMDP = PedCarMDP()
    b::ParticleCollection{S} = ParticleCollection(Vector{S}())
    n_render::Int64 = 100
    rng::AbstractRNG = MersenneTwister(1)
    color::Colorant = RGBA(0.976, 0.592, 0.122, 0.15) # orange
end

function AutoViz.render!(rendermodel::RenderModel, overlay::ParticleOverlay{CarState}, scene::Scene, roadway::R) where R
    for i=1:overlay.n_render
        car, route = rand(overlay.rng, overlay.b)
        p = car.posG
        length = overlay.mdp.car_type.length
        width = overlay.mdp.car_type.width
        add_instruction!(rendermodel, render_vehicle, (p.x, p.y, p.θ, length, width, overlay.color, overlay.color, overlay.color))
    end
    overlay.rng = MersenneTwister(1) # reset rng
    return rendermodel
end

function AutoViz.render!(rendermodel::RenderModel, overlay::ParticleOverlay{VehicleState}, scene::Scene, roadway::R) where R
    for i=1:overlay.n_render
        ped = rand(overlay.rng, overlay.b)
        p = ped.posG
        length = overlay.mdp.ped_type.length
        width = overlay.mdp.ped_type.width
        add_instruction!(rendermodel, render_vehicle, (p.x, p.y, p.θ, length, width, overlay.color, overlay.color, overlay.color))
    end
    overlay.rng = MersenneTwister(1) # reset rng
    return rendermodel
end