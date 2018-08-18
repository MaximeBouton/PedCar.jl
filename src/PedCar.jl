module PedCar

#=
Computationally efficient implementation of an autonomous driving scenario modeled as an MDP
This problem involves three agents:
    - an ego vehicle
    - an other car
    - a pedestrian 
=#

using StaticArrays
using GridInterpolations
using POMDPs
using POMDPToolbox
using ParticleFilters
using AutomotiveDrivingModels
using AutoViz
using AutomotivePOMDPs
using AutomotiveSensors
using Parameters

export
    PedCarMDP,
    PedCarMDPState,
    PedCarMDPAction,
    init_car_grid,
    init_ped_grid,
    init_l_grid,
    init_v_grid,
    car_transition,
    car_interpolation,
    car_reset,
    ped_transition,
    ped_interpolation,
    ped_reset,
    ego_transition,
    crash,
    ind2state,
    init_transition!,
    init_collision_checker!,
    animate_states,
    state2scene,
    get_mdp_state

include("mdp_type.jl")
include("spaces.jl")
include("interpolation.jl")
include("driver_models_helpers.jl")
include("transition.jl")
include("helpers.jl")
include("reward.jl")
include("render_helpers.jl")

export
    init_car_belief,
    init_ped_belief,
    ParticleOverlay,
    CarState,
    CarStateEstimator,
    PedStateEstimator

include("belief_updater.jl")

end # module
