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
using AutomotiveDrivingModels
using AutomotivePOMDPs
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
    init_collision_checker!

include("mdp_type.jl")
include("spaces.jl")
include("interpolation.jl")
include("driver_models_helpers.jl")
include("transition.jl")
include("helpers.jl")
include("reward.jl")

end # module
