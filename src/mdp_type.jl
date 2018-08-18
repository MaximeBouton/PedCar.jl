#=

T intersection
    ________________________________________________________
        <---- seg 2     main street <----- seg 1
        -----> seg 3                 -----> seg 4
    _________________            ___________________________
                    | seg . seg  |
                    |  5  .  6   |
                    |     .      |
                    |     .      |
                    |     .      |




=#

# State type
struct PedCarMDPState
    crash::Bool
    ego::VehicleState
    ped::VehicleState
    car::VehicleState
    route::SVector{2, LaneTag}
end

function Base.:(==)(v1::VehicleState, v2::VehicleState)
    return v1.posG == v2.posG && v1.v == v2.v 
end
Base.hash(veh::VehicleState, h::UInt) = hash(veh.posF, hash(veh.v, h))

# copy b to a
function Base.copy!(a::PedCarMDPState, b::PedCarMDPState)
    a.crash = b.crash
    a.ego = b.ego
    a.ped = b.ped
    a.car = b.car
    a.route = b.route
end

function Base.hash(s::PedCarMDPState, h::UInt64 = zero(UInt64))
    return hash(s.crash, hash(s.ego, hash(s.ped, hash(s.car, hash(s.route, h)))))
end

function Base.:(==)(a::PedCarMDPState, b::PedCarMDPState)
    return a.crash == b.crash && a.ego == b.ego && a.ped == b.ped && a.car == b.car && a.route == b.route
end

# Action type
const PedCarMDPAction = UrbanAction
const CarRoute = SVector{2, LaneTag}
const CarState = Tuple{VehicleState, CarRoute}
const OFF_ROUTE = SVector{2, LaneTag}(LaneTag(0,0), LaneTag(0,0))
const N_NEXT = 12
const N_CAR_START = 12 # for v_res=2.0, had to hardcode for performance
const N_PED_START = 12 # for v_ped_res = 1.0
const CarTransitionDict = Dict{Tuple{VehicleState, CarRoute, LonAccelDirection}, SparseCat{Vector{Tuple{VehicleState, CarRoute}}, Vector{Float64}}}
const PedTransitionDict = Dict{Tuple{VehicleState, ConstantSpeedDawdling}, SparseCat{Vector{VehicleState}, Vector{Float64}}}
const EgoTransitionDict = Dict{Tuple{VehicleState, LonAccelDirection}, SparseCat{Vector{VehicleState}, Vector{Float64}}}

@with_kw mutable struct PedCarMDP <: MDP{PedCarMDPState, PedCarMDPAction}
    env::UrbanEnv = UrbanEnv(params=UrbanParams(nlanes_main=1,
                crosswalk_pos =  [VecSE2(6, 0., pi/2), VecSE2(-6, 0., pi/2), VecSE2(0., -5., 0.)],
                crosswalk_length =  [14.0, 14., 14.0],
                crosswalk_width = [4.0, 4.0, 3.1],
                stop_line = 22.0))
    ΔT::Float64 = 0.5
    pos_res::Float64 = 2.0
    vel_res::Float64 = 2.0
    vel_ped_res::Float64 = 1.0
    car_action_space::Vector{Float64} = collect(-9.0:1.0:4.0)
    ped_action_space::Vector{Float64} = collect(0.0:1.0:2.0)
    car_models::Dict{SVector{2, LaneTag}, DriverModel} = get_car_models(env, get_ttc_model)
    car_type::VehicleDef = VehicleDef()
    ego_type::VehicleDef = VehicleDef()
    ped_type::VehicleDef = VehicleDef(AgentClass.PEDESTRIAN, 1.0, 1.0)
    a_noise::Float64 = 1.0
    v_noise::Float64 = 1.0
    ped_birth::Float64 = 0.7
    car_birth::Float64 = 0.7
    ego_start::Float64 = env.params.stop_line - ego_type.length/2
    ego_goal::LaneTag = LaneTag(2,1)
    off_grid::VehicleState = VehicleState(VecSE2(UrbanEnv().params.x_min+VehicleDef().length/2, env.params.y_min+VehicleDef().width/2, 0), Frenet(env.roadway[LaneTag(5,1)], 25.1, -26.5, pi/2), 0.)
    collision_cost::Float64 = -1.0
    action_cost::Float64 = 0.
    goal_reward::Float64 = 1.
    γ::Float64 = 0.95
    _ped_grid::Dict{LaneTag, RectangleGrid{3}} = init_ped_grid(env, pos_res, vel_ped_res)
    _car_grid::Dict{LaneTag, RectangleGrid{2}} = init_car_grid(env, pos_res, vel_res)
    _l_grid::Dict{LaneTag, RectangleGrid{1}} = init_l_grid(env, pos_res)
    _v_grid::RectangleGrid{1} = init_v_grid(env, vel_res)
    _car_transition_dict::CarTransitionDict = CarTransitionDict()
    _ped_transition_dict::PedTransitionDict = PedTransitionDict()
    _ego_transition_dict::EgoTransitionDict = EgoTransitionDict()
    _collision_checker::Dict{Tuple{Vararg{VehicleState, 3}}, Bool} = Dict{Tuple{Vararg{VehicleState, 3}}, Bool}()
end