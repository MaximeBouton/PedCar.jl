using PedCar
using Test
using POMDPs
using AutomotiveDrivingModels
using AutomotivePOMDPs
using ProgressMeter

@testset begin "PedCar basics"
    params = UrbanParams(nlanes_main=1,
                     crosswalk_pos =[VecSE2(6, 0., pi/2), VecSE2(-6, 0., pi/2), VecSE2(0., -5., 0.)],
                     crosswalk_length =  [14.0, 14., 14.0],
                     crosswalk_width = [4.0, 4.0, 3.1],
                     stop_line = 22.0)
    env = UrbanEnv(params=params)
    mdp = PedCarMDP(env=env, pos_res=4.0, vel_res=3., ped_birth=0.3, car_birth=0.3)

end

include("test_convert.jl")