using POMDPs
using AutomotiveDrivingModels
using AutomotivePOMDPs
using PedCar
using ProgressMeter

function test_convert(mdp::PedCarMDP)
    @showprogress for i=1:n_states(mdp)
        s = ind2state(mdp, i)
        svec = convert_s(Vector{Float64}, s, mdp)
        s_reconstructed = convert_s(PedCarMDPState, svec, mdp)
        if s_reconstructed != s 
            println("index $i, state $s")
            println("reconstructed state: $s_reconstructed")
            return false 
        end
    end
    return true 
end

params = UrbanParams(nlanes_main=1,
                     crosswalk_pos =[VecSE2(6, 0., pi/2), VecSE2(-6, 0., pi/2), VecSE2(0., -5., 0.)],
                     crosswalk_length =  [14.0, 14., 14.0],
                     crosswalk_width = [4.0, 4.0, 3.1],
                     stop_line = 22.0)
env = UrbanEnv(params=params)
mdp = PedCarMDP(env=env, pos_res=2.0, vel_res=2., ped_birth=0.3, car_birth=0.3)



test_convert(mdp)