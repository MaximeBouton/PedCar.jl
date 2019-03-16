function test_convert(mdp::PedCarMDP)
    @showprogress for i=1:n_states(mdp)
        s = ind2state(mdp, i)
        svec = convert_s(Vector{Float64}, s, mdp)
        s_reconstructed = convert_s(PedCarMDPState, svec, mdp)
        if s_reconstructed ≈ s 
            @debug("index $i, state $s")
            @debug("reconstructed state: $s_reconstructed")
            return false 
        end
    end
    return true 
end

function Base.isapprox(s1::PedCarMDPState, s2::PedCarMDPState, kwargs...)
    return isapprox(s1.crash, s2.crash, kwargs...) &&
           isapprox(s1.ego, s2.ego, kwargs...) &&
           isapprox(s1.car, s2.car, kwargs...) &&
           isapprox(s1.ped, s2.ped, kwargs...) &&
           s1.route == s2.route
end

@testset begin "convert_s"
    params = UrbanParams(nlanes_main=1,
                        crosswalk_pos =[VecSE2(6, 0., pi/2), VecSE2(-6, 0., pi/2), VecSE2(0., -5., 0.)],
                        crosswalk_length =  [14.0, 14., 14.0],
                        crosswalk_width = [4.0, 4.0, 3.1],
                        stop_line = 22.0)
    env = UrbanEnv(params=params)
    mdp = PedCarMDP(env=env, pos_res=4.0, vel_res=3., ped_birth=0.3, car_birth=0.3)

    @test test_convert(mdp)
end