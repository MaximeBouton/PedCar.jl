using StaticArrays
using GridInterpolations
using POMDPs
using POMDPToolbox
using AutomotiveDrivingModels
using AutomotivePOMDPs
using PedCar



mdp = PedCarMDP()


init_transition!(mdp)
init_collision_checker!(mdp)

rng = MersenneTwister(1)
N_SAMPLES = 10000
ns = n_states(mdp)
action_space = actions(mdp)
sampled_states = [ind2state(mdp, si) for si in rand(rng, 1:ns, N_SAMPLES)]


using ProfileView

Profile.clear()

@time for (i, s) in enumerate(sampled_states)
    @profile d = transition(mdp, s, rand(rng, action_space))
end

s = sampled_states[107]

d = transition(mdp, s, rand(rng, action_space))

mdp._car_transition_dict[(s.car, s.route, LonAccelDirection(-0.0, 1))]

LonAccelDirection(-0.0, 1) == LonAccelDirection(0.0, 1)

