function POMDPs.reward(mdp::PedCarMDP, s::PedCarMDPState, a::PedCarMDPAction, sp::PedCarMDPState)
    r = mdp.action_cost
    ego = sp.ego
    collision = crash(mdp, ego, sp.ped, sp.car)
    if collision 
        r += mdp.collision_cost
    end
    if ego.posF.s >= get_end(mdp.env.roadway[mdp.ego_goal]) &&
       get_lane(mdp.env.roadway, ego).tag == mdp.ego_goal
        r += mdp.goal_reward
    end
    return r
end
