// Copyright (c) 2019 Julian Bernhard
// 
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.
// ========================================================

#ifndef DOMAIN_BASED_HEURISTIC_H
#define DOMAIN_BASED_HEURISTIC_H

#include "mcts/mcts.h"
#include <iostream>
#include <chrono>

 namespace mcts {
// assumes all agents have equal number of actions and the same node statistic
class DomainHeuristic :  public mcts::Heuristic<DomainHeuristic>
{
public:
    DomainHeuristic(const MctsParameters& mcts_parameters) :
            mcts::Heuristic<DomainHeuristic>(mcts_parameters) {}

    template<class S, class SE, class SO, class H>
    std::pair<SE, std::unordered_map<AgentIdx, SO>> calculate_heuristic_values(const std::shared_ptr<StageNode<S,SE,SO,H>> &node) {
        //catch case where newly expanded state is terminal
        if(node->get_state()->is_terminal()){
            const auto ego_agent_idx = node->get_state()->get_ego_agent_idx();
            const ActionIdx num_ego_actions = node->get_state()->get_num_actions(ego_agent_idx); 
            SE ego_heuristic(num_ego_actions, node->get_state()->get_ego_agent_idx(), mcts_parameters_);
            ego_heuristic.set_heuristic_estimate(0.0f, 0.0f);
            std::unordered_map<AgentIdx, SO> other_heuristic_estimates;
            for (const auto& ai : node->get_state()->get_other_agent_idx())
            { 
              SO statistic(node->get_state()->get_num_actions(ai), ai, mcts_parameters_);
              statistic.set_heuristic_estimate(0.0f, 0.0f);
              other_heuristic_estimates.insert(std::pair<AgentIdx, SO>(ai, statistic));
            }
            return std::pair<SE, std::unordered_map<AgentIdx, SO>>(ego_heuristic, other_heuristic_estimates) ;
        }
        

        // generate an extra node statistic for each agent
        SE ego_heuristic(0, node->get_state()->get_ego_agent_idx(), mcts_parameters_);
        Reward ego_all_reward = 1/(node->get_state()->get_distance_to_goal() + 0.01)
        ego_heuristic.set_heuristic_estimate(ego_all_reward, accum_cost);
        std::unordered_map<AgentIdx, SO> other_heuristic_estimates;
        AgentIdx reward_idx=1;
        for (auto agent_idx : node->get_state()->get_other_agent_idx())
        {
            SO statistic(0, agent_idx, mcts_parameters_);
            statistic.set_heuristic_estimate(other_accum_rewards[agent_idx], accum_cost);
            other_heuristic_estimates.insert(std::pair<AgentIdx, SO>(agent_idx, statistic));
            reward_idx++;
        }
        return std::pair<SE, std::unordered_map<AgentIdx, SO>>(ego_heuristic, other_heuristic_estimates);
    }

};

 } // namespace mcts

#endif