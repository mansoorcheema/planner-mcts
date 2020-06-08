// Copyright (c) 2019 fortiss GmbH
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#include "src/behavior_uct_single_agent_macro_actions.hpp"

#include "modules/models/behavior/idm/idm_classic.hpp"
#include "modules/models/behavior/motion_primitives/macro_actions.hpp"
#include "modules/models/behavior/motion_primitives/param_config/behav_macro_actions_from_param_server.hpp"
#include "modules/models/dynamic/single_track.hpp"

namespace modules {
namespace models {
namespace behavior {

using modules::models::behavior::primitives::Primitive;
using modules::models::behavior::primitives::PrimitiveConstAccChangeToLeft;
using modules::models::behavior::primitives::PrimitiveConstAccChangeToRight;
using modules::models::behavior::primitives::PrimitiveConstAccStayLane;
using modules::models::behavior::primitives::PrimitiveGapKeeping;
using modules::models::dynamic::Input;
using modules::models::dynamic::SingleTrackModel;
using modules::world::prediction::PredictionSettings;

PredictionSettings BehaviorUCTSingleAgentMacroActions::SetupPredictionSettings(
    const commons::ParamsPtr& params) {
  // Setup prediction models for ego agent and other agents
  auto prediction_params_ego = params->AddChild("EgoVehicle");
  BehaviorModelPtr ego_prediction_model(BehaviorMacroActionsFromParamServer(prediction_params_ego));
      
  auto prediction_params_other = params->AddChild("OtherVehicles");
  BehaviorModelPtr others_prediction_model(new BehaviorIDMClassic(prediction_params_other));
  return PredictionSettings(ego_prediction_model, others_prediction_model);
}

}  // namespace behavior
}  // namespace models
}  // namespace modules
