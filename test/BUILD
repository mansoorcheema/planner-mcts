cc_test(
    name = "single_agent_test",
    srcs = [
        "single_agent_test.cc",
    ],
    copts = ["-Iexternal/gtest/include", "-O3"],
    deps = [
        "//src:planner_uct_single_agent",
        "@bark_project//bark/world:world",
        "@bark_project//bark/world/tests:make_test_world",
        "@bark_project//bark/models/behavior:behavior",
        "@bark_project//bark/models/behavior/motion_primitives:motion_primitives",
        "@bark_project//bark/models/behavior/mobil:mobil",
        "@bark_project//bark/models/behavior/constant_velocity:constant_velocity",
        "@bark_project//bark/world/evaluation:evaluator_goal_reached",
        "@bark_project//bark/world/evaluation:evaluator_collision_ego_agent",
        "@bark_project//bark/world/evaluation:evaluator_drivable_area",
        "@bark_project//bark/world/goal_definition:goal_definition",
        "@gtest//:main",
    ],
)

cc_test(
    name = "single_agent_macro_actions_test",
    srcs = [
        "single_agent_macro_actions_test.cc"
    ],
    copts = ["-Iexternal/gtest/include"],
    deps = [
        "//src:planner_uct_single_agent_macro_actions",
        "@bark_project//bark/world:world",
        "@bark_project//bark/world/tests:make_test_world",
        "@bark_project//bark/models/behavior:behavior",
        "@bark_project//bark/models/behavior/motion_primitives:motion_primitives",
        "@bark_project//bark/models/behavior/mobil:mobil",
        "@bark_project//bark/models/behavior/idm:idm_lane_tracking",
        "@bark_project//bark/models/behavior/constant_velocity:constant_velocity",
        "@bark_project//bark/world/evaluation:evaluator_goal_reached",
        "@bark_project//bark/world/evaluation:evaluator_collision_ego_agent",
        "@bark_project//bark/world/evaluation:evaluator_drivable_area",
        "@bark_project//bark/world/goal_definition:goal_definition",
        "@gtest//:main",
    ],
)