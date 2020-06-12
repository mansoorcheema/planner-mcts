package(default_visibility = ["//visibility:public"])

cc_library(
  name = "planner_uct",
  srcs = ["python_planner_uct.cpp"],
  hdrs = ["python_planner_uct.hpp"],
  deps = [
    "@bark_project//bark/python_wrapper:common",
    "//src:planner_uct_single_agent",
    "@python_linux//:python-lib",
    "@pybind11//:pybind11",
    "@com_github_eigen_eigen//:eigen", 
  ],
  visibility = ["//visibility:public"],
)


# the following rules are only for testing purposes
cc_binary(
  name = "planner_uct.so",
  srcs = ["python_test_module.cpp"],
  deps = [
    ":planner_uct",
    "@bark_project//bark/python_wrapper:behavior",
    "@bark_project//bark/models/behavior:behavior",
    "@bark_project//bark/models/behavior/motion_primitives:motion_primitives",
    "@bark_project//bark/models/behavior/constant_velocity:constant_velocity",
    "@bark_project//bark/models/behavior/mobil:mobil",
    "@bark_project//bark/world:world",
    "@bark_project//bark/world/evaluation:evaluator_goal_reached",
    "@bark_project//bark/world/evaluation:evaluator_collision_ego_agent",
    "@bark_project//bark/world/evaluation:evaluator_drivable_area",
  ],
  linkshared = 1,
  linkstatic = 1
)

py_test(
  name = "python_import_test",
  srcs = ["python_import_test.py"],
  data = ['//python:planner_uct.so'],
  imports = ['../python/'],
)
