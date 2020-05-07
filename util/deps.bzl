load("@bazel_tools//tools/build_defs/repo:git.bzl", "git_repository")

def planner_uct_rules_dependencies():
    _maybe(
    git_repository,
    name = "bark_project",
    commit="415bfedadc5b2f2c6e8893af9a80097c309de249",
    remote = "https://github.com/bark-simulator/bark",
    )

    _maybe(
    git_repository,
    name = "mamcts_project",
    commit="55d3478b954287fa360329cc8320855894b02ce5",
    remote = "https://github.com/juloberno/mamcts",
    )

def _maybe(repo_rule, name, **kwargs):
    if name not in native.existing_rules():
        repo_rule(name = name, **kwargs)