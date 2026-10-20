# tailor-meta

## rosdistro pull requests

The `tailorDistroPipeline` shared-library step invokes tailor-distro for every
build type. TRIVIAL builds (including pull requests) pass `graph_only=true` and
`deploy=false`; image, meta, and integration-test sub-pipelines remain disabled.
The rosdistro checkout is archived for the downstream job but is not uploaded
to S3.

Graph-only builds use the pull request's target release track and label:
`release/<track>` uses `<track>` / `<track>-rc`, and `feature/<name>` uses
`hotdog` / `feature-<name>` (lowercase). Other targets, or branches without pull
request metadata, use `hotdog` for both.

Both the tailor-meta library version and the tailor-distro job version pinned
in the rosdistro Jenkinsfile must include graph-only support before enabling
this workflow.
