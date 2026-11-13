# Implementation provenance

The builder implementations were imported without source changes from the
versions specified by `rosdistro/Jenkinsfile` at migration time:

* `locusrobotics/tailor-distro` tag `1.0.10`: `tailor_distro/`,
  `debian_packager/`, `tests/`, and `environment/distro/`.
* `locusrobotics/tailor-image` tag `0.2.19`: `tailor_image/` and
  `environment/image/`.

The Python namespaces and console commands are preserved for compatibility.
They now ship in the single `tailor-meta` distribution. Conflicting Click and
PyYAML dependencies use tailor-meta's existing versions (8.2.1 and 6.0.2).
Subsequent migration changes adapt paths, credentials, and Actions execution.
