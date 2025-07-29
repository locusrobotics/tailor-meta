#!/usr/bin/env groovy
def call(Map args) {
  // TODO(pbovbel) handle package whitelist
  String rosdistro_job = args.get('rosdistro_job')
  String rosdistro_name = args.get('rosdistro_name')
  List<String> distributions = args.get('distributions')
  String release_track = args.get('release_track')
  String release_label = args.get('release_label')
  String tailor_meta = args.get('tailor_meta')
  String source_branch = args.get('source_branch')
  String docker_registry = args.get('docker_registry')

  def recipes_yaml = 'rosdistro/config/recipes.yaml'
  def rosdistro_index = 'rosdistro/rosdistro/index.yaml'

  def docker_credentials = 'ecr:us-east-1:tailor_aws'

  def days_to_keep = 10
  def num_to_keep = 10

  def testImage = { distribution -> docker_registry - "https://" + ':tailor-image-test-' + distribution + '-' + release_label }
  def dependencyImage = { distribution -> docker_registry - "https://" + ':tailor-image-dep-checker-' + distribution + '-' + release_label }

  pipeline {
    agent none

    options {
      timestamps()
    }

    stages {
      stage("Configure build parameters") {
        agent { label 'master' }
        steps {
          script {
            sh 'env'
            def triggers = []
            library("tailor-meta@$tailor_meta")
            cancelPreviousBuilds()

            // Only build master or release branches automatically, feature branches require SCM or manual trigger.
            if (env.BRANCH_NAME == source_branch) {
              triggers.add(upstream(upstreamProjects: "$rosdistro_job", threshold: hudson.model.Result.SUCCESS))
            }

            properties([
              buildDiscarder(logRotator(
                artifactDaysToKeepStr: days_to_keep.toString(), artifactNumToKeepStr: num_to_keep.toString(),
                daysToKeepStr: days_to_keep.toString(), numToKeepStr: num_to_keep.toString()
              )),
              pipelineTriggers(triggers)
            ])
          }
        }
      }

      stage("Dependency check") {
        agent none
        steps {
          script {
            def jobs = distributions.collectEntries { distribution ->
              [distribution, { node {
                try {
                  def deps_image = docker.image(dependencyImage(distribution))
                  docker.withRegistry(docker_registry, docker_credentials) { deps_image.pull() }

                  deps_image.inside("-v $HOME/tailor/ccache:/ccache") {
                    echo('↓↓↓ DEPS OUTPUT ↓↓↓')
                    withCredentials([string(credentialsId: 'tailor_github', variable: 'GITHUB_TOKEN')]) {
                      sh("""#!/bin/bash
                        set -e
                        source /home/locus/.ros_env.sh

                        if [ "$rosdistro_name" = "ros1" ]; then
                          source "\$ROS1_SOURCE"
                        elif [ "$rosdistro_name" = "ros2" ]; then
                          source "\$ROS2_SOURCE"
                        else
                          echo "Unknown ROS distribution: $rosdistro_name"
                          exit 1
                        fi
                        echo "Pulling rosdistro..."
                        python3 /home/locus/pull_rosdistro.py --src-dir rosdistro --github-key $GITHUB_TOKEN --clean --ref $release_track
                        echo "Pulling distro repositories..."
                        python3 /home/locus/pull_distro_repositories.py --src-dir workspace/src --github-key $GITHUB_TOKEN \
                          --recipes $recipes_yaml --rosdistro-index $rosdistro_index --clean --ref ${BRANCH_NAME} --rosdistro-name $rosdistro_name

                        rosdep check --from-paths workspace/src/$rosdistro_name --ignore-src
                      """)
                      echo('↑↑↑ DEPS OUTPUT ↑↑↑')
                    }
                    echo('↓↓↓ LINTER OUTPUT ↓↓↓')
                      sh("""#!/bin/bash
                        repo_name=\$(echo "$JOB_NAME" | cut -d'/' -f3)
                        echo "Running catkin_lint for repository: workspace/src/$rosdistro_name/\$repo_name"
                        cd workspace/src/$rosdistro_name/\$repo_name
                        catkin_lint . -W2 --ignore unknown-package,order_violation,literal_project_name,shadowed_find
                      """)
                    echo('↑↑↑ LINTER OUTPUT ↑↑↑')
                  }
                } finally {
                  library("tailor-meta@$tailor_meta")
                  cleanDocker()
                  deleteDir()
                }
              }}]
            }
            parallel(jobs)
          }
        }
      }

      stage("Build and test") {
        agent none
        steps {
          script {
            def jobs = distributions.collectEntries { distribution ->
              [distribution, { node {
                try {
                  dir('package') {
                    checkout(scm)
                  }
                  def test_image = docker.image(testImage(distribution))
                  docker.withRegistry(docker_registry, docker_credentials) { test_image.pull() }

                  def colcon_path_args = "--merge-install --base-paths package --test-result-base test_results"

                  // TODO(pbovbel) pull from last rosdistro build artifact? or from s3?
                  def colcon_build_args = "--cmake-args -DCMAKE_CXX_FLAGS='-g -O3 -fext-numeric-literals -march=haswell -DBOOST_BIND_GLOBAL_PLACEHOLDERS -DBOOST_ALLOW_DEPRECATED_HEADERS' " +
                                          "-DCMAKE_CXX_STANDARD='17' -DCMAKE_CXX_STANDARD_REQUIRED='ON' " +
                                          "-DCMAKE_CXX_EXTENSIONS='ON' -DCMAKE_CXX_COMPILER_LAUNCHER=ccache"

                  test_image.inside("-v $HOME/tailor/ccache:/ccache") {
                    echo('↓↓↓ TEST OUTPUT ↓↓↓')
                    sh("""#!/bin/bash
                      source \$BUNDLE_ROOT/$rosdistro_name/setup.bash &&
                      rosdep install --from-paths package --ignore-src -y &&
                      colcon build $colcon_path_args $colcon_build_args &&
                      colcon test $colcon_path_args --executor sequential --event-handlers console_direct+
                    """)
                    echo('↑↑↑ TEST OUTPUT ↑↑↑')
                    junit(testResults: 'test_results/**/*.xml', allowEmptyResults: true)
                  }
                } finally {
                  library("tailor-meta@$tailor_meta")
                  cleanDocker()
                  deleteDir()
                }
              }}]
            }
            parallel(jobs)
          }
        }
      }
    }
  }
}
