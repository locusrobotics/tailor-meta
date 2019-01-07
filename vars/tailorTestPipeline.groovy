#!/usr/bin/env groovy
def call(Map args) {
  // TODO(pbovbel) handle package whitelist
  String rosdistro_job = args.get('rosdistro_job')
  String rosdistro_name = args.get('rosdistro_name')
  List<String> distributions = args.get('distributions')
  String release_track = args.get('release_track')
  String flavour = args.get('flavour')
  String tailor_meta_branch = args.get('tailor_meta_branch')
  String repo_main_branch = args.get('repo_main_branch')
  String docker_registry = args.get('docker_registry')

  def docker_credentials = 'ecr:us-east-1:tailor_aws'

  def days_to_keep = 10
  def num_to_keep = 10

  def testImage = { distribution -> docker_registry - "https://" + ':tailor-image-' + distribution + '-test-image' }

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
            library("tailor-meta@$tailor_meta_branch")
            cancelPreviousBuilds()

            // Only build 'master' branch regularly/automatically, release/feature branches require SCM or manual trigger.
            if (env.BRANCH_NAME == repo_main_branch) {
              triggers.add(upstream(upstreamProjects: "$rosdistro_job", threshold: hudson.model.Result.SUCCESS))
            }
            // TODO(pbovbel) detect if we should use a different bundle version? Need a variety of test images.
            // if env.CHANGE_TARGET.startsWith('release/') {
            //   release_track = env.CHANGE_TARGET - 'release/'
            // }

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
                  def colcon_build_args = "--cmake-args -DCMAKE_CXX_FLAGS='-DNDEBUG -g -O3 -fext-numeric-literals' " +
                                          "-DCMAKE_CXX_STANDARD='14' -DCMAKE_CXX_STANDARD_REQUIRED='ON' " +
                                          "-DCMAKE_CXX_EXTENSIONS='ON' -DCMAKE_CXX_COMPILER_LAUNCHER=ccache"

                  test_image.inside("-v $HOME/tailor/ccache:/ccache") {
                    sh("""#!/bin/bash
                      source /opt/locusrobotics/$release_track/$flavour/$rosdistro_name/setup.bash &&
                      colcon build $colcon_path_args $colcon_build_args &&
                      colcon build --cmake-target tests $colcon_path_args $colcon_build_args || true &&
                      colcon test $colcon_path_args --executor sequential --event-handlers console_direct+
                    """)
                    junit(testResults: 'test_results/**/*.xml', allowEmptyResults: true)
                  }
                } finally {
                  deleteDir()
                  sh('docker image prune -af --filter="until=3h" --filter="label=tailor" || true')
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
