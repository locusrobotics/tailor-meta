#!/usr/bin/env groovy
def call(Map args) {
  String rosdistro = args.get('rosdistro')
  List<String> distributions = args.get('distributions')
  String release_track = args.get('release_track')
  String flavour = args.get('flavour')
  String meta_ref = args.get('meta_ref')

  def docker_registry = '084758475884.dkr.ecr.us-east-1.amazonaws.com/tailor-image'
  def docker_registry_uri = 'https://' + docker_registry
  def docker_credentials = 'ecr:us-east-1:tailor_aws'

  def days_to_keep = 10
  def num_to_keep = 10

  def testImage = { distribution -> docker_registry + ':jenkins-' + distribution + '-test-image' }

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
            def triggers = []  // TODO(pbovbel) trigger from tailor-distro/tailor-image builds?
            // cancelPreviou  sBuilds()

            if (env.BRANCH_NAME == 'master') {
              // Build master branch daily
              triggers.add(cron('H H * * *'))
            }
            triggers.add(upstream(upstreamProjects: "../tailor-images/$meta_ref", threshold: hudson.model.Result.SUCCESS))
            triggers.add(upstream(upstreamProjects: "../tailor-meta/$meta_ref", threshold: hudson.model.Result.SUCCESS))
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
        agent any
        steps {
          script {
            dir('package') {
              checkout(scm)
            }

            // TODO(pbovbel) parallel tests across distributions
            def distribution = 'xenial'
            def test_image = docker.image(testImage(distribution))
            docker.withRegistry(docker_registry_uri, docker_credentials) { test_image.pull() }

            def colcon_path_args = "--base-paths package --test-result-base test_results"

            // TODO(pbovbel) pull from rosdistro
            def colcon_build_args = "--cmake-args -DCMAKE_CXX_FLAGS='-DNDEBUG -g -fext-numeric-literals' " +
                                    "-DCMAKE_CXX_STANDARD='14' -DCMAKE_CXX_STANDARD_REQUIRED='ON' " +
                                    "-DCMAKE_CXX_EXTENSIONS='ON' -DCMAKE_CXX_COMPILER_LAUNCHER=ccache"

            test_image.inside("-v $HOME/tailor/ccache:/ccache") {
              sh("""#!/bin/bash
                source /opt/locusrobotics/$release_track/$flavour/$rosdistro/setup.bash &&
                colcon build $colcon_path_args $colcon_build_args &&
                colcon build --cmake-target tests $colcon_path_args $colcon_build_args &&
                colcon test $colcon_path_args
              """)
              junit(testResults: 'test_results/**/*.xml')
            }
          }
        }
        post {
          cleanup {
            deleteDir()
            sh('docker image prune -af --filter="until=3h" --filter="label=tailor" || true')
          }
        }
      }
    }
  }
}
