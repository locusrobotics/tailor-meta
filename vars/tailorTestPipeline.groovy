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
  def integration_tests_branch = 'main'
  def pr_comment_cause = 'com.adobe.jenkins.github_pr_comment_build.GitHubPullRequestCommentCause'

  pipeline {
    agent none

    options {
      timestamps()
    }

    stages {
      stage("Trigger PR integration tests"){
        agent none
        when {
          expression { currentBuild.getBuildCauses(pr_comment_cause) }
        }
        steps{
          script{
            node {
              def repository = checkout(scm)
              def sha = repository.GIT_COMMIT

              def comment_trigger = currentBuild.getBuildCauses(pr_comment_cause)
              def comment_body = (comment_trigger && comment_trigger.size() > 0) ? (comment_trigger[0].commentBody ?: "") : ""

              build job: "ci_integration_tests/$integration_tests_branch",
                wait: false,
                propagate: false,
                parameters: [
                  string(name: 'tailor_meta', value: tailor_meta),
                  string(name: 'pr_url', value: env.CHANGE_URL ),
                  string(name: 'trigger_comment_body', value: comment_body),
                  booleanParam(name: 'build_custom_docker', value: true),
                  string(name: 'sha', value: sha),
                ]
            }
          }
        }
      }

      stage("Configure build parameters") {
        agent { label 'master' }
        when {
          expression { !currentBuild.getBuildCauses(pr_comment_cause) }
        }
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

            // Retrieve repository information to handle github norifications
            def parts = env.GIT_URL
                        .replace('https://github.com/', '')
                        .replace('.git', '')
                        .split('/')
            env.NOTIFICATION_ACCOUNT = parts[0]
            env.NOTIFICATION_REPO = parts[1]
            env.SHA = env.GIT_COMMIT

            githubNotify(
              credentialsId: 'tailor_github_keypass',
              account: env.NOTIFICATION_ACCOUNT,
              repo: env.NOTIFICATION_REPO,
              sha: env.SHA,
              context: 'ci/build-and-test',
              description: 'Build and test stage running',
              status: 'PENDING',
              targetUrl: env.RUN_DISPLAY_URL
            )
          }
        }
      }

      stage("Rosdep check") {
        agent none
        when {
          expression { !currentBuild.getBuildCauses(pr_comment_cause) }
        }
        steps {
          script {
            def jobs = distributions.collectEntries { distribution ->
              [distribution, { node {
                try {
                  def deps_image = docker.image(dependencyImage(distribution))
                  docker.withRegistry(docker_registry, docker_credentials) { deps_image.pull() }

                  deps_image.inside("-v $HOME/tailor/ccache:/ccache") {
                    echo('↓↓↓ ROSDEP OUTPUT ↓↓↓')
                    withCredentials([string(credentialsId: 'tailor_github', variable: 'GITHUB_TOKEN')]) {
                      sh("""#!/bin/bash
                        source \$BUNDLE_ROOT/$rosdistro_name/setup.bash
                        echo "Pulling rosdistro..."
                        python3 /home/locus/pull_rosdistro.py --src-dir rosdistro --github-key $GITHUB_TOKEN --clean --ref $release_track
                        echo "Pulling distro repositories..."
                        python3 /home/locus/pull_distro_repositories.py --src-dir workspace/src --github-key $GITHUB_TOKEN \
                          --recipes $recipes_yaml --rosdistro-index $rosdistro_index --clean --ref ${env.CHANGE_BRANCH ?: env.BRANCH_NAME} --rosdistro-name $rosdistro_name

                        rosdep check --from-paths workspace/src/$rosdistro_name --ignore-src
                      """)
                      echo('↑↑↑ ROSDEP OUTPUT ↑↑↑')
                    }
                  }
                } finally {
                  library("tailor-meta@$tailor_meta")
                  cleanDocker()
                  deleteDir()
                }
              }}]
            }
            warnError('Rosdep check errors'){
              parallel(jobs)
            }
          }
        }
      }

      stage("Build and test") {
        agent none
        when {
          expression {
            !currentBuild.getBuildCauses(pr_comment_cause)
          }
        }
        steps {
          script {
            def jobs = distributions.collectEntries { distribution ->
              [distribution, { node {
                try {
                  def repository_dir = sh(
                    script: 'echo "$JOB_NAME" | cut -d"/" -f3',
                    returnStdout: true
                  ).trim()

                  dir(repository_dir) {
                    checkout(scm)
                  }
                  def test_image = docker.image(testImage(distribution))
                  docker.withRegistry(docker_registry, docker_credentials) { test_image.pull() }

                  def colcon_path_args = "--merge-install --base-paths ${repository_dir} --test-result-base test_results"

                  // TODO(pbovbel) pull from last rosdistro build artifact? or from s3?
                  def colcon_build_args = "--cmake-args -DCMAKE_CXX_FLAGS='-g -O3 -fext-numeric-literals -march=haswell -DBOOST_BIND_GLOBAL_PLACEHOLDERS -DBOOST_ALLOW_DEPRECATED_HEADERS' " +
                                          "-DCMAKE_CXX_STANDARD='17' -DCMAKE_CXX_STANDARD_REQUIRED='ON' " +
                                          "-DCMAKE_CXX_EXTENSIONS='ON' -DCMAKE_CXX_COMPILER_LAUNCHER=ccache"

                  test_image.inside("-v $HOME/tailor/ccache:/ccache") {
                    dir(repository_dir){
                      echo('↓↓↓ PRE-COMMIT OUTPUT ↓↓↓')
                      warnError('Pre-commit errors detected'){
                        sh('git locus-pre-commit-all')
                      }
                      echo('↑↑↑ PRE-COMMIT OUTPUT ↑↑↑')
                    }

                    echo('↓↓↓ TEST OUTPUT ↓↓↓')
                    sh("""#!/bin/bash
                      source \$BUNDLE_ROOT/$rosdistro_name/setup.bash &&
                      rosdep install --from-paths ${repository_dir} --ignore-src -y &&
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
    post{
      always{
        script{
          def build_causes = currentBuild.getBuildCauses(pr_comment_cause)
          if (build_causes.isEmpty()) {
            def result = currentBuild.currentResult
            switch (result) {
              case 'SUCCESS':
                github_status = 'SUCCESS'
                description = 'Build and test passed'
                break
              case 'FAILURE':
                github_status = 'FAILURE'
                description = 'Build and test failed'
                break
              case 'UNSTABLE':
                github_status = 'FAILURE'
                description = 'Build unstable (tests failed)'
                break
              case 'ABORTED':
                github_status = 'ERROR'
                description = 'Build and test aborted'
                break
              default:
                github_status = 'ERROR'
                description = "Unexpected build result: ${result}"
            }
            githubNotify(
              credentialsId: 'tailor_github_keypass',
              account: env.NOTIFICATION_ACCOUNT,
              repo: env.NOTIFICATION_REPO,
              sha: env.SHA,
              context: 'ci/build-and-test',
              description: description,
              status: github_status,
              targetUrl: env.RUN_DISPLAY_URL
            )
          } else {
            echo "Skipping GitHub notification"
          }
        }
      }
    }
  }
}
