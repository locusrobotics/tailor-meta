#!/usr/bin/env groovy
def docker_credentials = 'ecr:us-east-1:tailor_aws'
def parentImage = { release, docker_registry -> docker_registry - "https://" + ':tailor-meta-' + release + '-parent-' + env.BRANCH_NAME }

def rosdistro_index = 'rosdistro/rosdistro/index.yaml'
def recipes_yaml = 'rosdistro/config/recipes.yaml'

def workspace_dir = 'workspace'
def recipes_dir = workspace_dir + '/recipes'
def src_dir = workspace_dir + '/src'
def debian_dir = workspace_dir + '/debian'


def debianStash = { recipe -> recipe + "-debian"}
def packageStash = { recipe -> recipe + "-packages"}
def recipeStash = { recipe -> recipe + "-recipes"}
def srcStash = { release -> release + '-src' }
def graphStash = { recipe -> recipe + "-graph"}

pipeline {
  agent none

  parameters {
    string(name: 'rosdistro_job', defaultValue: '/ci/rosdistro/master')
    string(name: 'release_track', defaultValue: 'hotdog')
    string(name: 'release_label', defaultValue: 'build-per-package')
    string(name: 'num_to_keep', defaultValue: '10')
    string(name: 'days_to_keep', defaultValue: '10')
    string(name: 'docker_registry', defaultValue: 'https://084758475884.dkr.ecr.us-east-1.amazonaws.com/locus-tailor')
    booleanParam(name: 'deploy', defaultValue: false)
    booleanParam(name: 'invalidate_cache', defaultValue: false)
    string(name: 'apt_refresh_key')
    string(name: 'tailor_meta', defaultValue: 'build-per-package')
    string(name: 'python_version', defaultValue: '3')
  }

  options {
    timestamps()
  }

  stages {
    stage("Configure build parameters") {
      agent { label 'master' }
      steps {
        script {
          sh('env')

          properties([
            buildDiscarder(logRotator(
              daysToKeepStr: params.days_to_keep, numToKeepStr: params.num_to_keep,
              artifactDaysToKeepStr: params.days_to_keep, artifactNumToKeepStr: params.num_to_keep
            ))
          ])

          copyArtifacts(
            projectName: params.rosdistro_job,
            selector: upstream(fallbackToLastSuccessful: true),
          )
          stash(name: 'rosdistro', includes: 'rosdistro/**')
        }
      }
      post {
        cleanup {
          deleteDir()
        }
      }
    }

    stage("Build tailor-meta") {
      agent any
      steps {
        script {
          dir('tailor-meta') {
            checkout(scm)
          }
          stash(name: 'source', includes: 'tailor-meta/**')
          def parent_image_label = parentImage(params.release_label, params.docker_registry)
          def parent_image = docker.image(parent_image_label)
          withEnv(['DOCKER_BUILDKIT=1']) {
            try {
              docker.withRegistry(params.docker_registry, docker_credentials) { parent_image.pull() }
            } catch (all) {
              echo("Unable to pull ${parent_image_label} as a build cache")
            }

            withCredentials([[$class: 'AmazonWebServicesCredentialsBinding', credentialsId: 'tailor_aws']]) {
              parent_image = docker.build(parent_image_label,
                "${params.invalidate_cache ? '--no-cache ' : ''}" +
                "-f tailor-meta/environment/Dockerfile --cache-from ${parent_image_label} " +
                "--build-arg AWS_ACCESS_KEY_ID=$AWS_ACCESS_KEY_ID " +
                "--build-arg AWS_SECRET_ACCESS_KEY=$AWS_SECRET_ACCESS_KEY " +
                "--build-arg BUILDKIT_INLINE_CACHE=1 " +
                "--build-arg APT_REFRESH_KEY=${params.apt_refresh_key} .")
            }
            parent_image.inside() {
              sh('pip3 install -e tailor-meta --break-system-packages')
              //withCredentials([string(credentialsId: 'tailor_github', variable: 'GITHUB_TOKEN')]) {
              //  sh "pull_distro_repositories --src-dir $src_dir --github-key $GITHUB_TOKEN " +
              //    "--recipes $recipes_yaml  --rosdistro-index $rosdistro_index --clean"
              //  stash(name: srcStash(params.release_label), includes: "$src_dir/")
              //}
              //sh(
              //  script: "create_recipes --recipes $recipes_yaml --recipes-dir $recipes_dir " +
              //        "--release-track $params.release_track --release-label $params.release_label --debian-version $params.timestamp"
              //)
            }


            docker.withRegistry(params.docker_registry, docker_credentials) {
              parent_image.push()
            }
          }
        }
      }
      post {
        cleanup {
          library("tailor-meta@${params.tailor_meta}")
          cleanDocker()
          deleteDir()
        }
      }
    }

    stage("build graph") {
      agent any
      steps {
        script {
          def parent_image = docker.image(parentImage(params.release_label, params.docker_registry))
          docker.withRegistry(params.docker_registry, docker_credentials) {
            parent_image.pull()
          }
          def repositories = null
          parent_image.inside() {
            unstash(name: 'rosdistro')
            withCredentials([string(credentialsId: 'tailor_github', variable: 'github_token')]) {
              unstash(name: 'rosdistro')
              // Generate recipe configuration files
              def recipe_yaml = sh(
                script: "create_recipes --recipes $recipes_yaml --recipes-dir $recipes_dir " +
                        "--release-track $params.release_track --release-label $params.release_label --debian-version $params.timestamp",
                returnStdout: true).trim()

              // Script returns a mapping of recipe labels and paths
              recipes = readYaml(text: recipe_yaml)

              distributions = readYaml(file: recipes_yaml)['os'].collect {
                os, distribution -> distribution }.flatten()

              // Stash each recipe configuration individually for parallel build nodes
              recipes.each { recipe_label, recipe_path ->
                stash(name: recipeStash(recipe_label), includes: recipe_path)
              }

              // Pull down distribution sources
              withCredentials([string(credentialsId: 'tailor_github', variable: 'GITHUB_TOKEN')]) {
                sh "pull_distro_repositories --src-dir $src_dir --github-key $GITHUB_TOKEN " +
                  "--recipes $recipes_yaml  --rosdistro-index $rosdistro_index --clean"
                stash(name: srcStash(params.release_label), includes: "$src_dir/")
              }

              //recipes.each { recipe_label, recipe_path ->
              //  sh "ROS_PYTHON_VERSION=$params.python_version generate_bundle_templates --src-dir $src_dir --template-dir $debian_dir --recipe $recipe_path"
              //}
              sh "sudo rosdep init"
              sh "rosdep update"
              sh "rosdep resolve google-mock"
              //sh "rosdep update"
              //sh "rosdep resolve google-mock"

              //sh "ROS_PYTHON_VERSION=$params.python_version generate_bundle_templates --src-dir $src_dir --template-dir $debian_dir --recipe $recipe_path"
              sh "blossom graph --workspace $workspace_dir --recipe $recipes_dir"
              //stash(name: )

              sh "blossom test --workspace $workspace_dir --recipe $recipes_dir --graph $workspace_dir/graphs/ubuntu-jammy-ros1-graph.yaml"
            }
              //def repositories_yaml = sh(
              //  script: "create_pipelines --rosdistro-index $rosdistro_index  --recipes $recipes_yaml " +
              //          "--github-key $github_token --meta-branch $env.BRANCH_NAME " +
              //          "--release-track $params.release_track --release-label $params.release_label " +
              //          "--rosdistro-job $params.rosdistro_job ${params.deploy ? '--deploy' : ''}",
              //  returnStdout: true).trim()
              //sh(
              //  script: "update_repo_settings --rosdistro-index $rosdistro_index  --recipes $recipes_yaml " +
              //          "--github-key $github_token ${params.deploy ? '--deploy' : ''} " +
              //          "--release-track $params.release_track")
              //repositories = readYaml(text: repositories_yaml)
            }
          }
          unstash(name: 'source')
        }

        post {
          cleanup {
            deleteDir()
          }
        }
    }
    //stage("Update repositories") {
    //  agent any
    //  steps {
    //    script {
    //      def parent_image = docker.image(parentImage(params.release_label, params.docker_registry))
    //      docker.withRegistry(params.docker_registry, docker_credentials) {
    //        parent_image.pull()
    //      }
    //      def repositories = null
    //      parent_image.inside() {
    //        unstash(name: 'rosdistro')
    //        withCredentials([string(credentialsId: 'tailor_github', variable: 'github_token')]) {
    //          def repositories_yaml = sh(
    //            script: "create_pipelines --rosdistro-index $rosdistro_index  --recipes $recipes_yaml " +
    //                    "--github-key $github_token --meta-branch $env.BRANCH_NAME " +
    //                    "--release-track $params.release_track --release-label $params.release_label " +
    //                    "--rosdistro-job $params.rosdistro_job ${params.deploy ? '--deploy' : ''}",
    //            returnStdout: true).trim()
    //          sh(
    //            script: "update_repo_settings --rosdistro-index $rosdistro_index  --recipes $recipes_yaml " +
    //                    "--github-key $github_token ${params.deploy ? '--deploy' : ''} " +
    //                    "--release-track $params.release_track")
    //          repositories = readYaml(text: repositories_yaml)
    //        }
    //      }
    //      unstash(name: 'source')
    //      if (params.deploy && params.release_track == 'hotdog') {
    //        // Only manage jenkins jobs from master branch
    //        jobDsl(
    //          targets: 'tailor-meta/jobs/tailorTestJob.groovy',
    //          removedJobAction: 'DELETE',
    //          additionalParameters: [
    //            'repositories': repositories,
    //            'credentials_id': 'tailor_github_keypass',
    //            // this extracts the (hopefully) unique job name: '/ci/rosdistro/master' -> 'rosdistro'
    //            'folder_name': "test/${params.rosdistro_job.split('/')[-2]}",
    //          ]
    //        )
    //      }
    //    }
    //  }
    //  post {
    //    cleanup {
    //      deleteDir()
    //    }
    //  }
    //}
  }
}
