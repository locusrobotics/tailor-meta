#!/usr/bin/env groovy
def deploy = false

def docker_credentials = 'ecr:us-east-1:tailor_aws'

def recipes_yaml = 'rosdistro/config/recipes.yaml'
def upstream_yaml = 'rosdistro/config/upstream.yaml'
def rosdistro_index = 'rosdistro/rosdistro/index.yaml'
def workspace_dir = 'workspace'

def distributions = []
def recipes = [:]

def recipes_dir = workspace_dir + '/recipes'
def src_dir = workspace_dir + '/src'
def debian_dir = workspace_dir + '/debian'

def srcStash = { release -> release + '-src' }
def parentImage = { release, docker_registry -> docker_registry - "https://" + ':tailor-distro-' + release + '-parent-' + env.BRANCH_NAME }
def bundleImage = { release, os_version, docker_registry -> docker_registry - "https://" + ':tailor-distro-' + release + '-bundle-' + os_version + '-' + env.BRANCH_NAME }
def debianStash = { recipe -> recipe + "-debian"}
def packageStash = { recipe -> recipe + "-packages"}
def recipeStash = { recipe -> recipe + "-recipes"}

def call(Map args) {
  String tailor_distro = args['versions'].get('tailor_distro')
  String tailor_image = args['versions'].get('tailor_image')
  String tailor_meta = args['versions'].get('tailor_meta')

  pipeline {
    agent none

    parameters {
      string(name: 'rosdistro_job', defaultValue: '/ci/toydistro/master')
      string(name: 'release_track', defaultValue: 'hotdog')
      string(name: 'release_label', defaultValue: 'hotdog')
      string(name: 'num_to_keep', defaultValue: '10')
      string(name: 'days_to_keep', defaultValue: '10')
      string(name: 'timestamp')
      string(name: 'python_version', defaultValue: '3')
      string(name: 'tailor_meta')
      string(name: 'docker_registry')
      string(name: 'apt_repo')
      string(name: 'retries', defaultValue: '3')
      booleanParam(name: 'deploy', defaultValue: false)
      booleanParam(name: 'force_mirror', defaultValue: false)
      booleanParam(name: 'invalidate_cache', defaultValue: false)
      string(name: 'apt_refresh_key')
    }

    options {
      timestamps()
    }

    stages {
      stage("Configure build parameters") {
        agent { label('master') }
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

      stage("Build and test tailor-distro") {
        agent any
        steps {
          script {
            dir('tailor-distro') {
              checkout(scm)
            }
            def parent_image_label = parentImage(params.release_label, params.docker_registry)
            def parent_image = docker.image(parent_image_label)

            withEnv(['DOCKER_BUILDKIT=1']) {
              try {
                docker.withRegistry(params.docker_registry, docker_credentials) {parent_image.pull()}
              } catch (all) {
                echo("Unable to pull ${parent_image_label} as a build cache")
              }

              withCredentials([[$class: 'AmazonWebServicesCredentialsBinding', credentialsId: 'tailor_aws']]) {
                unstash(name: 'rosdistro')
                parent_image = docker.build(parent_image_label,
                  "${params.invalidate_cache ? '--no-cache ' : ''}" +
                  "-f tailor-distro/environment/Dockerfile --cache-from ${parent_image_label} " +
                  "--build-arg AWS_ACCESS_KEY_ID=$AWS_ACCESS_KEY_ID " +
                  "--build-arg AWS_SECRET_ACCESS_KEY=$AWS_SECRET_ACCESS_KEY " +
                  "--build-arg BUILDKIT_INLINE_CACHE=1 " +
                  "--build-arg APT_REFRESH_KEY=${params.apt_refresh_key} .")
              }
              parent_image.inside() {
                sh('pip3 install -e tailor-distro --break-system-packages')
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

      stage("Generate graph") {
        agent any
        steps {
          script {
            def parent_image = docker.image(parentImage(params.release_label, params.docker_registry))
            retry(params.retries as Integer) {
              docker.withRegistry(params.docker_registry, docker_credentials) { parent_image.pull() }
            }

            parent_image.inside() {
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
            }
          }
        }
      }
    }
  }
}
