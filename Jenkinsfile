#!/usr/bin/env groovy
def docker_credentials = 'ecr:us-east-1:tailor_aws'
def parentImage = { release, docker_registry -> docker_registry - "https://" + ':tailor-meta-' + release + '-parent-' + env.BRANCH_NAME }

def rosdistro_index = 'rosdistro/rosdistro/index.yaml'
def recipes_yaml = 'rosdistro/config/recipes.yaml'

def uploadSuccessfulBuildParams = {
  def timestampSuffix = params.timestamp?.trim() ? params.timestamp : env.BUILD_NUMBER
  def latestFile = 'tailor-meta-successful-params.yaml'
  def historyFile = "tailor-meta-successful-params-${timestampSuffix}.yaml"
  def payload = [
    pipeline: 'tailor-meta',
    job_name: env.JOB_NAME,
    build_number: env.BUILD_NUMBER,
    build_url: env.BUILD_URL,
    release_label: params.release_label,
    created_at_utc: new Date().format("yyyy-MM-dd'T'HH:mm:ss'Z'", TimeZone.getTimeZone('UTC')),
    parameters: params.collectEntries { k, v -> [(k): v] }
  ]

  writeYaml(file: latestFile, data: payload)
  writeYaml(file: historyFile, data: payload)
  archiveArtifacts(artifacts: "${latestFile}, ${historyFile}", allowEmptyArchive: false)

  if (!params.apt_repo?.trim()) {
    echo('Skipping successful parameter upload because apt_repo is not set.')
    return
  }

  withCredentials([[$class: 'AmazonWebServicesCredentialsBinding', credentialsId: 'tailor_aws']]) {
    def bucketName = params.apt_repo.replace('s3://', '')
    s3Upload(
      bucket: bucketName,
      path: "${params.release_label}/pipeline-params/tailor-meta",
      includePathPattern: latestFile,
      workingDir: '.'
    )
    s3Upload(
      bucket: bucketName,
      path: "${params.release_label}/pipeline-params/tailor-meta/history",
      includePathPattern: historyFile,
      workingDir: '.'
    )
  }
}

pipeline {
  agent none

  parameters {
    string(name: 'rosdistro_job', defaultValue: '/ci/toydistro/master')
    string(name: 'release_track', defaultValue: 'hotdog')
    string(name: 'release_label', defaultValue: 'hotdog')
    string(name: 'num_to_keep', defaultValue: '10')
    string(name: 'days_to_keep', defaultValue: '10')
    string(name: 'tailor_meta')
    string(name: 'timestamp')
    string(name: 'docker_registry')
    string(name: 'apt_repo')
    booleanParam(name: 'deploy', defaultValue: false)
    booleanParam(name: 'invalidate_cache', defaultValue: false)
    string(name: 'apt_refresh_key')
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

    stage("Update repositories") {
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
              if (params.release_track == 'hotdog') {
                sh(
                  script: "update_repo_settings --rosdistro-index $rosdistro_index  --recipes $recipes_yaml " +
                          "--github-key $github_token ${params.deploy ? '--deploy' : ''} " +
                          "--release-track $params.release_track"
                )
              }
              def repositories_yaml = sh(
                script: "create_pipelines --rosdistro-index $rosdistro_index  --recipes $recipes_yaml " +
                        "--github-key $github_token --meta-branch $env.BRANCH_NAME " +
                        "--release-track $params.release_track --release-label $params.release_label " +
                        "--rosdistro-job $params.rosdistro_job ${params.deploy ? '--deploy' : ''}",
                returnStdout: true).trim()
              repositories = readYaml(text: repositories_yaml)
            }
          }
          unstash(name: 'source')
          if (params.deploy && params.release_track == 'hotdog') {
            // Only manage jenkins jobs from master branch
            jobDsl(
              targets: 'tailor-meta/jobs/tailorTestJob.groovy',
              removedJobAction: 'DELETE',
              additionalParameters: [
                'repositories': repositories,
                'credentials_id': 'tailor_github_keypass',
                // this extracts the (hopefully) unique job name: '/ci/rosdistro/master' -> 'rosdistro'
                'folder_name': "test/${params.rosdistro_job.split('/')[-2]}",
              ]
            )
          }
        }
      }
      post {
        cleanup {
          deleteDir()
        }
      }
    }
  }

  post {
    success {
      script {
        node('master') {
          uploadSuccessfulBuildParams()
          deleteDir()
        }
      }
    }
  }
}
