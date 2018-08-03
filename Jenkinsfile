#!/usr/bin/env groovy

// Learn groovy: https://learnxinyminutes.com/docs/groovy/

def deploy = false

def docker_registry = '084758475884.dkr.ecr.us-east-1.amazonaws.com/tailor-meta'
def docker_registry_uri = 'https://' + docker_registry
def docker_credentials = 'ecr:us-east-1:tailor_aws'
def parentImage = { release -> docker_registry + ':jenkins-' + release + '-parent' }

def rosdistro_index = 'rosdistro/rosdistro/index.yaml'
def recipes_config = 'rosdistro/recipes/config.yaml'

pipeline {
  agent none

  parameters {
    string(name: 'rosdistro_source', defaultValue: 'master')
    string(name: 'release_track', defaultValue: 'hotdog')
    string(name: 'release_label', defaultValue: 'hotdog')
    string(name: 'num_to_keep', defaultValue: '10')
    string(name: 'days_do_keep', defaultValue: '10')
  }

  options {
    timestamps()
  }

  stages {
    stage("Configure build parameters") {
      agent any
      steps {
        script {
          sh('env')
          cancelPreviousBuilds()

          if (env.BRANCH_NAME == 'master') {
            deploy = true
          }

          properties([
            buildDiscarder(logRotator(
              daysToKeepStr: params.days_do_keep, numToKeepStr: params.num_to_keep,
              artifactDaysToKeepStr: params.days_to_keep, artifactNumToKeepStr: params.num_to_keep
            ))
          ])

          copyArtifacts(projectName: "../rosdistro/" + params.rosdistro_source)
          stash(name: 'rosdistro', includes: 'rosdistro/**')
        }
      }
      post {
        cleanup {
          deleteDir()
        }
      }
    }

    stage("Build and test tailor-meta") {
      agent any
      steps {
        script {
          dir('tailor-meta') {
            checkout(scm)
          }
          def parent_image = docker.image(parentImage(params.release_label))
          try {
            docker.withRegistry(docker_registry_uri, docker_credentials) { parent_image.pull() }
          } catch (all) {
            echo("Unable to pull ${parentImage(params.release_label)} as a build cache")
          }

          withCredentials([[$class: 'AmazonWebServicesCredentialsBinding', credentialsId: 'tailor_aws']]) {
            parent_image = docker.build(parentImage(params.release_label),
              "-f tailor-meta/environment/Dockerfile --cache-from ${parentImage(params.release_label)} " +
              "--build-arg AWS_ACCESS_KEY_ID=$AWS_ACCESS_KEY_ID " +
              "--build-arg AWS_SECRET_ACCESS_KEY=$AWS_SECRET_ACCESS_KEY .")
          }
          parent_image.inside() {
            sh('cd tailor-meta && python3 setup.py test')
          }
          docker.withRegistry(docker_registry_uri, docker_credentials) {
            parent_image.push()
          }
        }
      }
      post {
        always {
          junit(testResults: 'tailor-meta/test-results.xml', allowEmptyResults: true)
        }
        cleanup {
          deleteDir()
          // If two docker prunes run simulataneously, one will fail, hence || true
          sh('docker image prune -af --filter="until=3h" --filter="label=tailor" || true')
        }
      }
    }

    stage("Update repositories") {
      agent any
      steps {
        script {
          def parent_image = docker.image(parentImage(params.release_label))
          docker.withRegistry(docker_registry_uri, docker_credentials) {
            parent_image.pull()
          }
          parent_image.inside() {
            unstash(name: 'rosdistro')
            withCredentials([string(credentialsId: 'tailor_github', variable: 'github_token')]) {
              sh("create_pipelines --rosdistro-index $rosdistro_index  --recipes $recipes_config " +
                 "--github-key $github_token --meta-branch $env.BRANCH_NAME ${deploy ? '--deploy' : ''} " +
                 "--release-track $params.release_track")
            }
          }
          jobDsl(scriptText: """
            multibranchPipelineJob('locus_tools') {
              branchSources {
                github {
                  repository('locus_tools')
                  repoOwner('locusrobotics')
                  checkoutCredentialsId('tailor_github_keypass')
                  scanCredentialsId('tailor_github_keypass')
                }
              }
              orphanedItemStrategy {
                discardOldItems {
                  daysToKeep(10)
                  numToKeep(10)
                }
              }
            }
          """)
          jobDsl(scriptText: """
            multibranchPipelineJob('locus_lasers') {
              branchSources {
                github {
                  repository('locus_lasers')
                  repoOwner('locusrobotics')
                  checkoutCredentialsId('tailor_github_keypass')
                  scanCredentialsId('tailor_github_keypass')
                }
              }
              orphanedItemStrategy {
                discardOldItems {
                  daysToKeep(10)
                  numToKeep(10)
                }
              }
            }
          """)

        }
      }
      post {
        cleanup {
          deleteDir()
        }
      }
    }
  }
}


@NonCPS
def cancelPreviousBuilds() {
    def jobName = env.JOB_NAME
    def buildNumber = env.BUILD_NUMBER.toInteger()
    /* Get job name */
    def currentJob = Jenkins.instance.getItemByFullName(jobName)

    /* Iterating over the builds for specific job */
    for (def build : currentJob.builds) {
        /* If there is a build that is currently running and it's older than current build */
        if (build.isBuilding() && build.number.toInteger() < buildNumber) {
            /* Than stopping it */
            build.doStop()
        }
    }
}
