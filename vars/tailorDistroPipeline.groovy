#!/usr/bin/env groovy
enum BuildType{
  TRIVIAL,
  FEATURE,
  HOTDOG,
  CANDIDATE,
  FINAL
}

def call(Map args) {
  String tailor_distro = args['versions'].get('tailor_distro')
  String tailor_image = args['versions'].get('tailor_image')
  String tailor_meta = args['versions'].get('tailor_meta')

  def recipes_yaml = 'rosdistro/config/recipes.yaml'
  def common_config = [:]

  def getBuildType = {
    if (env.TAG_NAME != null) {
      return BuildType.FINAL
    } else if (env.BRANCH_NAME.startsWith('release/')) {
      return BuildType.CANDIDATE
    } else if (env.BRANCH_NAME == 'master') {
      return BuildType.HOTDOG
    } else if (env.BRANCH_NAME.startsWith('feature/')) {
      return BuildType.FEATURE
    } else {
      return BuildType.TRIVIAL
    }
  }

  def getBuildTrack = {
    switch(getBuildType()) {
      case BuildType.FINAL:  // convert 19.1.x into 19.1
        return env.TAG_NAME.split('\\.')[0..1].join('.')
      case BuildType.CANDIDATE: // convert release/19.1 into 19.1
        return env.BRANCH_NAME - 'release/'
      case BuildType.HOTDOG:
      case BuildType.FEATURE:
        return 'hotdog'
      case BuildType.TRIVIAL:
        return null
    }
  }

  def getBuildLabel = {
    switch(getBuildType()) {
      case BuildType.FINAL:
        return env.TAG_NAME
      case BuildType.CANDIDATE:
        return getBuildTrack() + '-rc'
      case BuildType.HOTDOG:
        return getBuildTrack()
      case BuildType.FEATURE:
        return env.BRANCH_NAME - 'feature/'
      case BuildType.TRIVIAL:
        return null
    }
  }

  def numToKeep = {
    switch(getBuildType()){
      case BuildType.FINAL:
      case BuildType.CANDIDATE:
      case BuildType.HOTDOG:
      case BuildType.FEATURE:
        return "10"
      case BuildType.TRIVIAL:
        return "1"
    }
  }

  def daysToKeep = {
    switch(getBuildType()){
      case BuildType.FINAL:
      case BuildType.CANDIDATE:
        return ""
      case BuildType.HOTDOG:
      case BuildType.FEATURE:
        return "10"
      case BuildType.TRIVIAL:
        return "1"
    }
  }

  // (pbovbel) Currentlyt all sub-pipelines use the same parameters, even if some of them are unused.
  // This may need to change in the future.
  def createJobParameters = {
    [
      string(name: 'rosdistro_job', value: ('/' + env.JOB_NAME)),
      string(name: 'release_track', value: getBuildTrack()),
      string(name: 'release_label', value: getBuildLabel()),
      string(name: 'num_to_keep', value: numToKeep()),
      string(name: 'days_to_keep', value: daysToKeep()),
      string(name: 'apt_repo', value: common_config['apt_repo']),
      string(name: 'docker_registry', value: common_config['docker_registry']),
      booleanParam(name: 'force_mirror', value: params.force_mirror),
      booleanParam(name: 'deploy', value: true),
    ]
  }

  def createTailorJob = { job_name, branch ->
    build(
      job: "/ci/$job_name/$branch",
      quietPeriod: 5,
      parameters: createJobParameters()
    )
  }

  pipeline {
    agent none

    parameters {
      booleanParam(name: 'force_mirror', defaultValue: false)
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
            library("tailor-meta@$tailor_meta")
            cancelPreviousBuilds()

            echo "Build Type: ${getBuildType()}, Track:${getBuildTrack()}, Label: ${getBuildLabel()}"

            def triggers = []

            if (getBuildType() == BuildType.HOTDOG) {
              triggers.add(cron('0 2 * * *')) // Build at 2 am every day
            }

            properties([
              buildDiscarder(logRotator(
                daysToKeepStr: daysToKeep(), numToKeepStr: numToKeep(),
                artifactDaysToKeepStr: daysToKeep(), artifactNumToKeepStr: numToKeep()
              )),
              pipelineTriggers(triggers)
            ])

            dir('rosdistro') {
              checkout(scm)
            }

            // TODO(pbovbel) validate rosdistro and config here

            common_config = readYaml(file: recipes_yaml)['common']
            archiveArtifacts(artifacts: "rosdistro/**/*", allowEmptyArchive: true)
            if (getBuildType() in [BuildType.HOTDOG, BuildType.CANDIDATE, BuildType.FINAL]) {
              s3Upload(
                bucket: common_config['apt_repo'] - 's3://',
                file: 'rosdistro/rosdep',
                path: getBuildTrack() + '/'
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

      stage("Sub-pipeline: build distribution") {
        agent none
        when {
          expression {
            getBuildType() in [BuildType.FEATURE, BuildType.HOTDOG, BuildType.CANDIDATE, BuildType.FINAL]
          }
        }
        steps {
          script {
            createTailorJob('tailor-distro', tailor_distro)
          }
        }
      }

      stage("Sub-pipeline: bake images") {
        agent none
        when {
          expression {
            getBuildType() in [BuildType.HOTDOG, BuildType.CANDIDATE, BuildType.FINAL]
          }
        }
        steps {
          script {
            createTailorJob('tailor-image', tailor_image)
          }
        }
      }

      stage("Sub-pipeline: process meta") {
        agent none
        when {
          expression {
            getBuildType() in [BuildType.HOTDOG, BuildType.CANDIDATE]
          }
        }
        steps {
          script {
            createTailorJob('tailor-meta', tailor_meta)
          }
        }
      }

    }
  }
}
