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
    def track
    switch(getBuildType()) {
      case BuildType.FINAL:
        track = env.TAG_NAME
        break
      case BuildType.CANDIDATE:
        track = env.BRANCH_NAME - 'release/'
        break
      case BuildType.HOTDOG:
        track = 'hotdog'
        break
      case BuildType.FEATURE:
        track = 'hotdog'
        break
      case BuildType.TRIVIAL:
        track = null
        break
    }
    return track
  }

  def getBuildLabel = {
    def label
    switch(getBuildType()) {
      case BuildType.FINAL:
        label = getBuildTrack() + '-final'
        break
      case BuildType.CANDIDATE:
        label = getBuildTrack() + '-candidate'
        break
      case BuildType.HOTDOG:
        label = 'hotdog'
        break
      case BuildType.FEATURE:
        label = env.BRANCH_NAME - 'feature/'
        break
      case BuildType.TRIVIAL:
        label = null
        break
    }
    return label
  }

  def numToKeep = {
    switch(getBuildType()){
      case BuildType.FINAL: return 10
      case BuildType.CANDIDATE: return 10
      case BuildType.HOTDOG: return 10
      case BuildType.FEATURE: return 10
      case BuildType.TRIVIAL: return 10
    }
  }

  def daysToKeep = {
    switch(getBuildType()){
      case BuildType.FINAL: return null
      case BuildType.CANDIDATE: return null
      case BuildType.HOTDOG: return 10
      case BuildType.FEATURE: return 10
      case BuildType.TRIVIAL: return 10
    }
  }

  def createJobParameters = {
    def recipes_config = 'rosdistro/config/recipes.yaml'
    def common_config = readYaml(file: recipes_config)['common']
    [
      string(name: 'rosdistro_job', value: ('/' + env.JOB_NAME)),
      string(name: 'release_track', value: getBuildTrack()),
      string(name: 'release_label', value: getBuildLabel()),
      string(name: 'num_to_keep', value: numToKeep().toString()),
      string(name: 'days_to_keep', value: daysToKeep().toString()),
      // TODO(pbovbel) read these from rosdistro?
      string(name: 'apt_repo', value: common_config['apt_repo']),
      string(name: 'docker_registry', value: common_config['docker_registry']),
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
              triggers.add(cron( '0 */12 * * *')) // Build hotdog every 12 hours
            }

            properties([
              buildDiscarder(logRotator(
                daysToKeep: daysToKeep(), numToKeep: numToKeep(),
                artifactDaysToKeep: daysToKeep(), artifactNumToKeep: numToKeep()
              )),
              pipelineTriggers(triggers)
            ])
          }
        }
      }

      stage("Process rosdistro") {
        agent { label 'master' }
        steps {
          script {
            dir('rosdistro') {
              checkout(scm)
            }
            // TODO(pbovbel) validate rosdistro and config here
            archiveArtifacts(artifacts: "rosdistro/**/*", allowEmptyArchive: true)
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

      // stage("Sub-pipeline: bake images") {
      //   agent none
      //   when {
      //     expression {
      //       getBuildType() in [BuildType.HOTDOG, BuildType.CANDIDATE, BuildType.FINAL]
      //     }
      //   }
      //   steps {
      //     script {
      //       createTailorJob('tailor-image', tailor_image)
      //     }
      //   }

      // }

      // stage("Sub-pipeline: process meta") {
      //   agent none
      //   when {
      //     expression {
      //       getBuildType() in [BuildType.HOTDOG]
      //     }
      //   }
      //   steps {
      //     script {
      //       createTailorJob('tailor-meta', tailor_meta)
      //     }
      //   }
      // }

    }
  }
}
