#!/usr/bin/env groovy
def call(Map args) {
  // TODO(pbovbel) handle package whitelist
  String tailor_upstream = args.get('tailor_upstream')
  String tailor_distro = args.get('tailor_distro')
  String tailor_image = args.get('tailor_image')
  String tailor_meta = args.get('tailor_meta')

  enum BuildType{
    TRIVIAL,
    FEATURE,
    HOTDOG,
    CANDIDATE,
    FINAL
  }

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
      case BuildType.FINAL: track = env.TAG_NAME
      case BuildType.CANDIDATE: track = env.BRANCH_NAME - 'release/'
      case BuildType.HOTDOG: track = 'hotdog'
      case BuildType.FEATURE: track = 'hotdog'
      case BuildType.TRIVIAL: track = null
    }
    return track ? track.replaceAll("\\.", '-') : track
  }

  def getBuildLabel = {
    def label
    switch(getBuildType()) {
      case BuildType.FINAL: label = getBuildTrack() + '-final'
      case BuildType.CANDIDATE: label = getBuildTrack() + '-candidate'
      case BuildType.HOTDOG: label = 'hotdog'
      case BuildType.FEATURE: label = env.BRANCH_NAME - 'feature/'
      case BuildType.TRIVIAL: label = null
    }
    return label ? label.replaceAll("\\.", '-') : label
  }

  def numToKeep = {
    switch(getBuildType()){
      case BuildType.FINAL: 10
      case BuildType.CANDIDATE: 10
      case BuildType.HOTDOG: 10
      case BuildType.FEATURE: 10
      case BuildType.TRIVIAL: 10
    }
  }

  def daysToKeep = {
    switch(getBuildType()){
      case BuildType.FINAL: null
      case BuildType.CANDIDATE: null
      case BuildType.HOTDOG: 10
      case BuildType.FEATURE: 10
      case BuildType.TRIVIAL: 10
    }
  }

  def createJobParameters = {
    [
      // TODO(pbovbel) use URL encoding
      string(name: 'rosdistro_source', value: env.BRANCH_NAME.replaceAll('/', '%2F')),
      string(name: 'release_track', value: getBuildTrack()),
      string(name: 'release_label', value: getBuildLabel()),
      string(name: 'num_to_keep', value: numToKeep().toString()),
      string(name: 'days_to_keep', value: daysToKeep().toString()),
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
      booleanParam(name: 'force_distro', defaultValue: false)
      booleanParam(name: 'force_images', defaultValue: false)
      booleanParam(name: 'force_meta', defaultValue: false)
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

      stage("Sub-pipeline: create mirror") {
        agent none
        when {
          expression {
            params.force_mirror
          }
        }
        steps {
          script {
            createTailorJob('tailor-upstream', tailor_upstream)
          }
        }
      }

      stage("Sub-pipeline: build distribution") {
        agent none
        when {
          expression {
            params.force_distro || getBuildType() in [BuildType.FEATURE, BuildType.HOTDOG, BuildType.CANDIDATE, BuildType.FINAL]
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
            params.force_images || getBuildType() in [BuildType.HOTDOG, BuildType.CANDIDATE, BuildType.FINAL]
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
            params.force_meta || getBuildType() in [BuildType.HOTDOG]
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
