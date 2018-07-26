#!/usr/bin/env groovy

// Learn groovy: https://learnxinyminutes.com/docs/groovy/

def days_to_keep = 10
def num_to_keep = 10
def upstream_project = '../rosdistro/' + env.BRANCH_NAME

timestamps {
  stage("Configure build parameters") {
    node('master') {
      sh 'env'
      cancelPreviousBuilds()

      def triggers = [
        upstream(upstreamProjects: upstream_project, threshold: hudson.model.Result.SUCCESS)
      ]

      // Choose build type based on tag/branch name
      if (env.TAG_NAME != null) {
        // Create tagged release
        days_to_keep = null
      } else if (env.BRANCH_NAME.startsWith('release/')) {
        // Create a release candidate
        days_to_keep = null
      }

      properties([
        buildDiscarder(logRotator(
          artifactDaysToKeepStr: days_to_keep.toString(), artifactNumToKeepStr: num_to_keep.toString(),
          daysToKeepStr: days_to_keep.toString(), numToKeepStr: num_to_keep.toString()
        )),
        pipelineTriggers(triggers)
      ])
    }
  }

  stage("Update repositories") {
    node {
      try {
        dir('tailor-meta') {
          checkout(scm)
        }
        copyArtifacts(projectName: upstream_project)
        sh 'ls'

      } finally {
        deleteDir()
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
