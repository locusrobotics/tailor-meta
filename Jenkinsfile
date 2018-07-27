#!/usr/bin/env groovy

// Learn groovy: https://learnxinyminutes.com/docs/groovy/

def release_track = 'hotdog'
def release_label = release_track
def days_to_keep = 10
def num_to_keep = 10
def deploy = true
def build_schedule = null

def docker_registry = '084758475884.dkr.ecr.us-east-1.amazonaws.com/tailor-meta'
def docker_credentials = 'ecr:us-east-1:tailor_aws'
def parentImage = { release -> docker_registry + ':jenkins-' + release + '-parent' }

timestamps {
  // TODO(pbovbel) this is repeated, a LOT. Make this a common piece somehow.
  stage("Configure build parameters") {
    node('master') {
      sh 'env'
      cancelPreviousBuilds()

      def triggers = []

      // Choose build type based on tag/branch name
      if (env.TAG_NAME != null) {
        // Create tagged release
        release_track = env.TAG_NAME
        release_label = release_track + '-final'
        days_to_keep = null
      } else if (env.BRANCH_NAME.startsWith('release/')) {
        // Create a release candidate
        release_track = env.BRANCH_NAME - 'release/'
        release_label = release_track + '-rc'
        days_to_keep = null
      } else if (env.BRANCH_NAME == 'master') {
        // Create mystery meat mirror
        if (build_schedule) { triggers.add(cron(build_schedule)) }
      } else {
        // Don't deploy for builds on feature branches
        release_label = release_track + '-' + env.BRANCH_NAME
        deploy = false
      }
      release_track = release_track.replaceAll("\\.", '-')
      release_label = release_label.replaceAll("\\.", '-')

      def rosdistro_project = '../rosdistro/' + env.BRANCH_NAME
      triggers.add(upstream(upstreamProjects: rosdistro_project, threshold: hudson.model.Result.SUCCESS))
      copyArtifacts(projectName: rosdistro_project)
      stash(name: 'rosdistro', includes: 'rosdistro/**')

      properties([
        buildDiscarder(logRotator(
          artifactDaysToKeepStr: days_to_keep.toString(), artifactNumToKeepStr: num_to_keep.toString(),
          daysToKeepStr: days_to_keep.toString(), numToKeepStr: num_to_keep.toString()
        )),
        pipelineTriggers(triggers)
      ])
    }
  }

  // TODO(pbovbel) this is repeated, a LOT. Make this a common piece somehow.
  stage("Build and test tailor-meta") {
    node {
      try {
        dir('tailor-meta') {
          checkout(scm)
        }
        def parent_image = docker.image(parentImage(release_label))
        try {
          docker.withRegistry(docker_registry_uri, docker_credentials) { parent_image.pull() }
        } catch (all) {
          echo "Unable to pull ${parentImage(release_label)} as a build cache"
        }

        withCredentials([[$class: 'AmazonWebServicesCredentialsBinding', credentialsId: 'tailor_aws']]) {
          parent_image = docker.build(parentImage(release_label),
            "-f tailor-meta/environment/Dockerfile --cache-from ${parentImage(release_label)} " +
            "--build-arg AWS_ACCESS_KEY_ID=$AWS_ACCESS_KEY_ID " +
            "--build-arg AWS_SECRET_ACCESS_KEY=$AWS_SECRET_ACCESS_KEY .")
        }
        parent_image.inside() {
          sh 'cd tailor-meta && python3 setup.py test'
        }
        docker.withRegistry(docker_registry_uri, docker_credentials) {
          parent_image.push()
        }
      } finally {
        junit(testResults: 'tailor-meta/test-results.xml', allowEmptyResults: true)
        deleteDir()
        // If two docker prunes run simulataneously, one will fail, hence || true
        sh 'docker image prune -af --filter="until=3h" --filter="label=tailor" || true'
      }
    }
  }

  stage("Deploy pipelines") {
    node {
      try {
        def parent_image = docker.image(parentImage(release_label))
        docker.withRegistry(docker_registry_uri, docker_credentials) {
          parent_image.pull()
        }
        parent_image.inside() {
          unstash(name: 'rosdistro')

          sh("create_pipelines ${deploy ? '--deploy' : ''}")
        }

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
