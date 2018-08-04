#!/usr/bin/env groovy

def buildPipelineJob(String repo_name, String owner_name, String credentials_id) {

    multibranchPipelineJob(repo_name) {
        branchSources {
            github {
                id(repo_name)
                repository(repo_name)
                repoOwner(owner_name)
                checkoutCredentialsId(credentials_id)
                scanCredentialsId(credentials_id)
            }
        }
        orphanedItemStrategy {
            discardOldItems {
                daysToKeep(10)
                numToKeep(10)
            }
        }
    }

    queue(repo_name)
}

repositories.each { repository ->
    def repo_name = repository['repo_name']
    def owner_name = repository['owner_name']
    println("Creating pipeline for $owner_name/$repo_name")
    buildPipelineJob(repo_name, owner_name, credentials_id)
}
