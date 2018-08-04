#!/usr/bin/env groovy

def buildPipelineJob(String job_name, String repo_name, String owner_name, String credentials_id) {
    multibranchPipelineJob(job_name) {
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

    queue(job_name)
}

def folder_name = 'repos'
folder(folder_name)

repositories.each { repository ->
    def repo_name = repository['repo_name']
    def job_name = "$folder_name/$repo_name"
    def owner_name = repository['owner_name']
    println("Creating pipeline for $owner_name/$repo_name")
    buildPipelineJob(job_name, repo_name, owner_name, credentials_id)
}
