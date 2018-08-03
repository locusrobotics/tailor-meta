#!/usr/bin/env groovy
for (repository in repositories) {
    println("Creating multibranch pipeline for ${repository['owner_name']}/${repository['repo_name']")
    multibranchPipelineJob(repository['repo_name']) {
        branchSources {
            github {
                id(repository['repo_name'])
                repository(repository['repo_name'])
                repoOwner(repository['owner_name'])
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

    queue(repository['repo_name'])
}
