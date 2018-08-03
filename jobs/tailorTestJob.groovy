#!/usr/bin/env groovy
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
