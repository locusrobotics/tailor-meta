#!/usr/bin/env groovy
multibranchPipelineJob(repo_name) {
    branchSources {
        github {
            id(repo_name)
            repository(repo_name)
            repoOwner(org_name)
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
    // configure {
    //     def traits = it / sources / data / 'jenkins.branch.BranchSource' / source / traits
    //     traits << 'com.cloudbees.jenkins.plugins.github_branch_source.BranchDiscoveryTrait' {
    //         strategyId(1) // detect all branches -refer the plugin source code for various options
    //     }
    // }
}

queue(repo_name)
