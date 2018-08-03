#!/usr/bin/env groovy
multibranchPipelineJob(repo) {
    branchSources {
    github {
        repository(repo)
        repoOwner(org)
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
    configure {
    def traits = it / sources / data / 'jenkins.branch.BranchSource' / source / traits
    traits << 'com.cloudbees.jenkins.plugins.bitbucket.BranchDiscoveryTrait' {
        strategyId(3) // detect all branches -refer the plugin source code for various options
    }
    }
}
