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

        configure { job ->
            def sourceNode = job / 'sources' / 'data' / 'jenkins.branch.BranchSource' / 'source'
            def traitsNode = sourceNode.children().find { it instanceof groovy.util.Node && it.name() == 'traits' }
            if (traitsNode == null) {
               traitsNode = sourceNode.appendNode('traits')
            }

            // Remove existing traits
            traitsNode.children().removeAll { n ->
              n.name() in [
                'org.jenkinsci.plugins.github_branch_source.BranchDiscoveryTrait',
                'org.jenkinsci.plugins.github_branch_source.OriginPullRequestDiscoveryTrait'
              ]
            }

            // Add Branch discovery, 1 = EXCLUDE_PRS
            traitsNode.appendNode('org.jenkinsci.plugins.github_branch_source.BranchDiscoveryTrait')
                      .appendNode('strategyId', '1')

            // Add PR discovery from origin, 1 = MERGE
            traitsNode.appendNode('org.jenkinsci.plugins.github_branch_source.OriginPullRequestDiscoveryTrait')
                    .appendNode('strategyId', '1')

            // Add property to trigger via PR comment
            def strategy = (job / 'sources' / 'data' / 'jenkins.branch.BranchSource' / 'strategy')
            def properties = (job / 'sources' / 'data' / 'jenkins.branch.BranchSource' / 'strategy' / 'properties')


            // If properties is empty-list, convert it to real list
            if (properties.@class == 'empty-list') {
                strategy.remove(properties)
                properties = strategy.appendNode('properties', [
                    class: 'java.util.Arrays$ArrayList'
                ])
                properties.appendNode('a', [
                    class: 'jenkins.branch.BranchProperty-array'
                ])
            }

            def aNode = properties / 'a'
            aNode.children().removeAll { n ->
                n instanceof groovy.util.Node &&
                n.name() == 'com.adobe.jenkins.github__pr__comment__build.TriggerPRCommentBranchProperty'
            }

            // Add PR comment trigger with the following regex
            def trigger = aNode.appendNode('com.adobe.jenkins.github__pr__comment__build.TriggerPRCommentBranchProperty')
            trigger.appendNode('commentBody', '(?is)^integration_tests\\b.*')
            trigger.appendNode('addReaction', 'true')
            trigger.appendNode('allowUntrusted', 'false')
        }
    }
    queue(job_name)
}

folder(folder_name)

repositories.each { repository ->
    def repo_name = repository['repo_name']
    def job_name = "$folder_name/$repo_name"
    def owner_name = repository['owner_name']
    println("Creating pipeline for $owner_name/$repo_name")
    buildPipelineJob(job_name, repo_name, owner_name, credentials_id)
}
