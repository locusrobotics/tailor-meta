def call(String prNbr, String repo) {
    checkout([$class: 'GitSCM',
        branches: [[name: branch]],
        doGenerateSubmoduleConfigurations: false,
        extensions: [[$class: 'LocalBranch'], [$class: 'RelativeTargetDirectory', relativeTargetDir: "${repo}"]],
        userRemoteConfigs: [[refspec: "+refs/pull/*:refs/remotes/origin/PR-* +refs/heads/devel:refs/remotes/origin/devel",
                            url: "https://${this.env.GITHUB_TOKEN}@github.com/locusrobotics/${repo}"]]
    ])
}
