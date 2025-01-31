def call(String prNbr, String repo) {
    checkout([$class: 'GitSCM',
        branches: [[name: "FETCH_HEAD"]],
        doGenerateSubmoduleConfigurations: false,
        extensions: [[$class: 'LocalBranch'], [$class: 'RelativeTargetDirectory', relativeTargetDir: "${repo}"]],
        userRemoteConfigs: [[refspec: "+refs/pull/${prNbr}/head:refs/remotes/origin/PR-${prNbr} +refs/heads/devel:refs/remotes/origin/devel",
                            url: "https://${this.env.GITHUB_TOKEN}@github.com/locusrobotics/${repo}"]]
    ])
}
