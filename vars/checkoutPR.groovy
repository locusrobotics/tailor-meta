def call(String prNbr, String repo) {
    checkout([$class: 'GitSCM',
        branches: [[name: "FETCH_HEAD"]],
        doGenerateSubmoduleConfigurations: false,
        extensions: [[$class: 'LocalBranch'], [$class: 'RelativeTargetDirectory', relativeTargetDir: "${repo}"]],
        userRemoteConfigs: [[refspec: "+refs/pull/${pr_num}/head:refs/remotes/origin/PR-${pr_num} +refs/heads/devel:refs/remotes/origin/devel",
                            url: "https://${this.env.GITHUB_TOKEN}@github.com/locusrobotics/${repo}"]]
    ])
}
