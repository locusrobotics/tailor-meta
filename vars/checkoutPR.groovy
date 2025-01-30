Void call(String prNbr, String repo) {
    checkout([$class: 'GitSCM',
        branches: [[name: "FETCH_HEAD"]],
        doGenerateSubmoduleConfigurations: false,
        extensions: [[$class: 'LocalBranch'], [$class: 'RelativeTargetDirectory', relativeTargetDir: "${repo}"]],
        userRemoteConfigs: [[refspec: "+refs/pull/${prNbr}/head:refs/remotes/origin/PR-${prNbr} +refs/heads/master:refs/remotes/origin/master",
                            url: "https://${env.GITHUB_TOKEN}@github.com/locusrobotics/${repo}"]]
    ])
}
