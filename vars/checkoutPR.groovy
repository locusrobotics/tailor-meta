def call(String pr_num, String repo) {
    checkout([$class: 'GitSCM',
        branches: [[name: "FETCH_HEAD"]],
        doGenerateSubmoduleConfigurations: false,
        extensions: [[$class: 'LocalBranch'], [$class: 'RelativeTargetDirectory', relativeTargetDir: "${repo}"]],
        userRemoteConfigs: [[
            url: "git@github.com:locusrobotics/${repo}.git",
            credentialsId: 'tailor_github_keypass',
            refspec: "+refs/pull/${pr_num}/head:refs/remotes/origin/PR-${pr_num} +refs/heads/devel:refs/remotes/origin/devel"
        ]],
    ])
}
