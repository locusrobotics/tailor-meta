import java.net.URI

def call(Map args = [:]) {
  List<String> pr_urls = []
  if (args.pr_urls) pr_urls.addAll(args.pr_urls.collect { it.toString() })

  if (pr_urls.isEmpty()) {
    error "checkoutPR: pr_urls is required"
  }

  pr_urls.each { String pr_url ->
    def pr = parseGithubPrUrl(pr_url)

    checkout([$class: 'GitSCM',
        branches: [[name: "refs/remotes/origin/PR-${pr.prNum}"]],
        doGenerateSubmoduleConfigurations: false,
        extensions: [[$class: 'LocalBranch'], [$class: 'RelativeTargetDirectory', relativeTargetDir: "${args.base_dir}/${pr.repo}-PR-${pr.prNum}"]],
        userRemoteConfigs: [[
            url: "https://github.com/${pr.owner}/${pr.repo}.git",
            credentialsId: 'tailor_github_keypass',
            refspec: "+refs/pull/${pr.prNum}/head:refs/remotes/origin/PR-${pr.prNum}"
        ]],
    ])
  }
}

private Map parseGithubPrUrl(String pr_url) {
  def u = new URI(pr_url)
  def parts = u.path.split('/').findAll { it }

  return [
    owner: parts[0],
    repo:  parts[1],
    prNum: parts[3]
  ]
}
