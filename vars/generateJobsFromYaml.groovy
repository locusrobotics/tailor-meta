
return { String yamlPath ->
    def config = readYaml file: yamlPath

    if (!config?.packages) {
        error "No packages found in ${yamlPath}"
    }

    def dslScript = config.packages.collect { pkg ->
        def downstreamJobs = pkg.dependencies.collect { dep ->
            "publishers { downstream('${dep}', 'SUCCESS') }"
        }.join('\n')

        """
        job('${pkg.name}') {
            description('Build job for ${pkg.name}')
            steps {
                shell("Would build ${pkg.name}!!!!")
            }
            ${downstreamJobs}
        }
        """
    }.join('\n')

    jobDsl scriptText: dslScript, sandbox: false
}
