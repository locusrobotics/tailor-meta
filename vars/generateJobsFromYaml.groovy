
def call(String yamlPath) {
    // Read the YAML file
    def config = readYaml file: yamlPath

    if (!config?.packages) {
        error "No packages found in ${yamlPath}"
    }

    // Build the Job DSL script dynamically
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

    // Execute Job DSL
    jobDsl scriptText: dslScript, sandbox: false
}
