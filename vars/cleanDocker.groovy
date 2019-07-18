#!/usr/bin/env groovy
def call(Map args) {
    sh('docker image prune -af --filter="until=24h" --filter="label=tailor" || true')
    sh('docker image prune -af --filter="until=01h" --filter="label=tailor=environment" || true')
}
