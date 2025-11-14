def call(Map args) {
  String tailor_distro = args['versions'].get('tailor_distro')
  String tailor_image = args['versions'].get('tailor_image')
  String tailor_meta = args['versions'].get('tailor_meta')

  pipeline {
    agent any

    stages {
      stage('Clone Repository') {
        steps {
          git branch: 'build-per-package', url: 'https://github.com/locusrobotics/tailor-distro.git'
        }
      }

      stage('Set Up Virtual Environment') {
          steps {
              sh """
                  /usr/bin/python3 -m venv venv
                  source venv/bin/activate
                  pip install --upgrade pip
              """
          }
      }

      stage('Build Package') {
          steps {
              sh """
                  source venv/bin/activate
                  pip install .
              """
          }
      }
    }
  }
}
