pipeline {
    agent { label 'amd64'}

    stages {
        stage('Build') {
            steps {
                sh '/bin/bash ./jenkins_build.sh'
            }
        }
    }
}
