pipeline {
    agent {
        docker { image 'tandukion/ci-runner:test' }
    }
    stages {
        stage('Build') {
            steps {
                sh 'echo "Hello World"'
            }
            post {
                success {
                    sh 'tar cf build.tar build devel'
                    archiveArtifacts artifacts: 'build.tar', fingerprint: true
                }
            }
        }
        stage('Test') {
            steps {
                copyArtifacts filter: 'build.tar', fingerprintArtifacts: true, projectName: '${JOB_NAME}', selector: specific('${BUILD_NUMBER}')
                sh 'tar xf build.tar'
                sh 'source devel/setup.bash'
                sh 'catkin_make run_tests'
                sh 'catkin_make test'
            }
        }
    }
}