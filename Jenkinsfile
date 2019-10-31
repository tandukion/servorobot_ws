pipeline {
    agent {
        docker { image 'tandukion/ci-runner:test' }
    }
    stages {
        stage('Build') {
            steps {
                bash '''#!/bin/bash
                    catkin_make
                '''
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
                bash '''#!/bin/bash
                    tar xf build.tar
                    source devel/setup.bash
                    catkin_make run_tests
                    catkin_make test
                '''
            }
        }
    }
}