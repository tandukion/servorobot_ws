pipeline {
    agent {
        docker { image 'tandukion/ci-runner:test' }
    }
    stages {
        stage('Build') {
            steps {
                sh '''#!/bin/bash
                    source "/opt/ros/kinetic/setup.bash"
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
                sh '''#!/bin/bash
                    tar xf build.tar
                    source "/opt/ros/kinetic/setup.bash"
                    source devel/setup.bash
                    catkin_make run_tests
                    catkin_make test
                '''
            }
        }
    }
}