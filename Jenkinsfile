pipeline {
    agent {
        docker { image 'tandukion/ci-runner:test' }
    }
    stages {
        stage('Build') {
            steps {
                sh '''#!/bin/bash
                    source "/opt/ros/kinetic/setup.bash"
                    git submodule sync --recursive
                    git submodule update --init --recursive
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