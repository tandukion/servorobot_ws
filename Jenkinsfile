pipeline {
    agent {
        docker { image 'moveit/moveit:kinetic-release' }
    }
    stages {
        stage('Build') {
            steps {
                sh 'ros_entrypoint.sh'
                sh 'catkin_make'
            }
            post {
                always {
                    sh 'tar cf build.tar build devel'
                    archiveArtifacts artifacts: 'build.tar', fingerprint: true
                }
            }
        }
        stage('Test') {
            steps {
                script {
                    copyArtifacts filter: 'build.tar', fingerprintArtifacts: true, projectName: '${JOB_NAME}', selector: specific('${BUILD_NUMBER}')
                    sh 'tar xf build.tar'
                    sh 'source devel/setup.bash'
                    sh 'catkin_make run_tests'
                    sh 'catkin_make test'
                }
            }
        }
    }
}