pipeline {
    agent {
        docker { image 'moveit/moveit:kinetic-release' }
    }
    stages {
        stage('Build') {
            steps {
                sh 'source /opt/ros/kinetic/setup.bash'
                sh 'catkin_make'
            }
            post {
                always {
                    zip zipFile: 'build.zip', archive: false, dir: 'build'
                    zip zipFile: 'devel.zip', archive: false, dir: 'devel'
                    archiveArtifacts artifacts: 'build.zip', fingerprint: true
                    archiveArtifacts artifacts: 'devel.zip', fingerprint: true
                }
            }
        }
        stage('Test') {
            steps {
                copyArtifacts filter: 'build.zip', fingerprintArtifacts: true, projectName: '${JOB_NAME}', selector: specific('${BUILD_NUMBER}')
                copyArtifacts filter: 'devel.zip', fingerprintArtifacts: true, projectName: '${JOB_NAME}', selector: specific('${BUILD_NUMBER}')
                unzip zipFile: 'build.zip'
                unzip zipFile: 'devel.zip'
                sh 'source devel/setup.bash'
                sh 'catkin_make run_tests'
                sh 'catkin_make test'
            }
        }
    }
}