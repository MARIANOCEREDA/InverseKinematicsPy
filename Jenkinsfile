pipeline {
  //The agent is where the pipeline is going to be executed
  agent any
  
  // All the stages will be written inside this
  stages{
    stage("build"){
      steps{
        //For python --> pip install lib1, pip install lib2 would go here, etc
        echo 'building python application'
      }
    }
    stage("test"){
      steps{
        echo 'testing python application'
      }
    }
    stage("deploy"){
      steps{
        echo 'deploying python application'
      }
    }
  }
}

//This is a declarative pipeline
