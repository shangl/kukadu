#include "kukadu/robot/CustomKinestheticTeacher.hpp"
#include "Eigen/Jacobi"

using namespace std;
using namespace arma;

namespace kukadu{

CustomKinestheticTeacher::CustomKinestheticTeacher(KUKADU_SHARED_PTR<kukadu::ControlQueue> myQueue,KUKADU_SHARED_PTR<kukadu::MoveItKinematics> myKin,KUKADU_SHARED_PTR<kukadu::SensorStorage> myStore){//(ros::NodeHandle &node)

    teacherRunning=false;
    filterRunning=false;
    // Kukadu

    recordingPath="/home/qusai/catkin_ws/src/kinesthetic_teaching/src/rrt";
    robotinoQueue = myQueue;
    mvKin = myKin;
    store = myStore;


}


void CustomKinestheticTeacher::init(){


    qThread = robotinoQueue->startQueue();
    if(robotinoQueue->getCurrentMode() != KukieControlQueue::KUKA_JNT_POS_MODE) {
        robotinoQueue->stopCurrentMode();
        robotinoQueue->switchMode(KukieControlQueue::KUKA_JNT_POS_MODE);
    }

    moveThread =std::make_shared<std::thread>(&CustomKinestheticTeacher::teachingThreadHandler, this);
}


void CustomKinestheticTeacher::generateNextCommand(){



    auto currentJointState=robotinoQueue->getCurrentJoints().joints;
    auto diff = getNextDifferentialCommand(mvKin->getJacobian(),currentJointState,JACOBIAN);
    if (isDifferentialCommandSafe(diff,currentJointState))
        robotinoQueue->move(currentJointState +diff); // cout << "next command: " <<  currentJointState + diff << endl;
    else
        cout << "not safe command" << endl;

}

void CustomKinestheticTeacher::startRecording(){

    store->setExportMode(SensorStorage::STORE_TIME | SensorStorage::STORE_RBT_CART_POS | SensorStorage::STORE_RBT_JNT_POS);
    deleteDirectory(recordingPath);
    recordingThread= store->startDataStorage(recordingPath);

}

void CustomKinestheticTeacher::stopRecording(){

    store->stopDataStorage();
}


arma::vec CustomKinestheticTeacher::getNextDifferentialCommand(Eigen::MatrixXd jacobian,arma::vec currentJointState, ControllerType myType){


    std::vector<double> sensorReading= armadilloToStdVec(robotinoQueue->getCurrentProcessedCartesianFrcTrq().joints);
    auto numberOfCartesianFTs=jacobian.rows();
    if (numberOfCartesianFTs != sensorReading.size()){
        cout << "Problem in sensor readings vector size" << endl;
        return stdToArmadilloVec({0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0});
    }
    std::vector<double> forceVector = scaleForcesTorques(sensorReading);
    std::vector<double> additiveDifferential;

    switch(myType){
    case JACOBIAN:
    {

        additiveDifferential =  eigenToStdVec(jacobian.transpose() * stdToEigenVec(forceVector));
        additiveDifferential= scaleJointCommands(additiveDifferential);
        break;
    }
    case INVERSE://Pseudo inverse..horrible!
    {
        Eigen::JacobiSVD<Eigen::MatrixXd> svd(jacobian, Eigen::ComputeThinU | Eigen::ComputeThinV);
        additiveDifferential= scaleJointCommands(eigenToStdVec(0.15*svd.solve(stdToEigenVec(forceVector))));
        break;
    }
    case IK:
    {
        const double scaleIK=0.2;
        forceVector = armadilloToStdVec( scaleIK*stdToArmadilloVec(forceVector));
        geometry_msgs::Pose currentPose = mvKin->computeFk(armadilloToStdVec(currentJointState));
        geometry_msgs::Pose newPose;
        newPose.position.x= currentPose.position.x + forceVector.at(0);
        newPose.position.y=currentPose.position.y + forceVector.at(1);
        newPose.position.z=currentPose.position.z + forceVector.at(2);
        tf::Quaternion quat(currentPose.orientation.x,currentPose.orientation.y,currentPose.orientation.z,currentPose.orientation.w);
        tf::Matrix3x3 m(quat);
        double roll,pitch,yaw;
        m.getRPY(roll,pitch,yaw);
        roll+= forceVector.at(3);
        pitch+= forceVector.at(4);
        yaw+= forceVector.at(5);
        newPose.orientation = tf::createQuaternionMsgFromRollPitchYaw(roll,pitch,yaw);
        auto planIK = mvKin->computeIk(armadilloToStdVec(currentJointState),newPose);
        auto target =currentJointState;
        if (planIK.size()>0)
            target=planIK.back();
        else
            cout << "IK solution not found" << endl;

        additiveDifferential = armadilloToStdVec( target - currentJointState);
        if (isBigJump(additiveDifferential)){
            additiveDifferential={0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0};
            cout << " Big Jump from IK solution" << endl;
        }
        break;
    }
    case HYBRID:
    {
        const double scaleIK=0.1;
        additiveDifferential={0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0};
        forceVector = armadilloToStdVec( scaleIK*stdToArmadilloVec(forceVector));
        geometry_msgs::Pose currentPose = mvKin->computeFk(armadilloToStdVec(currentJointState));
        geometry_msgs::Pose newPose;
        newPose.position.x= currentPose.position.x + forceVector.at(0);
        newPose.position.y=currentPose.position.y + forceVector.at(1);
        newPose.position.z=currentPose.position.z + forceVector.at(2);
        tf::Quaternion quat(currentPose.orientation.x,currentPose.orientation.y,currentPose.orientation.z,currentPose.orientation.w);
        tf::Matrix3x3 m(quat);
        double roll,pitch,yaw;
        m.getRPY(roll,pitch,yaw);
        roll+= forceVector.at(3);
        pitch+= forceVector.at(4);
        yaw+= forceVector.at(5);
        newPose.orientation = tf::createQuaternionMsgFromRollPitchYaw(roll,pitch,yaw);
        auto planIK = mvKin->computeIk(armadilloToStdVec(currentJointState),newPose);
        auto target =currentJointState;
        bool jacobianUse = false;
        if (planIK.size()>0){
            target=planIK.back();
            additiveDifferential = armadilloToStdVec( target - currentJointState);
        }
        else
            jacobianUse =true;

        if (isBigJump(additiveDifferential))
            jacobianUse =true;


        if (jacobianUse){
            additiveDifferential =  eigenToStdVec(jacobian.transpose() * stdToEigenVec(forceVector));
            additiveDifferential= scaleJointCommands(additiveDifferential);
            cout << " jacobian" << endl;
        }
        break;
    }
    }


    return stdToArmadilloVec(capVec(additiveDifferential,MAXIMUM_JOINT_STEP));

}

bool CustomKinestheticTeacher::isBigJump(std::vector<double> myVec){
    bool bigJump =false;
    for (auto element: myVec)
        if (element > MAX_JOINT_MOVEMENT_ALLOWED_FOR_IK)
            bigJump =true;
    return bigJump;
}

std::vector<double> CustomKinestheticTeacher::scaleForcesTorques(std::vector<double> myVec){
    auto torqueIndexStart=myVec.size()/2;
    //weigh force and torque readings
    for (int i=0;i< torqueIndexStart; ++i)
        myVec.at(i) *= FORCES_MOVING_MULTIPLIER;
    for (int i=torqueIndexStart;i< myVec.size(); ++i)
        myVec.at(i) *= TORQUES_MOVING_MULTIPLIER;
    return myVec;
}

std::vector<double> CustomKinestheticTeacher::scaleJointCommands(std::vector<double> myVec){
    //weigh base and arm movements
    myVec.at(0)*= BASE_XY_MOVING_MULTIPLIER;
    myVec.at(1)*=BASE_XY_MOVING_MULTIPLIER;
    myVec.at(2)*=BASE_Z_MOVING_MULTIPLIER;
    for (auto i=3; i< myVec.size();i++)
        myVec.at(i)*=ARM_ALLJOINTS_MOVING_MULTIPLIER;
    return myVec;
}
Eigen::VectorXd CustomKinestheticTeacher::stdToEigenVec(std::vector<double> myVec){
    int n = myVec.size();
    Eigen::VectorXd temp(n);
    for (int i=0;i< n; ++i)
        temp(i)= myVec.at(i);
    return temp;
}

std::vector<double> CustomKinestheticTeacher::eigenToStdVec(Eigen::VectorXd myVec){
    int n = myVec.rows();
    std::vector<double> temp;
    for (int i=0;i< n; ++i)
        temp.push_back(myVec(i));
    return temp;

}
bool CustomKinestheticTeacher::isDifferentialCommandSafe(arma::vec diffCommand,arma::vec currentJointStates){
    bool okay=true;
    int checkTimesAhead=3;
    for (auto i = 1;i<=checkTimesAhead;i++)
        if (isColliding(armadilloToStdVec(i*diffCommand + currentJointStates)))
            okay=false;
    return okay;
}

bool CustomKinestheticTeacher::isColliding(std::vector<double> jointStates){
    auto pose = mvKin->computeFk(jointStates);
    return mvKin->isColliding(stdToArmadilloVec(jointStates),pose);
}


std::vector<double> CustomKinestheticTeacher::capVec(std::vector<double> input, double maxCap){

    for (int i =0;i< input.size();i++)
        input.at(i)= std::isnan(input.at(i)) || std::isinf(input.at(i)) ? 0.0 : sign(input.at(i)) * min(maxCap,sign(input.at(i)) * input.at(i));
    return input;

}

int CustomKinestheticTeacher::sign(double x){
    return (x>=0.0) ? 1: -1;
}



void CustomKinestheticTeacher::startTeaching(){
    teacherRunning=true;
}

void CustomKinestheticTeacher::stopTeaching(){
    teacherRunning=false;
}

void CustomKinestheticTeacher::teachingThreadHandler(){
    ros::Rate myRate(50);
    while(1){
        if (teacherRunning){
            generateNextCommand();

        }

        myRate.sleep();
    }

}


void CustomKinestheticTeacher::quit(){
    teacherRunning=false;
    robotinoQueue->stopQueue();
    moveThread->join();
    filterThread->join();
    qThread->join();
}
}
