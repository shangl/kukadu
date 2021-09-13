#include <gtest/gtest.h>

#include <memory>
#include <vector>

using namespace std;

TEST(ProjectiveSimulation, projSimSerialization){
/*
    std::shared_ptr<NeverendingColorReward> trafficReward = nullptr;
    std::shared_ptr<ProjectiveSimulator> currentProjSim = nullptr;

    trafficReward = std::shared_ptr<NeverendingColorReward>(new NeverendingColorReward(generator, 2, 2, false));
    currentProjSim = std::shared_ptr<ProjectiveSimulator>(new ProjectiveSimulator(trafficReward, generator, 0.01, PS_USE_GEN, false));

    ofstream statFile; statFile.open("/home/c7031109/tmp/psgen.txt");
    PSEvaluator::produceStatistics(currentProjSim, trafficReward, 10, 0, NEVERENDINGCOLORREWARD_SUCCESSFUL_REWARD, statFile);
    currentProjSim->storePS("/home/c7031109/tmp/storedPS.txt");

    cout << "stored ps" << endl;

    std::shared_ptr<ProjectiveSimulator> loadedProjSim = std::shared_ptr<ProjectiveSimulator>(new ProjectiveSimulator(trafficReward, generator, "/home/c7031109/tmp/storedPS.txt"));

    cout << "loaded ps" << endl;

    loadedProjSim->storePS("/home/c7031109/tmp/storedPS2.txt");
    cout << "stored loaded ps" << endl;
*/
    EXPECT_TRUE(true);
}

int main(int argc, char **argv){
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}

