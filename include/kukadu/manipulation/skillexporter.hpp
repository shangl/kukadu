#ifndef KUKADU_SKILLEXPORTER_H
#define KUKADU_SKILLEXPORTER_H

#include <string>
#include <vector>
#include <utility>
#include <kukadu/robot/hardware.hpp>
#include <kukadu/types/kukadutypes.hpp>
#include <kukadu/storage/storagesingleton.hpp>

namespace kukadu {

    class SkillExporter : public kukadu::StorageHolder {

    public:

        SkillExporter(kukadu::StorageSingleton& storage);

        std::vector<std::pair<int, std::string> > getSkills();
        std::vector<std::string> getSkillHardware(int skillId);

        // returns a tuple of executions containing:
        // - long long int startTime
        // - long long int endTime (endTime is 0, if the skill wasn't completed)
        // - bool success
        std::vector<std::tuple<long long int, long long int, bool> > getSkillExecutions(int skillId, long long int startTime = 0, long long int endTime = 0);
        std::vector<std::tuple<long long int, long long int, bool> > getSkillExecutions(std::string skillName, long long int startTime = 0, long long int endTime = 0);

        void exportSkillExecutions(int skillId, long long int startTime, long long int endTime, std::string folder, bool exportModuleStatistics = true, bool compress = true);

        std::pair<std::vector<int>, std::vector<arma::mat> > loadExecutions(std::string directory, std::vector<long long int>& startTimes, std::vector<long long int>& endTimes, long long int& timeStep);

        arma::mat computeAllDistances(std::vector<arma::mat>& skillData, bool giveFeedback = false);
        double computeDistance(arma::mat& m1, arma::mat& m2);

    };

}

#endif
