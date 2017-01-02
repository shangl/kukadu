#ifndef KUKADU_MODULEUSAGESINGLETON_H
#define KUKADU_MODULEUSAGESINGLETON_H

#include <map>
#include <string>
#include <utility>
#include <kukadu/utils/utils.hpp>
#include <kukadu/types/kukadutypes.hpp>
#include <kukadu/storage/storagesingleton.hpp>

#define KUKADU_MODULE_START_USAGE() ModuleUsageSingleton& mod = ModuleUsageSingleton::get(); static std::pair<int, int> statData = mod.loadFunctionId(__PRETTY_FUNCTION__); \
    static int functionId = statData.first; static int functionMode = statData.second; \
    long long int startTimeStamp = mod.storeFunctionUsedStart(functionId, functionMode);

#define KUKADU_MODULE_END_USAGE() if(startTimeStamp) mod.storeFunctionUsedEnd(functionId, functionMode, startTimeStamp);

namespace kukadu {

    class ModuleUsageSingleton : private TimedObject {

    private:

#ifndef USEBOOST
        static auto constexpr ID_NOT_FOUND = -1;
        static auto constexpr MODE_STD_STORAGE = 0;
        static auto constexpr MODE_POOL_STORAGE = 1;
#else
        static const int ID_NOT_FOUND = -1;
        static const int MODE_STD_STORAGE = 0;
        static const int MODE_POOL_STORAGE = 1;
#endif

        bool statisticsActivated;

        int maxFunctionId;

        std::map<int, int> poolingWindowSizes;
        std::map<int, int> pooledFunctionCount;
        std::map<int, std::string> moduleFunctionMap;
        std::map<std::string, std::pair<int, int> > supportedFunctions;
        std::map<int, std::pair<long long int, long long int> > pooledFunctionDuration;

        StorageSingleton& storage;

        ModuleUsageSingleton();
        ~ModuleUsageSingleton();

        std::string combineIdentifiers(const std::string& currentNamespace, const std::string& currentClass, const std::string& currentFunction);

        void storePooledStatistics(const int& functionId, const long long int& currentTime, const bool finalize = false);

        void loadStatisticsProperties(const std::string file);
        void addNewFunction(const std::string& currentModule, const std::string& currentNamespace, const std::string& currentClass, const std::string& currentFunction, const int& currentId, const int& currentMode);

        int checkConsistencyOrInsert(const std::string& currentModule, const std::string& currentNamespace, const std::string& currentClass, const std::string& currentFunction, const int& currentId, const int& currentMode);

    public :

        static ModuleUsageSingleton& get();

        void startStatisticsModule();
        void stopStatisticsModule();

        long long storeFunctionUsedStart(const int& functionId, const int& mode);
        void storeFunctionUsedEnd(const int& functionId, const int& mode, long long int& startTimestamp);

        std::pair<int, int> loadFunctionId(std::string prettyFunctionName);

    };

}

#endif
