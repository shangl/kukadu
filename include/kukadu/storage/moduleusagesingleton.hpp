#ifndef KUKADU_MODULEUSAGESINGLETON_H
#define KUKADU_MODULEUSAGESINGLETON_H

#include <map>
#include <string>
#include <utility>
#include <kukadu/utils/utils.hpp>
#include <kukadu/types/kukadutypes.hpp>
#include <kukadu/storage/storagesingleton.hpp>

#define KUKADU_MODULE_START_USAGE() ModuleUsageSingleton& mod = ModuleUsageSingleton::get(); static int functionId = mod.loadFunctionId(__PRETTY_FUNCTION__); long long int startTimeStamp = mod.storeFunctionUsedStart(functionId);
#define KUKADU_MODULE_END_USAGE() mod.storeFunctionUsedEnd(functionId, startTimeStamp);

namespace kukadu {

    class ModuleUsageSingleton : private TimedObject {

    private:

#ifndef USEBOOST
        static auto constexpr ID_NOT_FOUND{-1};
#else
        static const int ID_NOT_FOUND = -1;
#endif

        std::map<int, std::string> moduleFunctionMap;
        std::map<std::string, std::pair<int, int> > supportedFunctions;

        StorageSingleton& storage;

        ModuleUsageSingleton();

        std::string combineIdentifiers(const std::string& currentNamespace, const std::string& currentClass, const std::string& currentFunction);

        void loadStatisticsProperties(const std::string file);
        void addNewFunction(const std::string& currentModule, const std::string& currentNamespace, const std::string& currentClass, const std::string& currentFunction, const int& currentId, const int& currentMode);

        int checkConsistencyOrInsert(const std::string& currentModule, const std::string& currentNamespace, const std::string& currentClass, const std::string& currentFunction, const int& currentId, const int& currentMode);

    public :

        static ModuleUsageSingleton& get();

        long long storeFunctionUsedStart(const int& functionId);
        void storeFunctionUsedEnd(const int& functionId, long long int& startTimestamp);

        int loadFunctionId(std::string prettyFunctionName);

    };

}

#endif
