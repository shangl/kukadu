#ifndef KUKADU_MODULEUSAGESINGLETON_H
#define KUKADU_MODULEUSAGESINGLETON_H

#include <map>
#include <string>
#include <utility>
#include <kukadu/utils/utils.hpp>
#include <kukadu/storage/storagesingleton.hpp>

#define KUKADU_MODULE_START_USAGE() ModuleUsageSingleton& mod = ModuleUsageSingleton::get(); static int functionId = mod.loadFunctionId(__PRETTY_FUNCTION__); ModuleUsageSingleton::get().storeFunctionUsedStart(functionId);
#define KUKADU_MODULE_END_USAGE() mod.storeFunctionUsedEnd(functionId);

namespace kukadu {

    class ModuleUsageSingleton {

    private:

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

        void storeFunctionUsedStart(const int& functionId);
        void storeFunctionUsedEnd(const int& functionId);

        int loadFunctionId(std::string prettyFunctionName);

    };

}

#endif
