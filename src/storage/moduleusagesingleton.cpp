#include <fstream>
#include <algorithm>
#include <kukadu/utils/utils.hpp>
#include <kukadu/utils/kukadutokenizer.hpp>
#include <kukadu/storage/moduleusagesingleton.hpp>

using namespace std;

namespace kukadu {

    ModuleUsageSingleton::ModuleUsageSingleton() : storage(StorageSingleton::get()) {
        loadStatisticsProperties(resolvePath("$KUKADU_HOME/cfg/core/module_stat.list"));
    }

    void ModuleUsageSingleton::loadStatisticsProperties(const std::string file) {

        ifstream is;
        is.open(file.c_str());

        string line = "";
        KukaduTokenizer tok(line);

        int currentFunctionStorageMode = 0;
        int currentFunctionStorageId = ID_NOT_FOUND;

        string nextToken;

        string currentModule = "general";
        string currentNamespace = "";
        string currentClass = "";
        string currentFunction = "";

        while(getline(is, line)) {
            // if not an empty line
            if(!all_of(line.begin(), line.end(), [](char c){
                       return std::isspace(static_cast<unsigned char>(c));
                    })) {

                tok = KukaduTokenizer(line);
                nextToken = tok.next();

                switch(nextToken[0]) {
                // new module
                case '[':
                    currentModule = nextToken.substr(1, nextToken.length() - 2);
                    currentFunctionStorageMode = 0;
                    currentFunctionStorageId = ID_NOT_FOUND;
                    currentNamespace = "";
                    currentClass = "";
                    currentFunction = "";
                    break;
                // new namespace
                case '-':
                    currentNamespace = nextToken.substr(1, nextToken.length() - 1);
                    currentFunctionStorageMode = 0;
                    currentFunctionStorageId = ID_NOT_FOUND;
                    currentClass = "";
                    currentFunction = "";
                    break;
                case '+':
                    currentClass = nextToken.substr(1, nextToken.length() - 1);
                    currentFunctionStorageMode = 0;
                    currentFunctionStorageId = ID_NOT_FOUND;
                    currentFunction = "";
                    break;
                case '#':
                    currentFunction = nextToken.substr(1, nextToken.length() - 1);;
                    currentFunctionStorageId = atoi(tok.next().c_str());
                    currentFunctionStorageMode = atoi(tok.next().c_str());

                    if(currentFunctionStorageMode == MODE_POOL_STORAGE)
                        poolingWindowSizes[currentFunctionStorageId] = atoi(tok.next().c_str());

                    addNewFunction(currentModule, currentNamespace, currentClass, currentFunction, currentFunctionStorageId, currentFunctionStorageMode);
                    break;
                // ignore comment
                case '/':
                    break;
                }

            }
        }

        is.close();

    }

    std::string ModuleUsageSingleton::combineIdentifiers(const std::string& currentNamespace, const std::string& currentClass, const std::string& currentFunction) {
        return ((currentNamespace == "") ? "" : (currentNamespace + "::")) + ((currentClass == "") ? "" : (currentClass + "::")) + currentFunction;
    }

    void ModuleUsageSingleton::addNewFunction(const std::string& currentModule, const std::string& currentNamespace, const std::string& currentClass, const std::string& currentFunction, const int& currentId, const int& currentMode) {

        moduleFunctionMap[currentId] = currentModule;

        std::string identifier = combineIdentifiers(currentNamespace, currentClass, currentFunction);
        if(supportedFunctions.find(identifier) == supportedFunctions.end()) {

            checkConsistencyOrInsert(currentModule, currentNamespace, currentClass, currentFunction, currentId, currentMode);
            supportedFunctions[identifier] = {currentId, currentMode};
            if(currentMode == MODE_POOL_STORAGE) {
                pooledFunctionDuration[currentId] = {0, 0};
                pooledFunctionCount[currentId] = 0;
            }

        } else
            throw KukaduException("(ModuleUsageSingleton) duplicate function signature (check the property file)");

    }

    int ModuleUsageSingleton::checkConsistencyOrInsert(const std::string& currentModule, const std::string& currentNamespace, const std::string& currentClass, const std::string& currentFunction, const int& currentId, const int& currentMode) {

        bool insertNewFunction = false;

        int modId = ID_NOT_FOUND;
        int classId = ID_NOT_FOUND;
        int functionId = ID_NOT_FOUND;
        int namespaceId = ID_NOT_FOUND;

        // that is very ugly --> fix at some point
        try { modId = storage.getCachedLabelId("software_modules", "id", "name", currentModule); } catch(KukaduException& ex) {}
        try { functionId = storage.getCachedLabelId("software_functions", "id", "name", currentFunction); } catch(KukaduException& ex) {}
        try { classId = storage.getCachedLabelId("software_classes", "id", "name", currentClass); } catch(KukaduException& ex) {}
        try { namespaceId = storage.getCachedLabelId("software_namespaces", "id", "name", currentNamespace); } catch(KukaduException& ex) {}

        if(modId == ID_NOT_FOUND) {
            storage.executeStatement("insert into software_modules(name) values('" + currentModule + "')");
            storage.waitForEmptyCache();
            modId = storage.getCachedLabelId("software_modules", "id", "name", currentModule);
        }

        if(classId == ID_NOT_FOUND) {
            stringstream s;
            s << "insert into software_classes(module_id, name) values(" << modId << ", '" << currentClass << "')";

            storage.executeStatement(s.str());
            storage.waitForEmptyCache();
            classId = storage.getCachedLabelId("software_classes", "id", "name", currentClass);
        }

        if(namespaceId == ID_NOT_FOUND) {
            storage.executeStatement("insert into software_namespaces(name) values('" + currentNamespace + "')");
            storage.waitForEmptyCache();
            namespaceId = storage.getCachedLabelId("software_namespaces", "id", "name", currentNamespace);
        }

        // if function is in the database
        if(functionId != ID_NOT_FOUND) {

            // if the id of a function with the same name has a different id --> it must differ in the namespace and/or the class --> otherwise there is an inconsistency
            if(functionId != currentId) {

                string sel = "select fun.namespace_id as nam, fun.class_id as cla from software_functions as fun where fun.name = '" + currentFunction + "'";

                auto selResult = storage.executeQuery(sel);
                if(selResult->next()) {

                    auto storedClassId = selResult->getInt("cla");
                    auto storedNamespaceId = selResult->getInt("nam");

                    if(storedClassId != classId || storedNamespaceId != namespaceId)
                        insertNewFunction = true;
                    else if(storedClassId == classId && storedNamespaceId == namespaceId) {}
                    else
                        throw KukaduException("(ModuleUsageSingleton) function is already stored with a different id (check you property file)");

                } else
                    throw KukaduException("(ModuleUsageSingleton) sql problem occurred");

            }

        }
        // if it is not in the database --> insert it
        else
            insertNewFunction = true;

        if(insertNewFunction) {

            // if the name wasn't found in the query above, but the id already exists --> there is some inconsistency between database and property file
            if(storage.checkIdExists("software_functions", "id", currentId))
                throw KukaduException("(ModuleUsageSingleton) function id for one function was already used for another one");

            // if program reaches this point, function can be inserted safely
            stringstream s;
            s << "insert into software_functions(id, namespace_id, class_id, name, storage_mode) values(" << currentId << ", " << namespaceId << ", " << classId << ", '" << currentFunction << "', " << currentMode << ")";
            storage.executeStatement(s.str());
            storage.waitForEmptyCache();
            functionId = currentId;


        }

        return functionId;

    }

    ModuleUsageSingleton& ModuleUsageSingleton::get() {
        static ModuleUsageSingleton instance;
        return instance;
    }

    std::pair<int, int> ModuleUsageSingleton::loadFunctionId(std::string prettyFunctionName) {

        auto signature = readFunctionSignature(prettyFunctionName);
        auto identifier = combineIdentifiers(signature.at(0), signature.at(1), signature.at(2));
        if(supportedFunctions.find(identifier) != supportedFunctions.end())
            return supportedFunctions[identifier];

        // if not listed in property file --> return ID_NOT_FOUND
        auto notFound = ID_NOT_FOUND;
        return {notFound, notFound};

    }

    long long int ModuleUsageSingleton::storeFunctionUsedStart(const int& functionId, const int& mode) {

        static auto notFound = ID_NOT_FOUND;

        if(functionId != notFound) {

            auto currentTime = getCurrentTime();

            if(mode == MODE_STD_STORAGE) {
                stringstream s;
                s << "insert into software_statistics_mode0(function_id, start_timestamp, end_timestamp) values(" << functionId << ", " << currentTime << ", NULL)";
                storage.executeStatement(s.str());
            } else if(mode == MODE_POOL_STORAGE) {

                // the function functionId is called the first time
                auto& pooledData = pooledFunctionDuration[functionId];
                auto& pooledCount = pooledFunctionCount[functionId];

                if(pooledData.first == 0) {

                    pooledData.first = currentTime;
                    pooledData.second = currentTime;
                    pooledCount = 0;

                } else if((currentTime - pooledData.first) > poolingWindowSizes[functionId]) {

                    // if the window is already bigger than the window --> store it to the database and make a new window
                    stringstream s;
                    s << "insert into software_statistics_mode1(function_id, start_timestamp, end_timestamp, cnt) values(" << functionId << ", " << pooledData.first << ", " << pooledData.second << ", " << pooledCount << ")";
                    storage.executeStatement(s.str());
                    pooledData.first = currentTime;
                    pooledData.second = currentTime;
                    pooledCount = 0;

                } else {

                    // if the window is still open, count more
                    ++pooledCount;
                    pooledData.second = currentTime;

                }


            }

            return currentTime;

        }

        return 0;

    }

    void ModuleUsageSingleton::storeFunctionUsedEnd(const int& functionId, const int& mode, long long& startTimestamp) {

        auto currentTime = getCurrentTime();

        stringstream s;
        s << "update software_statistics_mode0 set end_timestamp = " << currentTime << " where function_id = " << functionId << " and start_timestamp = " << startTimestamp;

        storage.executeStatement(s.str());

    }

}
