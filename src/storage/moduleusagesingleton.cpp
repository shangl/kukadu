#include <limits>
#include <fstream>
#include <algorithm>
#include <boost/program_options.hpp>

#include <kukadu/utils/utils.hpp>
#include <kukadu/utils/kukadutokenizer.hpp>
#include <kukadu/storage/moduleusagesingleton.hpp>

using namespace std;

namespace kukadu {

    ModuleUsageSingleton::ModuleUsageSingleton() : storage(StorageSingleton::get()) {
        maxFunctionId = numeric_limits<int>::min();
        loadStatisticsProperties(resolvePath("$KUKADU_HOME/cfg/core/module_stat.list"));
    }

    ModuleUsageSingleton::~ModuleUsageSingleton() {

        long long int currentFakeTime = 0;
        for(auto& pooledFunctions : pooledFunctionCount)
            if(pooledFunctions.second)
                storePooledStatistics(pooledFunctions.first, currentFakeTime, true);

    }

    void ModuleUsageSingleton::startStatisticsModule() {
        statisticsActivated = true;
    }

    void ModuleUsageSingleton::stopStatisticsModule() {
        statisticsActivated = false;
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

                if(currentFunctionStorageId != ID_NOT_FOUND && currentFunctionStorageId > maxFunctionId)
                    maxFunctionId = currentFunctionStorageId;

            }

        }

        is.close();

        boost::program_options::options_description desc("kukadu properties");
        desc.add_options()("database.store_function_profile", boost::program_options::value<bool>()->required(), "");
        ifstream parseFile(resolvePath("$KUKADU_HOME/cfg/core/kukadu.prop").c_str(), std::ifstream::in);
        boost::program_options::variables_map vm;
        boost::program_options::store(boost::program_options::parse_config_file(parseFile, desc, true), vm);
        boost::program_options::notify(vm);
        statisticsActivated = vm["database.store_function_profile"].as<bool>();

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

        bool insertNewFunction = true;

        if(currentNamespace != "kukadu")
            throw KukaduException("(ModuleUsageSingleton) currently only the kukadu namespace is supported for function statistics");

        // assume only one class with the same name (currently only one namespace is supported)
        int modId = ID_NOT_FOUND;
        int classId = ID_NOT_FOUND;
        int namespaceId = ID_NOT_FOUND;

        vector<int> functionId;

        // that is very ugly --> fix at some point
        try { auto modIdTmp = storage.getCachedLabelId("software_modules", "id", "name", currentModule); modId = modIdTmp; } catch(KukaduException& ex) {}
        try { auto functionIdTmp = storage.getCachedLabelIds("software_functions", "id", "name", currentFunction); functionId = functionIdTmp; } catch(KukaduException& ex) {}
        try { auto classIdTmp = storage.getCachedLabelId("software_classes", "id", "name", currentClass); classId = classIdTmp; } catch(KukaduException& ex) {}
        try { auto namespaceIdTmp = storage.getCachedLabelId("software_namespaces", "id", "name", currentNamespace); namespaceId = namespaceIdTmp; } catch(KukaduException& ex) {}

        if(modId == ID_NOT_FOUND) {
            storage.executeStatementPriority("insert into software_modules(name) values('" + currentModule + "')");
            modId = storage.getCachedLabelId("software_modules", "id", "name", currentModule);
        }

        if(classId == ID_NOT_FOUND) {

            stringstream s;
            s << "insert into software_classes(module_id, name) values(" << modId << ", '" << currentClass << "')";
            storage.executeStatementPriority(s.str());
            classId = storage.getCachedLabelId("software_classes", "id", "name", currentClass);

        }

        if(namespaceId == ID_NOT_FOUND) {
            storage.executeStatementPriority("insert into software_namespaces(name) values('" + currentNamespace + "')");
            namespaceId = storage.getCachedLabelId("software_namespaces", "id", "name", currentNamespace);
        }

        // if function is in the database
        if(functionId.size()) {

            // if the id of a function with the same name has a different id --> it must differ in the namespace and/or the class --> otherwise there is an inconsistency
            if(std::find(functionId.begin(), functionId.end(), currentId) != functionId.end()) {

                string sel = "select fun.namespace_id as nam, fun.class_id as cla from software_functions as fun where fun.name = '" + currentFunction + "'";

                auto selResult = storage.executeQuery(sel);
                int resCount = 0;
                for(; selResult->next(); ++resCount) {

                    auto storedClassId = selResult->getInt("cla");
                    auto storedNamespaceId = selResult->getInt("nam");

                    if(storedClassId != classId || storedNamespaceId != namespaceId) {}
                    else if(storedClassId == classId && storedNamespaceId == namespaceId) {
                        // if function is already in database, do not insert it again
                        insertNewFunction = false;
                        ++resCount;
                        break;
                    } else
                        throw KukaduException("(ModuleUsageSingleton) function is already stored with a different id (check you property file)");

                }

                if(resCount == 0)
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
            storage.executeStatementPriority(s.str());

        }

        return currentId;

    }

    ModuleUsageSingleton& ModuleUsageSingleton::get() {
        static std::mutex m;
        m.lock();
            static ModuleUsageSingleton instance;
        m.unlock();
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

    void ModuleUsageSingleton::storePooledStatistics(const int& functionId, const long long& currentTime, const bool finalize) {

        // the function functionId is called the first time
        auto& pooledData = pooledFunctionDuration[functionId];
        auto& pooledCount = pooledFunctionCount[functionId];

        if(pooledData.first == 0) {

            pooledData.first = currentTime;
            pooledData.second = currentTime;
            pooledCount = 0;

        } else if((currentTime - pooledData.first) > poolingWindowSizes[functionId] || finalize) {

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

    long long int ModuleUsageSingleton::storeFunctionUsedStart(const int& functionId, const int& mode) {

        if(statisticsActivated) {

            static auto notFound = ID_NOT_FOUND;

            if(functionId != notFound) {

                static vector<long long int> prevUsage(maxFunctionId, 0);
                auto currentTime = getCurrentTime();

                // prevents duplicate entries with the same id and timestamp
                if(prevUsage[functionId - 1] != currentTime) {

                    if(mode == MODE_STD_STORAGE) {
                        stringstream s;
                        s << "insert into software_statistics_mode0(function_id, start_timestamp, end_timestamp) values(" << functionId << ", " << currentTime << ", NULL)";
                        storage.executeStatement(s.str());
                    } else if(mode == MODE_POOL_STORAGE)
                        storePooledStatistics(functionId, currentTime);

                    prevUsage[functionId - 1] = currentTime;

                    return currentTime;

                }

                return ID_NOT_FOUND;

            }

        }

        return ID_NOT_FOUND;

    }

    void ModuleUsageSingleton::storeFunctionUsedEnd(const int& functionId, const int& mode, long long& startTimestamp) {

        if(statisticsActivated && startTimestamp != ID_NOT_FOUND) {

            auto currentTime = getCurrentTime();

            stringstream s;
            s << "update software_statistics_mode0 set end_timestamp = " << currentTime << " where function_id = " << functionId << " and start_timestamp = " << startTimestamp;

            storage.executeStatement(s.str());

        }

    }

}
