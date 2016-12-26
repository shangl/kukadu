#include <fstream>
#include <algorithm>
#include <kukadu/utils/utils.hpp>
#include <kukadu/utils/kukadutokenizer.hpp>
#include <kukadu/storage/moduleusagesingleton.hpp>

using namespace std;

namespace kukadu {

/*
 *
 * // how to do the rest
// --> create a singleton that reads the function names and id from a file
// --> initialize static variable with id value (and corresponding module id)
// --> the same singleton is used to store usage information and maybe some pooling into the database
// --> write a preprocessor macro that encapsulates all of this
// --> add it to every function
std::map<std::string, int> supportedFunctions = {{"asdf", 1}, {"blub", 2}, {"main", 3}};
*/

    ModuleUsageSingleton::ModuleUsageSingleton() : storage(StorageSingleton::get()) {
        loadStatisticsProperties(resolvePath("$KUKADU_HOME/cfg/core/module_stat.list"));
    }

    void ModuleUsageSingleton::loadStatisticsProperties(const std::string file) {

        ifstream is;
        is.open(file.c_str());

        string line = "";
        KukaduTokenizer tok(line);

        int currentFunctionStorageMode = 0;
        int currentFunctionStorageId = -1;

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
                    currentFunctionStorageId = -1;
                    currentNamespace = "";
                    currentClass = "";
                    currentFunction = "";
                    break;
                // new namespace
                case '-':
                    currentNamespace = nextToken.substr(1, nextToken.length() - 1);
                    currentFunctionStorageMode = 0;
                    currentFunctionStorageId = -1;
                    currentClass = "";
                    currentFunction = "";
                    break;
                case '+':
                    currentClass = nextToken.substr(1, nextToken.length() - 1);
                    currentFunctionStorageMode = 0;
                    currentFunctionStorageId = -1;
                    currentFunction = "";
                    break;
                case '#':
                    currentFunction = nextToken.substr(1, nextToken.length() - 1);;
                    currentFunctionStorageId = atoi(tok.next().c_str());
                    currentFunctionStorageMode = atoi(tok.next().c_str());
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

        } else
            throw KukaduException("(ModuleUsageSingleton) duplicate function signature (check the property file)");


    }

    int ModuleUsageSingleton::checkConsistencyOrInsert(const std::string& currentModule, const std::string& currentNamespace, const std::string& currentClass, const std::string& currentFunction, const int& currentId, const int& currentMode) {

        bool insertNewFunction = false;

        int modId = -1;
        int classId = -1;
        int functionId = -1;
        int namespaceId = -1;

        // that is very ugly --> fix at some point
        try { modId = storage.getCachedLabelId("software_modules", "id", "name", currentModule); } catch(KukaduException& ex) {}
        try { functionId = storage.getCachedLabelId("software_functions", "id", "name", currentFunction); } catch(KukaduException& ex) {}
        try { classId = storage.getCachedLabelId("software_classes", "id", "name", currentClass); } catch(KukaduException& ex) {}
        try { namespaceId = storage.getCachedLabelId("software_namespaces", "id", "name", currentNamespace); } catch(KukaduException& ex) {}

        if(modId == -1) {
            storage.executeStatement("insert into software_modules(name) values('" + currentModule + "')");
            storage.waitForEmptyCache();
            modId = storage.getCachedLabelId("software_modules", "id", "name", currentModule);
        }

        if(classId == -1) {
            stringstream s;
            s << "insert into software_classes(module_id, name) values(" << modId << ", '" << currentClass << "')";

            storage.executeStatement(s.str());
            storage.waitForEmptyCache();
            classId = storage.getCachedLabelId("software_classes", "id", "name", currentClass);
        }

        if(namespaceId == -1) {
            storage.executeStatement("insert into software_namespaces(name) values('" + currentNamespace + "')");
            storage.waitForEmptyCache();
            namespaceId = storage.getCachedLabelId("software_namespaces", "id", "name", currentNamespace);
        }

        // if function is in the database
        if(functionId != -1) {

            // if the id of a function with the same name has a different id --> it must differ in the namespace and/or the class --> otherwise there is an inconsistency
            if(functionId != currentId) {

                string sel = "select fun.namespace_id as nam, fun.class_id as cla from software_functions as fun where fun.name = '" + currentFunction + "'";

                auto selResult = storage.executeQuery(sel);
                if(selResult->next()) {

                    auto storedClassId = selResult->getInt("cla");
                    auto storedNamespaceId = selResult->getInt("name");

                    if(storedClassId != classId || storedNamespaceId != namespaceId)
                        insertNewFunction = true;
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

    int ModuleUsageSingleton::loadFunctionId(std::string prettyFunctionName) {

        auto signature = readFunctionSignature(prettyFunctionName);
        //cout << signature.at(0) << " " << signature.at(1) << " " << signature.at(2) << endl;

        return 0;

    }

    void ModuleUsageSingleton::storeFunctionUsedStart(const int& functionId) {

    }

    void ModuleUsageSingleton::storeFunctionUsedEnd(const int& functionId) {

    }

}
