#include <kukadu/storage/moduleusagesingleton.hpp>
#include <kukadu/storage/storagesingleton.hpp>
#include <boost/program_options.hpp>
#include <kukadu/utils/utils.hpp>
#include <algorithm>
#include <ros/ros.h>
#include <sstream>

using namespace std;

namespace kukadu {

    StorageSingleton::StorageSingleton() {

        boost::program_options::options_description desc("kukadu properties");
        desc.add_options()
                ("kukadu.databaseurl", boost::program_options::value<string>()->required(), "")
                ("kukadu.database", boost::program_options::value<string>()->required(), "")
                ("kukadu.user", boost::program_options::value<string>()->required(), "")
                ("kukadu.pw", boost::program_options::value<string>()->required(), "")
                ("database.storage_caching", boost::program_options::value<bool>()->required(), "")
                ("database.storage_caching_size", boost::program_options::value<int>()->required(), "")
                ("database.storage_cache_polling_rate", boost::program_options::value<int>()->required(), "")
                ("database.storage_caching_bulk_size", boost::program_options::value<int>()->required(), "")
                ;
        ifstream parseFile(resolvePath("$KUKADU_HOME/cfg/core/kukadu.prop").c_str(), std::ifstream::in);
        boost::program_options::variables_map vm;
        boost::program_options::store(boost::program_options::parse_config_file(parseFile, desc, true), vm);
        boost::program_options::notify(vm);

        driver = get_driver_instance();
        con = driver->connect(vm["kukadu.databaseurl"].as<std::string>(), vm["kukadu.user"].as<std::string>(), vm["kukadu.pw"].as<std::string>());

        databaseName = vm["kukadu.database"].as<std::string>();

        if(!isInstalled())
            install();

        executeStatement("use " + databaseName);

        cacheDemonRunning = false;
        useCaching = vm["database.storage_caching"].as<bool>();
        maxCacheSize = vm["database.storage_caching_size"].as<int>();
        cacheDemonPollingRate = vm["database.storage_cache_polling_rate"].as<int>();
        cacheBulkSize = vm["database.storage_caching_bulk_size"].as<int>();

        if(useCaching) {
            cacheDemonRunning = true;
            cacheDemonThread = kukadu_thread(&StorageSingleton::runStatementsDemon, this);
        }

    }

    void StorageSingleton::installDirectory(std::string directory) {

        vector<string> installFiles = getFilesInDirectory(resolvePath(directory));
        std::sort(installFiles.begin(), installFiles.end());
        for(auto& installFile : installFiles) {

            if(!isDirectory(installFile)) {
                ifstream t(resolvePath(directory + "/" + installFile));
                string sqlStr((std::istreambuf_iterator<char>(t)),
                                 std::istreambuf_iterator<char>());
                executeStatement(sqlStr);
            } else if(installFile != "." && installFile != "..")
                installDirectory(directory + "/" + installFile);

        }

    }

    void StorageSingleton::install() {

        string installDir = "$KUKADU_HOME/install";
        executeStatement("use " + databaseName);

        char answer = 'n';
        cout << "kukadu is currently not properly installed. do you want to install it? (y = yes, of course, n = no)" << endl;
        cin >> answer;
        if(answer == 'y')
            installDirectory(installDir);
        else
            throw KukaduException("(StorageSingleton) you chose not to install kukadu. core functionality might be affected.");
    }

    bool StorageSingleton::isInstalled() {

        executeStatement("use information_schema");
        auto existsSet = executeQuery("select count(*) as cnt from tables where table_schema = '" + databaseName + "' and table_name = 'skills'");
        existsSet->next();
        if(existsSet->getInt("cnt") > 0)
            return true;
        return false;

    }

    StorageSingleton& StorageSingleton::get() {
        static StorageSingleton instance;
        return instance;
    }

    long long int StorageSingleton::getNextIdInTable(std::string table, std::string idCol) {

        auto key = table + "+++" + idCol;
        auto el = idsMap.find(key);

        // check if id is already in map --> increment id and return it
        if(el != idsMap.end()) {
            auto& id = *el;
            return ++(id.second);
        }
        // else find highest id, increment it and insert new value in map
        else {
            string maxIdQuery = "select max(" + idCol + ") as max_val from " + table;
            auto maxIdRes = executeQuery(maxIdQuery);
            long long int maxId;
            if(maxIdRes->next())
                // thats a bit nasty (need to do that better at some point)
                maxId = maxIdRes->getUInt64("max_val");
            else
                throw KukaduException("(StorageSingleton) retrieving maximum id failed");

            idsMap[key] = ++maxId;
            return maxId;

        }

    }

    bool StorageSingleton::checkIdExists(std::string table, std::string idCol, int val) {

        stringstream s;
        s << "select count(*) as c from " << table << " where " << idCol << " = " << val;
        auto res = executeQuery(s.str());
        if(res->next()) {
            return (res->getInt("c")) ? true : false;
        } else
            throw KukaduException("(StorageSingleton) sql problem occured");

    }

    std::vector<int> StorageSingleton::getCachedLabelIds(string table, string labelIdCol, string labelCol, string label, string additionalWhere) {

        auto key = table + "+++" + labelCol + "+++" + label + "+++" + additionalWhere;
        auto el = labelIdsMap.find(key);

        // check if label is already in map --> increment id and return it
        if(el != labelIdsMap.end()) {

            auto& id = *el;
            return id.second;

        }

        // else find highest id, insert new label in map
        else {

            stringstream s;
            s << "select distinct(" << labelIdCol << ") from " << table << " where " << labelCol << " = \"" << label << "\"" << ((additionalWhere == "") ? "" : " and ") << additionalWhere << ";";
            auto labelQuery = s.str();

            auto labelResSet = executeQuery(labelQuery);

            bool firstRun = true;
            while(labelResSet->next()) {

                if(firstRun) {
                    labelIdsMap[key] = vector<int>();
                    firstRun = false;
                }

                // thats a bit nasty (need to do that better at some point)
                labelIdsMap[key].push_back(labelResSet->getInt(labelIdCol));

            }

            // if no label was found
            if(firstRun)
                throw KukaduException("(StorageSingleton) retrieving label id failed");


            return labelIdsMap[key];

        }

        return {};

    }

    int StorageSingleton::getCachedLabelId(std::string table, std::string labelIdCol, std::string labelCol, std::string label, string additionalWhere) {
        return getCachedLabelIds(table, labelIdCol, labelCol, label, additionalWhere).at(0);
    }

    void StorageSingleton::waitForEmptyCache() {

        static ros::Rate r(cacheDemonPollingRate);
        while(!cachedStatements.empty())
            r.sleep();

    }

    std::string StorageSingleton::getCachedLabel(std::string table, std::string labelIdCol, std::string labelCol, int labelId) {

        auto key = table + "+++" + labelCol;
        auto el = labelsMap.find(key);

        // check if label is already in map --> increment id and return it
        if(el != labelsMap.end()) {
            auto& id = *el;
            return id.second;
        }
        // else find highest id, insert new label in map
        else {
            stringstream s;
            s << "select " << labelCol << " from " << table << " where " << labelIdCol << " = " << labelId;
            auto labelQuery = s.str();
            auto labelResSet = executeQuery(labelQuery);
            string labelRes;
            if(labelResSet->next())
                // thats a bit nasty (need to do that better at some point)
                labelRes = labelResSet->getString(labelCol);
            else
                throw KukaduException("(StorageSingleton) retrieving label failed");

            labelsMap[key] = labelRes;
            return labelRes;

        }

    }

    void StorageSingleton::runStatementsDemon() {

        static ros::Rate pollingRate(cacheDemonPollingRate);
        while(cacheDemonRunning) {

            // if there is something in the cache --> go and store it
            if(!cachedStatements.empty()) {

                statementsMutex.lock();

                    vector<string> cacheBulk;
                    for(int i = 0; i < cacheBulkSize && !cachedStatements.empty(); ++i) {
                        cacheBulk.push_back(cachedStatements.front());
                        cachedStatements.pop();
                    }

                statementsMutex.unlock();

                try {
                    actualExecuteStatements(cacheBulk);
                } catch(sql::SQLException& ex) {
                    // catch the exception to prevent the thread from breaking down
                    // cout << ex.what() << endl;
                }

            } else
                // if not, just sleep to not overload the cpu
                pollingRate.sleep();

        }


    }

    void StorageSingleton::actualExecuteStatements(const std::vector<std::string>& statements) {

        connectionMutex.lock();

            auto stmt = con->createStatement();
            for(auto& sql : statements)
                try { stmt->execute(sql); } catch(sql::SQLException& ex) { delete stmt; connectionMutex.unlock(); throw ex; }
            delete stmt;

        connectionMutex.unlock();

    }

    void StorageSingleton::executeStatement(std::string sql) {

        static ros::Rate r(50);
        if(useCaching) {

            // block until cache has recovered
            while(cachedStatements.size() > maxCacheSize)
                r.sleep();

            statementsMutex.lock();
                cachedStatements.push(sql);
            statementsMutex.unlock();

        } else
            actualExecuteStatements({sql});

    }

    void StorageSingleton::executeStatements(std::vector<std::string> sqls) {

        static ros::Rate r(50);
        if(useCaching) {

            // block until cache has recovered
            while(cachedStatements.size() > maxCacheSize)
                r.sleep();

            statementsMutex.lock();
                for(auto& sql : sqls)
                    cachedStatements.push(sql);
            statementsMutex.unlock();

        } else
            actualExecuteStatements(sqls);

    }

    KUKADU_SHARED_PTR<sql::ResultSet> StorageSingleton::executeQuery(std::string sql) {

        connectionMutex.lock();
            auto stmt = con->createStatement();
            auto retSet = KUKADU_SHARED_PTR<sql::ResultSet>(stmt->executeQuery(sql));
            // is it allowed to delete the statement before deleteing res? (we will find out :) )
            delete stmt;
        connectionMutex.unlock();
        return retSet;

    }

    StorageSingleton::~StorageSingleton() {
        cacheDemonRunning = false;
        if(cacheDemonThread.joinable())
            cacheDemonThread.join();
        delete con;
    }

}
