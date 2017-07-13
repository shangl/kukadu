#ifndef KUKADU_STORAGESINGLETON_H
#define KUKADU_STORAGESINGLETON_H

#include <map>
#include <queue>
#include <string>
#include <vector>
#include <iostream>
#include <kukadu/types/kukadutypes.hpp>

#include <mysql_connection.h>
#include <cppconn/driver.h>
#include <cppconn/exception.h>
#include <cppconn/resultset.h>
#include <cppconn/statement.h>

namespace kukadu {

    class StorageSingleton {

    private:

        bool useCaching;
        int maxCacheSize;
        kukadu_mutex statementsMutex;
        std::queue<std::string> cachedStatements;

        // required because connection is not thread safe
        kukadu_mutex connectionMutex;

        bool cacheDemonRunning;
        int cacheDemonPollingRate;
        int cacheBulkSize;
        kukadu_thread cacheDemonThread;

        sql::Connection* con;
        sql::Driver* driver;

        std::string databaseName;

        std::map<std::string, long long int> idsMap;

        std::map<std::string, std::vector<int> > labelIdsMap;
        std::map<std::string, std::string> labelsMap;

        StorageSingleton();

        void runStatementsDemon();
        void installDirectory(std::string directory);
        void actualExecuteStatements(const std::vector<std::string>& statements);

    public:

        static StorageSingleton& get();

        long long int getNextIdInTable(std::string table, std::string idCol);

        bool checkTableExists(std::string table);
        bool checkIdExists(std::string table, std::string idCol, int val);
        bool checkLabelExists(std::string table, std::string labelCol, std::string val);

        int getCachedLabelId(std::string table, std::string labelIdCol, std::string labelCol, std::string label, std::string additionalWhere = "");
        std::vector<int> getCachedLabelIds(std::string table, std::string labelIdCol, std::string labelCol, std::string label, std::string additionalWhere = "");

        std::string getCachedLabel(std::string table, std::string labelIdCol, std::string labelCol, int labelId);

        void executeStatementPriority(std::string sql);

        void executeStatement(std::string sql);
        void executeStatements(std::vector<std::string> sqls);
        KUKADU_SHARED_PTR<sql::ResultSet> executeQuery(std::string sql);

        void waitForEmptyCache();

        void install(std::string directory);

        bool isInstalled(std::string directory);

        ~StorageSingleton();

    };

    // for convenience to avoid duplicate code for all classes that contain a storage singleton
    // (I want to pass the singleton explicitely just to make the dependency more clear)
    class StorageHolder {

    private:

        StorageSingleton& storage;

    public:

        StorageHolder(StorageSingleton& dbStorage);
        StorageSingleton& getStorage();

    };

}

#endif
