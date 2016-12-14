#ifndef KUKADU_STORAGESINGLETON
#define KUKADU_STORAGESINGLETON

#include <map>
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

    sql::Connection* con;
    sql::Driver* driver;

    std::string databaseName;

    std::map<std::string, long long int> idsMap;

    std::map<std::string, int> labelIdsMap;
    std::map<std::string, std::string> labelsMap;

    StorageSingleton();

    void installDirectory(std::string directory);

public:

    static StorageSingleton& get();

    long long int getNextIdInTable(std::string table, std::string idCol);

    int getCachedLabelId(std::string table, std::string labelIdCol, std::string labelCol, std::string label, std::string additionalWhere = "");
    std::string getCachedLabel(std::string table, std::string labelIdCol, std::string labelCol, int labelId);

    void executeStatement(std::string sql);
    void executeStatements(std::vector<std::string> sqls);
    KUKADU_SHARED_PTR<sql::ResultSet> executeQuery(std::string sql);

    void install();

    bool isInstalled();

    ~StorageSingleton();

};

}

#endif
