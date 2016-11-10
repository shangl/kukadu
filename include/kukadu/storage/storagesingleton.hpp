#ifndef KUKADU_STORAGESINGLETON
#define KUKADU_STORAGESINGLETON

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

    StorageSingleton();

    void installDirectory(std::string directory);

public:

    static StorageSingleton& get();

    void executeStatement(std::string sql);
    void executeStatements(std::vector<std::string> sqls);
    KUKADU_SHARED_PTR<sql::ResultSet> executeQuery(std::string sql);

    void install();

    bool isInstalled();

    ~StorageSingleton();

};

}

#endif
