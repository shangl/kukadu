#include <kukadu/storage/storagesingleton.hpp>
#include <boost/program_options.hpp>
#include <kukadu/utils/utils.hpp>
#include <algorithm>

using namespace std;

namespace kukadu {

    StorageSingleton::StorageSingleton() {

        boost::program_options::options_description desc("kukadu properties");
        desc.add_options()
                ("kukadu.databaseurl", boost::program_options::value<string>()->required(), "")
                ("kukadu.database", boost::program_options::value<string>()->required(), "")
                ("kukadu.user", boost::program_options::value<string>()->required(), "")
                ("kukadu.pw", boost::program_options::value<string>()->required(), "");
        ifstream parseFile(resolvePath("$KUKADU_HOME/cfg/kukadu.prop").c_str(), std::ifstream::in);
        boost::program_options::variables_map vm;
        boost::program_options::store(boost::program_options::parse_config_file(parseFile, desc, true), vm);
        boost::program_options::notify(vm);

        driver = get_driver_instance();
        con = driver->connect(vm["kukadu.databaseurl"].as<std::string>(), vm["kukadu.user"].as<std::string>(), vm["kukadu.pw"].as<std::string>());

        databaseName = vm["kukadu.database"].as<std::string>();

        if(!isInstalled())
            install();

        executeStatement("use " + databaseName);

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

    void StorageSingleton::executeStatement(std::string sql) {

        auto stmt = con->createStatement();
        stmt->execute(sql);
        delete stmt;

    }

    void StorageSingleton::executeStatements(std::vector<std::string> sqls) {

        auto stmt = con->createStatement();
        for(auto& sql : sqls)
            stmt->execute(sql);
        delete stmt;

    }

    KUKADU_SHARED_PTR<sql::ResultSet> StorageSingleton::executeQuery(std::string sql) {

        auto stmt = con->createStatement();
        auto retSet = KUKADU_SHARED_PTR<sql::ResultSet>(stmt->executeQuery(sql));
        // is it allowed to delete the statement before deleteing res? (we will find out :) )
        delete stmt;
        return retSet;

    }

    StorageSingleton::~StorageSingleton() {
        delete con;
    }

}
