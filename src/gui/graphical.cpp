#include <kukadu/gui/graphical.hpp>

// very nasty tweak
//#include <QtWidgets/../QtWebKitWidgets/QWebView>
//#include <QtWidgets/../QtWebKitWidgets/QWebPage>
//#include <QtWidgets/../QtWebKitWidgets/QWebFrame>


using namespace std;

namespace kukadu {

    KukaduGraphical::KukaduGraphical() {
        mainTab = new QTabWidget(this);
        mainTab->setMinimumSize(DEFAULT_WIDTH, DEFAULT_HEIGHT);
        mainTab->setMaximumSize(DEFAULT_WIDTH, DEFAULT_HEIGHT);

        auto tab = createUI();
        mainTab->addTab(tab, "Blockly");
    }

    KukaduGraphical::~KukaduGraphical() {

    }

    void KukaduGraphical::loadInformationFromDatabase(){
        kukadu::StorageSingleton& storage = kukadu::StorageSingleton::get();
        auto skillResult= storage.executeQuery("SELECT skill_id as id, label as skillName, controller_types.controller_implementation_class as controller FROM skills INNER JOIN controller_types ON skills.controller_type=controller_types.controller_id");

        nlohmann::json skillJson;

        while(skillResult->next()){
            int id = skillResult->getInt("id");
            nlohmann::json skillInformation;
            skillInformation["id"] = id;
            auto label = skillResult->getString("skillName");
            skillInformation["skillName"] = label;
            auto controller = skillResult->getString("controller");
            skillInformation["controller"] = controller;

            std::string query = "SELECT hardware_Instance_id as hardwareId FROM skills_robot WHERE skill_id=" + std::to_string(id);
            auto hardwareForSkill = storage.executeQuery(query);
            std::vector<int> requiredHardwareIds;
            while(hardwareForSkill->next()){
                requiredHardwareIds.push_back(hardwareForSkill->getInt("hardwareId"));
            }

            skillInformation["hardwareId"] = requiredHardwareIds;

            skillJson.push_back(skillInformation);
        }

        nlohmann::json hardwareJson;
        auto hardwareResult = storage.executeQuery("SELECT hardware_instances.instance_id AS hardwareId, hardware_instances.instance_name as hardwareName ,IFNULL(kukie_hardware.deg_of_freedom, 0) AS degOfFreedom FROM hardware_instances LEFT OUTER JOIN kukie_hardware ON hardware_instances.instance_id=kukie_hardware.hardware_instance_id");
        while(hardwareResult->next()){
            nlohmann::json hardwareInformation;
            int id = hardwareResult->getInt("hardwareId");
            hardwareInformation["hardwareId"] = id;
            auto name = hardwareResult->getString("hardwareName");
            hardwareInformation["hardwareName"] = name;
            int defOfFreedom = hardwareResult->getInt("degOfFreedom");
            hardwareInformation["degOfFreedom"] = defOfFreedom;

            hardwareJson.push_back(hardwareInformation);
        }

        nlohmann::json finalJson;
        finalJson["hardwareInformation"] = hardwareJson;
        finalJson["skillInformation"] = skillJson;
        finalJson["attributePath"] = resolvePath("$KUKADU_HOME/meta/xml/");

        std::string js = finalJson.dump();
        QString info = QString("initializeDatabaseloader('%1')").arg(QString::fromStdString(js));
        webView->page()->mainFrame()->evaluateJavaScript(info);
    }

    QGroupBox* KukaduGraphical::createUI() {
        std::string blocklyPath = resolvePath("$KUKADU_HOME/external/blockly/cake/test.html");
        auto mainView = new QGroupBox();
        auto mainLayout = new QGridLayout();
        auto buttonContainer = new QHBoxLayout();
        packeNameLineEdit = new QLineEdit();

        QWebSettings::globalSettings()->setAttribute(QWebSettings::DeveloperExtrasEnabled, true);
        QWebSettings::globalSettings()->setAttribute(QWebSettings::LocalContentCanAccessRemoteUrls,true);
        QWebSettings::globalSettings()->setAttribute(QWebSettings::LocalContentCanAccessFileUrls,true);

        auto frame = new QGroupBox();
        frame->setLayout(mainLayout);

        webView = new QWebView(this);
        webView->setMinimumSize(DEFAULT_WIDTH, DEFAULT_HEIGHT-150);
        webView->setMaximumSize(DEFAULT_WIDTH, DEFAULT_HEIGHT-150);
        std::string indexFilePath = "file://" + blocklyPath;
        QObject::connect(webView, SIGNAL(loadFinished(bool)), this, SLOT(onStart()));
        webView->load(QUrl(indexFilePath.c_str()));

        auto executeButton = new QPushButton("Execute");
        QObject::connect(executeButton, SIGNAL(clicked()),this, SLOT(clickedSlot()));


        mainView->setLayout(mainLayout);
        mainLayout->addLayout(buttonContainer, 1, 0);
        mainLayout->addWidget(webView, 0, 0);
        buttonContainer->addWidget(executeButton);
        buttonContainer->addWidget(packeNameLineEdit);

        return mainView;
    }

    void KukaduGraphical::onStart() {
        loadInformationFromDatabase();
        std::cout << "started" << std::endl;
    }

    void KukaduGraphical::clickedSlot() {
        std::string var = "test";
        getCatkinMakeString(var);

        QVariant codeVariant = webView->page()->mainFrame()->evaluateJavaScript("getCode()");

        std::string packageName = packeNameLineEdit->text().toUtf8().constData();
        if(packageName.empty()){
            packageName = "graphical_test";
        }

        std::string catkinSources = resolvePath("$KUKADU_HOME/../");
        std::string catkinWorkingDirectory = catkinSources + "../";

        std::string argumentString= "cd " + catkinSources + ";rm -r " + packageName + ";catkin_create_pkg " + packageName + " geometry_msgs kukadu;cd " + packageName + ";mkdir src;";
        system(argumentString.c_str());

        argumentString = catkinSources + packageName + "/src/code.cpp";
        QFile codeFile(argumentString.c_str());
        if(codeFile.exists()){
            codeFile.remove();
        }

        codeFile.open(QIODevice::WriteOnly);
        QString code = codeVariant.toString();
        QByteArray codeByteArray = code.toLatin1();
        codeFile.write(codeByteArray.data(), code.length());
        codeFile.close();

        argumentString = catkinSources + packageName + "/CMakeLists.txt";
        QFile cmakeInFile(argumentString.c_str());
        argumentString = catkinSources + packageName + "/CMakeLists.txt1";
        QFile cmakeOutFile(argumentString.c_str());
        cmakeInFile.open(QIODevice::ReadOnly);
        cmakeOutFile.open(QIODevice::WriteOnly);

        QTextStream streamIn(&cmakeInFile), streamOut(&cmakeOutFile);

        int arraysize = 124;
        QString textToAdd[arraysize];
        textToAdd[3] = QString("set(CMAKE_BUILD_TYPE Debug)");
        textToAdd[5] = QString("include(CheckCXXCompilerFlag)\r\nCHECK_CXX_COMPILER_FLAG(\"-std=c++11\" COMPILER_SUPPORTS_CXX11)\r\nCHECK_CXX_COMPILER_FLAG(\"-std=c++0x\" COMPILER_SUPPORTS_CXX0X)\r\nif(COMPILER_SUPPORTS_CXX11)\r\nset(CMAKE_CXX_FLAGS \"${CMAKE_CXX_FLAGS} -std=c++11\")\r\nadd_definitions(-DCPP11SUPPORTED)\r\nelseif(COMPILER_SUPPORTS_CXX0X)\r\nset(CMAKE_CXX_FLAGS \"${CMAKE_CXX_FLAGS} -std=c++0x\")\r\nadd_definitions(-DCPP11SUPPORTED)\r\nelse()\r\nmessage(STATUS \"The compiler ${CMAKE_CXX_COMPILER} has no C++11 support. Please use a different C++ compiler.\")\r\nendif()");
        argumentString = "add_executable(" + packageName + " src/code.cpp)\r\ntarget_link_libraries(" + packageName + " ${catkin_LIBRARIES} kukadu kukaduvision)";
        textToAdd[123] = QString(argumentString.c_str());
        int i = 0;
        while (!streamIn.atEnd()){
            if(i < arraysize){
                auto data = textToAdd[i];
                if(data != NULL){
                    streamOut << data << "\r\n";
                }
            }

            QString line = streamIn.readLine();
            if(!line.startsWith("#")){
                streamOut << line << "\r\n";
            }

            i++;
        }
        cmakeInFile.close();
        cmakeOutFile.close();

        cmakeInFile.remove();
        argumentString = catkinSources + packageName + "/CMakeLists.txt";
        cmakeOutFile.rename(argumentString.c_str());


        argumentString = "cd " + catkinWorkingDirectory + ";";
        argumentString += getCatkinMakeString(packageName) + ";";
        argumentString += "cd devel/lib/" + packageName + ";";
        argumentString += "./" + packageName;
        system(argumentString.c_str());
    }

    std::string KukaduGraphical::getCatkinMakeString(const std::string& packageName){
        std::string cmakeCacheFilePath = resolvePath("$KUKADU_HOME/../../build/CMakeCache.txt");
        QFile cmakeCacheFile(cmakeCacheFilePath.c_str());
        cmakeCacheFile.open(QIODevice::ReadOnly);
        QTextStream inputStream(&cmakeCacheFile);


        std::string catkinMakeString = "";
        while(!inputStream.atEnd() && catkinMakeString.empty()){
            QString line = inputStream.readLine();
            if(line.startsWith("CATKIN_WHITELIST_PACKAGES:STRING=")){
                auto length = line.length()-33;
                auto arguments = line.mid(33, length);
                QString startString((packageName + ";").c_str());
                QString containsString((";" + packageName + ";").c_str());
                QString endString((";" + packageName).c_str());
                if(arguments.isEmpty() || arguments.startsWith(startString) || arguments.contains(containsString) || arguments.endsWith(endString)){
                    catkinMakeString = "catkin_make";
                } else {
                    catkinMakeString = "catkin_make --only-pkg-with-deps " + std::string(arguments.toUtf8().constData()) + ";" + packageName;
                }
            }
        }

        cmakeCacheFile.close();

        std::replace(catkinMakeString.begin(), catkinMakeString.end(), ';', ' ');
        std::cout << catkinMakeString << std::endl;
        return catkinMakeString;
    }
}
