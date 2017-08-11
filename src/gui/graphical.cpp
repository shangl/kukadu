#include <kukadu/gui/graphical.hpp>
#include <kukadu/manipulation/skillfactory.hpp>
#include <json.hpp>
#include <QtWidgets/QLayout>
#include <QtWebKitWidgets/QWebFrame>
#include <QtWidgets/QListView>
#include <QtWidgets/QPushButton>
#include <QtWidgets/QMessageBox>
#include <QtWebKit/QtWebKit>
#include <kukadu/generated_skills/KinestheticTeaching.hpp>
#include <kukadu/robot.hpp>
#include <QtWidgets/QFileDialog>


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

    void KukaduGraphical::loadInformationFromDatabase() {
        kukadu::StorageSingleton &storage = kukadu::StorageSingleton::get();
        auto skillResult = storage.executeQuery(
                "SELECT skill_id as id, label as skillName, controller_types.controller_implementation_class as controller FROM skills INNER JOIN controller_types ON skills.controller_type=controller_types.controller_id");

        nlohmann::json skillJson;
        while (skillResult->next()) {
            int id = skillResult->getInt("id");
            nlohmann::json skillInformation;
            skillInformation["id"] = id;
            auto label = skillResult->getString("skillName");
            skillInformation["skillName"] = label;
            auto controller = skillResult->getString("controller");
            skillInformation["controller"] = controller;

            std::string query =
                    "SELECT DISTINCT robot_config.robot_config_id as configId FROM skills_robot INNER JOIN robot_config ON skills_robot.robot_config_id=robot_config.robot_config_id WHERE skills_robot.skill_id=" +
                    std::to_string(id);
            auto configForSkill = storage.executeQuery(query);
            std::vector<int> roboterConfigIds;
            while (configForSkill->next()) {
                roboterConfigIds.push_back(configForSkill->getInt("configId"));
            }

            skillInformation["configId"] = roboterConfigIds;

            skillJson.push_back(skillInformation);
        }

        nlohmann::json robotConfigJson;
        auto robotConfigResult = storage.executeQuery("SELECT DISTINCT robot_config_id FROM robot_config");
        while (robotConfigResult->next()) {

            int id = robotConfigResult->getInt("robot_config_id");
            auto configForId = storage.executeQuery(
                    "SELECT DISTINCT hardware_instance_id, order_id FROM robot_config WHERE robot_config_id=" +
                    std::to_string(id));
            nlohmann::json configInformation;
            std::vector<int> hardwareIds;
            std::vector<int> orderIds;
            while (configForId->next()) {
                int robotHwId = configForId->getInt("hardware_instance_id");
                int order = configForId->getInt("order_id");
                hardwareIds.push_back(robotHwId);
                orderIds.push_back(order);
            }

            configInformation["hardwareId"] = hardwareIds;
            configInformation["order"] = orderIds;
            configInformation["id"] = id;
            robotConfigJson.push_back(configInformation);
        }

        nlohmann::json hardwareJson;
        auto hardwareResult = storage.executeQuery(
                "SELECT hardware_instances.instance_id AS hardwareId, hardware_instances.instance_name as hardwareName ,IFNULL(kukie_hardware.deg_of_freedom, 0) AS degOfFreedom FROM hardware_instances LEFT OUTER JOIN kukie_hardware ON hardware_instances.instance_id=kukie_hardware.hardware_instance_id");
        while (hardwareResult->next()) {
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
        finalJson["roboConfigs"] = robotConfigJson;
        finalJson["attributePath"] = resolvePath("$KUKADU_HOME/meta/xml/");

        std::string js = finalJson.dump();
        QString info = QString("initializeDatabaseloader('%1')").arg(QString::fromStdString(js));
        webView->page()->mainFrame()->evaluateJavaScript(info);
    }

    QGroupBox *KukaduGraphical::createUI() {
        std::string blocklyPath = resolvePath("$KUKADU_HOME/external/blockly/cake/test.html");
        auto mainView = new QGroupBox();
        auto mainLayout = new QGridLayout();
        auto executeSkillContainer = new QHBoxLayout();
        auto kinestheticTeachingContainer = new QHBoxLayout();
        auto loadAndSaveContainer = new QHBoxLayout();

        QWebSettings::globalSettings()->setAttribute(QWebSettings::DeveloperExtrasEnabled, true);
        QWebSettings::globalSettings()->setAttribute(QWebSettings::LocalContentCanAccessRemoteUrls, true);
        QWebSettings::globalSettings()->setAttribute(QWebSettings::LocalContentCanAccessFileUrls, true);

        auto frame = new QGroupBox();
        frame->setLayout(mainLayout);

        webView = new QWebView(this);
        webView->setMinimumSize(DEFAULT_WIDTH, DEFAULT_HEIGHT - 150);
        webView->setMaximumSize(DEFAULT_WIDTH, DEFAULT_HEIGHT - 150);
        std::string indexFilePath = "file://" + blocklyPath;
        QObject::connect(webView, SIGNAL(loadFinished(bool)), this, SLOT(onStart()));
        webView->load(QUrl(indexFilePath.c_str()));

        auto executeButton = new QPushButton("Execute");
        QObject::connect(executeButton, SIGNAL(clicked()), this, SLOT(executeSlot()));
        auto stopExecutionButton = new QPushButton("Stop Execution");
        QObject::connect(stopExecutionButton, SIGNAL(clicked()), this, SLOT(stopExecutionSlot()));

        auto kinestheticButton = new QPushButton("Teach new Skill");
        QObject::connect(kinestheticButton, SIGNAL(clicked()), this, SLOT(kinestethicTeachingSlot()));

        auto saveButton = new QPushButton("Save");
        QObject::connect(saveButton, SIGNAL(clicked()), this, SLOT(saveSlot()));
        auto loadButton = new QPushButton("Load");
        QObject::connect(loadButton, SIGNAL(clicked()), this, SLOT(loadSlot()));


        mainView->setLayout(mainLayout);
        mainLayout->addLayout(executeSkillContainer, 1, 0);
        mainLayout->addLayout(kinestheticTeachingContainer, 2, 0);
        mainLayout->addLayout(loadAndSaveContainer, 3, 0);
        mainLayout->addWidget(webView, 0, 0);
        executeSkillContainer->addWidget(executeButton);
        executeSkillContainer->addWidget(stopExecutionButton);
        kinestheticTeachingContainer->addWidget(kinestheticButton);
        loadAndSaveContainer->addWidget(saveButton);
        loadAndSaveContainer->addWidget(loadButton);
        return mainView;
    }

    void KukaduGraphical::onStart() {
        loadInformationFromDatabase();
    }

    void KukaduGraphical::executeSlot() {
        createCatkinPackage();

        if (isSkillInstalled()) {
            createProjectInKukadu();
        } else {
            createProjectInPackage();
        }

        std::string packageName = getPackageName();
        std::string catkinWorkingDirectory = resolvePath("$KUKADU_HOME/../../");
        std::string argumentString = "cd " + catkinWorkingDirectory + ";";
        argumentString += getCatkinMakeString(packageName) + ";";
        argumentString += "cd devel/lib/" + packageName + ";";
        argumentString += "./" + packageName + " " + getExecutionMode() + " &";
        system(argumentString.c_str());
    }

    void KukaduGraphical::createCatkinPackage() {
        std::string catkinSources = resolvePath("$KUKADU_HOME/../");
        std::string packageName = getPackageName();
        std::string packagePath = catkinSources + packageName;

        QFile packageFolder(QString::fromStdString(packagePath));
        if (packageFolder.exists()) {
            std::string mainPath = packagePath + "/src/main.cpp";
            std::string headerPath = packagePath + "/include/" + getPackageName() + "/header.hpp";
            std::string skillPath = packagePath + "/src/skill.cpp";
            QFile mainFile(QString::fromStdString(mainPath));
            QFile headerFile(QString::fromStdString(headerPath));
            QFile skillFile(QString::fromStdString(skillPath));

            if (mainFile.exists())
                mainFile.remove();

            if (headerFile.exists())
                headerFile.remove();

            if (skillFile.exists())
                skillFile.remove();

            std::string argumentString = packagePath + "/CMakeLists.txt";
            QFile cmakeListsFile(argumentString.c_str());
            cmakeListsFile.open(QIODevice::ReadOnly);
            QTextStream streamIn(&cmakeListsFile);

            bool lineFound = false;
            std::string lineToFind = "add_executable(" + packageName + " src/main.cpp src/skill.cpp)";
            if(isSkillInstalled())
                lineToFind = "add_executable(" + packageName + " src/main.cpp)";

            while (!streamIn.atEnd() && !lineFound) {
                std::string line = streamIn.readLine().toUtf8().constData();
                lineFound = lineToFind==line;
            }
            cmakeListsFile.close();

            if(!lineFound){
                cmakeListsFile.remove();
                std::string origCMakeListsFilePath = packagePath + "/CMakeLists.txt.org";
                QFile origCMakeListsFile(origCMakeListsFilePath.c_str());
                argumentString = packagePath + "/CMakeLists.txt";
                origCMakeListsFile.rename(argumentString.c_str());
                createCMakeLists();
            }
        } else {
            std::string argumentString = "cd " + catkinSources + "; " +
                                         "catkin_create_pkg " + packageName + " geometry_msgs kukadu; " +
                                         "cd " + packageName + "; " +
                                         "mkdir src; mkdir include; cd include; mkdir " + packageName;
            system(argumentString.c_str());

            createCMakeLists();
        }
    }

    void KukaduGraphical::createCMakeLists() {
        std::string catkinSources = resolvePath("$KUKADU_HOME/../");
        std::string packageName = getPackageName();
        std::string packagePath = catkinSources + packageName;
        std::string argumentString = packagePath + "/CMakeLists.txt";
        QFile cmakeInFile(argumentString.c_str());
        argumentString = packagePath + "/CMakeLists.txt1";
        QFile cmakeOutFile(argumentString.c_str());
        cmakeInFile.open(QIODevice::ReadOnly);
        cmakeOutFile.open(QIODevice::WriteOnly);

        QTextStream streamIn(&cmakeInFile), streamOut(&cmakeOutFile);

        int arraysize = 124;
        QString textToAdd[arraysize];
        textToAdd[3] = QString("set(CMAKE_BUILD_TYPE Debug)");
        textToAdd[5] = QString(
                "include(CheckCXXCompilerFlag)\r\nCHECK_CXX_COMPILER_FLAG(\"-std=c++11\" COMPILER_SUPPORTS_CXX11)\r\nCHECK_CXX_COMPILER_FLAG(\"-std=c++0x\" COMPILER_SUPPORTS_CXX0X)\r\nif(COMPILER_SUPPORTS_CXX11)\r\nset(CMAKE_CXX_FLAGS \"${CMAKE_CXX_FLAGS} -std=c++11\")\r\nadd_definitions(-DCPP11SUPPORTED)\r\nelseif(COMPILER_SUPPORTS_CXX0X)\r\nset(CMAKE_CXX_FLAGS \"${CMAKE_CXX_FLAGS} -std=c++0x\")\r\nadd_definitions(-DCPP11SUPPORTED)\r\nelse()\r\nmessage(STATUS \"The compiler ${CMAKE_CXX_COMPILER} has no C++11 support. Please use a different C++ compiler.\")\r\nendif()");
        textToAdd[118] = "  include";

        if(isSkillInstalled()) {
            argumentString = "add_executable(" + packageName + " src/main.cpp)\r\n";
        } else {
            argumentString = "add_executable(" + packageName + " src/main.cpp src/skill.cpp)\r\n";
        }

        argumentString += "target_link_libraries(" + packageName + " ${catkin_LIBRARIES} kukadu kukaduvision)";

        textToAdd[123] = QString(argumentString.c_str());

        int i = 0;
        while (!streamIn.atEnd()) {
            if (i < arraysize) {
                auto data = textToAdd[i];
                if (data != NULL) {
                    streamOut << data << "\r\n";
                }
            }

            QString line = streamIn.readLine();
            if (!line.startsWith("#")) {
                streamOut << line << "\r\n";
            }

            i++;
        }
        cmakeInFile.close();
        cmakeOutFile.close();

        argumentString = catkinSources + packageName + "/CMakeLists.txt.org";
        cmakeInFile.rename(argumentString.c_str());
        argumentString = catkinSources + packageName + "/CMakeLists.txt";
        cmakeOutFile.rename(argumentString.c_str());
    }

    void KukaduGraphical::createProjectInPackage() {
        std::string packageName = getPackageName();
        std::string catkinSources = resolvePath("$KUKADU_HOME/../");
        std::string catkinWorkingDirectory = catkinSources + "../";

        writeToFileInPackage("src/main.cpp", getCodeBlocks()[0]);
        writeToFileInPackage("include/" + packageName + "/header.hpp", getCodeBlocks()[1]);
        writeToFileInPackage("src/skill.cpp", getCodeBlocks()[2]);
    }

    void KukaduGraphical::createProjectInKukadu() {
        std::string packageName = getPackageName();
        std::string catkinSources = resolvePath("$KUKADU_HOME/../");
        std::string catkinWorkingDirectory = catkinSources + "../";
        std::string sourceFolder = resolvePath("$KUKADU_HOME/src/generated_skills");
        std::string includeFolder = resolvePath("$KUKADU_HOME/include/kukadu/generated_skills");

        std::string skillName = getCurrentSkillName();
        writeToFileInPackage("src/main.cpp", getCodeBlocks()[0]);
        writeToFileAtPath(includeFolder + "/" + skillName + ".hpp", getCodeBlocks()[1]);
        writeToFileAtPath(sourceFolder + "/" + skillName + ".cpp", getCodeBlocks()[2]);

        std::string gskillHFilePath = catkinSources + "/kukadu/include/kukadu/generated_skills.hpp";
        QFile generatedSkillsHeaderFile(QString::fromStdString(gskillHFilePath));
        generatedSkillsHeaderFile.open(QIODevice::ReadOnly);
        QTextStream gskillHFStream(&generatedSkillsHeaderFile);
        QString textOfFile("");
        while (!gskillHFStream.atEnd()) {
            QString line = gskillHFStream.readLine();

            if (line.startsWith("#endif")) {
                string includeLine = "\t#include <kukadu/generated_skills/" + skillName + ".hpp>\n";
                textOfFile += QString::fromStdString(includeLine);
            }

            textOfFile += line + "\n";
        }

        generatedSkillsHeaderFile.close();
        generatedSkillsHeaderFile.open(QIODevice::WriteOnly | QIODevice::Text);
        gskillHFStream << textOfFile;
        generatedSkillsHeaderFile.close();

        SkillFactory::addSkill(skillName);

    }

    void KukaduGraphical::writeToFileInPackage(std::string filename, QString content) {

        std::string packageName = getPackageName();
        std::string catkinSources = resolvePath("$KUKADU_HOME/../");
        std::string argumentString = catkinSources + packageName + "/" + filename;
        writeToFileAtPath(argumentString, content);
    }

    void KukaduGraphical::writeToFileAtPath(std::string filepath, QString content) {

        QFile writeFile(QString::fromStdString(filepath));
        if (writeFile.exists()) {
            writeFile.remove();
        }

        writeFile.open(QIODevice::WriteOnly);
        QByteArray codeByteArray = content.toLatin1();
        writeFile.write(codeByteArray.data(), content.length());
        writeFile.close();
    }

    std::string KukaduGraphical::getCatkinMakeString(const std::string &packageName) {
        std::string cmakeCacheFilePath = resolvePath("$KUKADU_HOME/../../build/CMakeCache.txt");
        QFile cmakeCacheFile(cmakeCacheFilePath.c_str());
        if (!cmakeCacheFile.exists())
            return "catkin_make";

        cmakeCacheFile.open(QIODevice::ReadOnly);
        QTextStream inputStream(&cmakeCacheFile);

        std::string catkinMakeString = "";
        while (!inputStream.atEnd() && catkinMakeString.empty()) {
            QString line = inputStream.readLine();
            if (line.startsWith("CATKIN_WHITELIST_PACKAGES:STRING=")) {
                auto length = line.length() - 33;
                auto arguments = line.mid(33, length);
                QString startString((packageName + ";").c_str());
                QString containsString((";" + packageName + ";").c_str());
                QString endString((";" + packageName).c_str());
                if (arguments.isEmpty() || arguments.startsWith(startString) || arguments.contains(containsString) ||
                    arguments.endsWith(endString)) {
                    catkinMakeString = "catkin_make";
                } else {
                    catkinMakeString =
                            "catkin_make --only-pkg-with-deps " + std::string(arguments.toUtf8().constData()) + " " +
                            packageName;
                }
            }
        }

        cmakeCacheFile.close();

        std::replace(catkinMakeString.begin(), catkinMakeString.end(), ';', ' ');
        std::cout << catkinMakeString << std::endl;
        return catkinMakeString;
    }

    bool KukaduGraphical::isSkillInstalled() {
        auto isSkillInstalled = webView->page()->mainFrame()->evaluateJavaScript(
                "isSkillInstalled()").toString().toStdString();

        return isSkillInstalled == "true";
    }

    std::string KukaduGraphical::getExecutionMode() {
        std::string executionMode = webView->page()->mainFrame()->evaluateJavaScript(
                "getExecutionMode()").toString().toStdString();
        return executionMode;
    }

    std::string KukaduGraphical::getPackageName() {
        return "generated_graphical_programming";
    }

    std::vector<QString> KukaduGraphical::getCodeBlocks() {
        QVariant codeVariant = webView->page()->mainFrame()->evaluateJavaScript("getCode()");
        std::string packageName = getPackageName();

        QString mainCode = codeVariant.toString();
        QString textToReplace("ros::init(argc, args, \"kukadu\")");
        QString replacement("ros::init(argc, args, \"");
        replacement.append(QString::fromStdString(packageName));
        replacement.append("\")");
        mainCode.replace(textToReplace, replacement);

        auto headerStartPosition = mainCode.indexOf(QString("//Skillheader for Skill\n")) + 24;
        auto implementationStartPosition = mainCode.indexOf(QString("//Skillimplementation for Skill:\n")) + 33;
        QString headerCode = mainCode.mid(headerStartPosition, implementationStartPosition - headerStartPosition - 33);
        QString implementationCode = mainCode.mid(implementationStartPosition);
        mainCode = mainCode.mid(0, headerStartPosition - 24);

        return {mainCode, headerCode, implementationCode};
    }

    std::string KukaduGraphical::getCurrentSkillName() {
        return webView->page()->mainFrame()->evaluateJavaScript("getCurrentSkillName()").toString().toStdString();
    }

    void KukaduGraphical::kinestethicTeachingSlot() {
        kinestheticTeachingView = new QWidget;

        auto mainLayout = new QGridLayout();
        auto goToStartPosButton = new QPushButton("Guide to start Position");
        auto executeButton = new QPushButton("Let's go");
        auto finishedExecuteButton = new QPushButton("Teaching done");
        auto testButton = new QPushButton("Test this Skill");
        auto installButton = new QPushButton("Install this Skill with name:");
        auto exitButton = new QPushButton("Exit");
        kinestheticSkillName = new QLineEdit();

        QObject::connect(goToStartPosButton, SIGNAL(clicked()), this, SLOT(goToStartPositionSlot()));
        QObject::connect(executeButton, SIGNAL(clicked()), this, SLOT(startKinestheticTeachingSlot()));
        QObject::connect(finishedExecuteButton, SIGNAL(clicked()), this, SLOT(finishedExecutionSlot()));
        QObject::connect(testButton, SIGNAL(clicked()), this, SLOT(testTaughtSkillSlot()));
        QObject::connect(installButton, SIGNAL(clicked()), this, SLOT(installSkillSlot()));
        QObject::connect(exitButton, SIGNAL(clicked()), this, SLOT(exitViewSlot()));

        mainLayout->addWidget(goToStartPosButton, 0, 0);
        mainLayout->addWidget(executeButton, 1, 0);
        mainLayout->addWidget(finishedExecuteButton, 2, 0);
        mainLayout->addWidget(testButton, 3, 0);
        mainLayout->addWidget(installButton, 4, 0);
        mainLayout->addWidget(kinestheticSkillName, 4, 1);
        mainLayout->addWidget(exitButton, 5, 0);

        kinestheticTeachingView->setLayout(mainLayout);

        teachingObject = KUKADU_DYNAMIC_POINTER_CAST<skill::KinestheticTeaching>(SkillFactory::get().loadSkill("kinesthetic_teaching", {HardwareFactory::get().loadHardware("kukie_left_arm")}));

        kinestheticTeachingView->show();
    }

    void KukaduGraphical::goToStartPositionSlot() {
        teachingObject->bringToStartPos();
    }

    void KukaduGraphical::startKinestheticTeachingSlot() {
        teachingObject->showDmp();
    }

    void KukaduGraphical::finishedExecutionSlot() {
        teachingObject->endTeachingAndTrainDmp();
    }

    void KukaduGraphical::testTaughtSkillSlot() {
        teachingObject->testTrainedDmp();
    }

    void KukaduGraphical::installSkillSlot() {
        std::string skillName = kinestheticSkillName->text().toUtf8().constData();
        teachingObject->installDmp(skillName);
    }

    void KukaduGraphical::exitViewSlot() {
        kinestheticTeachingView->close();
        loadInformationFromDatabase();
        HardwareFactory::get().stopAllCreatedHardware();
    }

    void KukaduGraphical::loadSlot() {
        QString fileName = QFileDialog::getOpenFileName(this,
                                                        tr("Load Blocks"), "",
                                                        tr("Blockly File (*.xml);;All Files (*)"));

        QString xml = "";
        QFile f(fileName);
        f.open(QFile::ReadOnly | QFile::Text);
        QTextStream in(&f);

        xml = in.readAll();
        cout << xml.toStdString() << endl;
        QString info = QString("reloadBlocks('%1')").arg(xml);
        webView->page()->mainFrame()->evaluateJavaScript(info);
    }

    void KukaduGraphical::saveSlot() {
        QString fileName = QFileDialog::getSaveFileName(this,
                                                        tr("Save Blocks"), "new BlocklyFile.xml",
                                                        tr("Blockly File (*.xml);;All Files (*)"));
        auto xml = webView->page()->mainFrame()->evaluateJavaScript("downloadBlocks()");

        cout << xml.toString().toStdString() << endl;

        writeToFileAtPath(fileName.toStdString(), xml.toString());
    }

    void KukaduGraphical::stopExecutionSlot() {
        //todo
        cout << "implement this" << endl;
    }
}
