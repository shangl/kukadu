#include <sstream>
#include <json.hpp>
#include <QtWidgets/QLabel>
#include <kukadu/robot.hpp>
#include <QtWidgets/QLayout>
#include <QtWebKit/QtWebKit>
#include <QtWidgets/QListView>
#include <QtWidgets/QComboBox>
#include <QtWidgets/QPushButton>
#include <QtWidgets/QMessageBox>
#include <QtWidgets/QFileDialog>
#include <kukadu/gui/graphical.hpp>
#include <QtWebKitWidgets/QWebFrame>
#include <kukadu/manipulation/skillfactory.hpp>
#include <kukadu/generated_skills/KinestheticTeaching.hpp>

using namespace std;

namespace kukadu {

    KukaduGraphical::KukaduGraphical() {
        mainTab = new QTabWidget(this);
        mainTab->setMinimumSize(DEFAULT_WIDTH, DEFAULT_HEIGHT);
        mainTab->setMaximumSize(DEFAULT_WIDTH, DEFAULT_HEIGHT);

        auto tab = createUI();
        mainTab->addTab(tab, "Blockly");
        selectedTeachingHardware = "kukie_left_arm";
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
                "SELECT hardware_instances.instance_id AS hardwareId, hardware_instances.instance_name as hardwareName ,IFNULL(kukie_hardware.deg_of_freedom, 0) AS degOfFreedom, hw.hardware_class as classId FROM hardware_instances LEFT OUTER JOIN kukie_hardware ON hardware_instances.instance_id=kukie_hardware.hardware_instance_id INNER JOIN hardware hw ON hw.hardware_id=hardware_instances.hardware_id");
        //"SELECT hardware_instances.instance_id AS hardwareId, hardware_instances.instance_name as hardwareName ,IFNULL(kukie_hardware.deg_of_freedom, 0) AS degOfFreedom FROM hardware_instances LEFT OUTER JOIN kukie_hardware ON hardware_instances.instance_id=kukie_hardware.hardware_instance_id");
        while (hardwareResult->next()) {
            nlohmann::json hardwareInformation;
            int id = hardwareResult->getInt("hardwareId");
            hardwareInformation["hardwareId"] = id;
            auto name = hardwareResult->getString("hardwareName");
            hardwareInformation["hardwareName"] = name;
            int defOfFreedom = hardwareResult->getInt("degOfFreedom");
            hardwareInformation["degOfFreedom"] = defOfFreedom;
            int classId = hardwareResult->getInt("classId");
            hardwareInformation["classId"] = classId;

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
        auto playingContainer = new QHBoxLayout();
        auto deleteContainer = new QHBoxLayout();
        deleteSkillName = new QLineEdit();

        hardwareSelector = new QComboBox();
        hardwareSelector->addItem("kukie_left_arm");
        hardwareSelector->addItem("kukie_right_arm");
        QObject::connect(hardwareSelector, SIGNAL(currentIndexChanged(QString)), this,
                         SLOT(selectionChangedSlot(QString)));


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

        auto playButton = new QPushButton("Play");
        QObject::connect(playButton, SIGNAL(clicked()), this, SLOT(playSlot()));

        auto deleteButton = new QPushButton("DeleteSkill");
        QObject::connect(deleteButton, SIGNAL(clicked()), this, SLOT(deleteSlot()));

        mainView->setLayout(mainLayout);
        mainLayout->addLayout(executeSkillContainer, 1, 0);
        mainLayout->addLayout(kinestheticTeachingContainer, 2, 0);
        mainLayout->addLayout(loadAndSaveContainer, 3, 0);
        mainLayout->addLayout(playingContainer, 4, 0);
        mainLayout->addLayout(deleteContainer, 5, 0);
        mainLayout->addWidget(webView, 0, 0);
        executeSkillContainer->addWidget(executeButton);
        executeSkillContainer->addWidget(stopExecutionButton);
        kinestheticTeachingContainer->addWidget(hardwareSelector);
        kinestheticTeachingContainer->addWidget(kinestheticButton);
        loadAndSaveContainer->addWidget(saveButton);
        loadAndSaveContainer->addWidget(loadButton);
        playingContainer->addWidget(playButton);
        deleteContainer->addWidget(deleteSkillName);
        deleteContainer->addWidget(deleteButton);

        return mainView;

    }

    void KukaduGraphical::loadMetafilesString() {
        auto files = getFilesInDirectory(resolvePath("$KUKADU_HOME/meta/xml"));
        stringstream filesStream;
        for (auto &f : files)
            if (f != "." && f != "..")
                filesStream << f << ";";
        QString callString = QString("set_available_files('%1')").arg(QString::fromStdString(filesStream.str()));
        webView->page()->mainFrame()->evaluateJavaScript(callString);
    }

    void KukaduGraphical::setPrevSkillIncludes() {

        string includeString = "";
        auto allFiles = getFilesInDirectory(resolvePath("$KUKADU_HOME/include/kukadu/generated_skills/"));

        bool firstTime = true;
        for (string &includeFile : allFiles) {
            if (includeFile.find(".hpp") != std::string::npos) {
                if (firstTime)
                    firstTime = false;
                else
                    includeString += ",";
                includeString += includeFile;
            }
        }

        QString callString = QString("setPreviousIncludes('%1')").arg(QString::fromStdString(includeString));
        webView->page()->mainFrame()->evaluateJavaScript(callString);

    }

    void KukaduGraphical::onStart() {
        loadMetafilesString();
        setPrevSkillIncludes();
        loadInformationFromDatabase();
    }

    void KukaduGraphical::executeSlot() {
        createCatkinPackage();

        bool execute = true;
        if (isSkillInstalled()) {
            execute = createProjectInKukadu();
        } else {
            createProjectInPackage();
        }

        if (execute) {
            std::string packageName = getPackageName();
            std::string catkinWorkingDirectory = resolvePath("$KUKADU_HOME/../../");
            std::string argumentString = "cd " + catkinWorkingDirectory + ";";
            argumentString += getCatkinMakeString(packageName);
            system(argumentString.c_str());

            argumentString = "cd " + catkinWorkingDirectory + "devel/lib/" + packageName + ";";
            argumentString += " ./" + packageName + " " + getExecutionMode() + " &";
            system(argumentString.c_str());
        }

        setPrevSkillIncludes();

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
            if (isSkillInstalled())
                lineToFind = "add_executable(" + packageName + " src/main.cpp)";

            while (!streamIn.atEnd() && !lineFound) {
                std::string line = streamIn.readLine().toUtf8().constData();
                lineFound = lineToFind == line;
            }
            cmakeListsFile.close();

            if (!lineFound) {
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

        if (isSkillInstalled()) {
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

    bool KukaduGraphical::createProjectInKukadu() {
        std::string skillName = getCurrentSkillName();
        if (checkSkillNameIsNew(skillName)) {
            std::string packageName = getPackageName();
            std::string catkinSources = resolvePath("$KUKADU_HOME/../");
            std::string catkinWorkingDirectory = catkinSources + "../";
            std::string sourceFolder = resolvePath("$KUKADU_HOME/src/generated_skills");
            std::string includeFolder = resolvePath("$KUKADU_HOME/include/kukadu/generated_skills");

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

            loadMetafilesString();
            return true;
        } else {
            return false;
        }
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

        int i = 0;
        mainLayout->addWidget(goToStartPosButton, i++, 0);
        mainLayout->addWidget(executeButton, i++, 0);
        mainLayout->addWidget(finishedExecuteButton, i++, 0);
        mainLayout->addWidget(testButton, i++, 0);
        mainLayout->addWidget(installButton, i, 0);
        mainLayout->addWidget(kinestheticSkillName, i++, 1);
        mainLayout->addWidget(exitButton, i++, 0);

        kinestheticTeachingView->setLayout(mainLayout);

        teachingObject = KUKADU_DYNAMIC_POINTER_CAST<KinestheticTeaching>(
                SkillFactory::get().loadSkill("kinesthetic_teaching",
                                              {HardwareFactory::get().loadHardware(selectedTeachingHardware)}));

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
        loadMetafilesString();
    }

    void KukaduGraphical::exitViewSlot() {
        kinestheticTeachingView->close();
        loadInformationFromDatabase();
        HardwareFactory::get().stopAllCreatedHardware();
    }

    void KukaduGraphical::loadSlot() {

        loadMetafilesString();

        QString fileName = QFileDialog::getOpenFileName(this,
                                                        tr("Load Blocks"), "",
                                                        tr("Blockly File (*.xml);;All Files (*)"));

        if (fileName != "") {

            QString xml = "";
            QFile f(fileName);
            f.open(QFile::ReadOnly | QFile::Text);
            QTextStream in(&f);

            xml = in.readAll();
            QString info = QString("reloadBlocks('%1')").arg(xml);
            webView->page()->mainFrame()->evaluateJavaScript(info);

        }

    }

    void KukaduGraphical::saveSlot() {
        QString fileName = QFileDialog::getSaveFileName(this,
                                                        tr("Save Blocks"), "skill.xml",
                                                        tr("Blockly File (*.xml);;All Files (*)"));
        auto xml = webView->page()->mainFrame()->evaluateJavaScript("downloadBlocks()");

        writeToFileAtPath(fileName.toStdString(), xml.toString());
    }

    void KukaduGraphical::stopExecutionSlot() {
        string argumentString = "pkill generated_graph";
        system(argumentString.c_str());
    }

    void KukaduGraphical::playSlot() {

        playingView = new QWidget;

        auto mainLayout = new QGridLayout();

        playableSkillsBox = new QComboBox();
        auto playableSkills = SkillFactory::get().loadPlayableSkills();
        for (auto &skill : playableSkills)
            playableSkillsBox->addItem(QString::fromStdString(skill));

        auto trainPerceptualStatesButton = new QPushButton("Train perceptual states");
        auto playButton = new QPushButton("Play");
        auto exitButton = new QPushButton("Exit");

        QLabel *noteLabel = new QLabel("Note: separate the behaviour names with a comma");
        QLabel *playWhichLabel = new QLabel("For which skill do you want to extend the DoA?");
        QLabel *usedSensingLabel = new QLabel("List of sensing behaviours:");
        QLabel *usedBehavioursLabel = new QLabel("List of playing behaviours:");

        usedSensingBehaviours = new QLineEdit("Slide,Poke");
        usedPlayingBehaviours = new QLineEdit("PushTranslation");

        QObject::connect(trainPerceptualStatesButton, SIGNAL(clicked()), this, SLOT(trainPerceptualStatesSlot()));
        QObject::connect(playButton, SIGNAL(clicked()), this, SLOT(performPlayingSlot()));
        QObject::connect(exitButton, SIGNAL(clicked()), this, SLOT(exitPlayingslot()));

        int i = 0;
        mainLayout->addWidget(playWhichLabel, i, 0);
        mainLayout->addWidget(playableSkillsBox, i++, 1);

        mainLayout->addWidget(noteLabel, i++, 0);
        mainLayout->addWidget(usedSensingLabel, i, 0);
        mainLayout->addWidget(usedSensingBehaviours, i, 1);
        mainLayout->addWidget(trainPerceptualStatesButton, i++, 2);

        mainLayout->addWidget(usedBehavioursLabel, i, 0);
        mainLayout->addWidget(usedPlayingBehaviours, i, 1);
        mainLayout->addWidget(playButton, i++, 2);

        mainLayout->addWidget(exitButton, i++, 0);

        playingView->setLayout(mainLayout);
        playingView->show();

    }

    std::vector<KUKADU_SHARED_PTR<kukadu::Hardware> > extractAndGenerateHardware(std::string hardwareList) {

        std::vector<KUKADU_SHARED_PTR<kukadu::Hardware> > retHardware;

        auto& hwFactory = HardwareFactory::get();
        hwFactory.setSimulation(false);
        KukaduTokenizer tok(hardwareList, ",");
        string currentHardware = "";
        while ((currentHardware = tok.next()) != "") {
            auto currentHwInstance = hwFactory.loadHardware(currentHardware);
            if (currentHwInstance) {
                currentHwInstance->install();
                currentHwInstance->start();
                retHardware.push_back(currentHwInstance);
            }
        }

        return retHardware;

    }

    template<typename T>
    std::vector<KUKADU_SHARED_PTR<T> >
    extractAndGenerateControllers(std::string controllerList, std::string hardwareToUse) {

        vector<KUKADU_SHARED_PTR<T> > retVec;

        KukaduTokenizer tok(controllerList, ",");
        string currentBehaviour = "";

        auto hardwareInstances = extractAndGenerateHardware(hardwareToUse);

        SkillFactory &factory = SkillFactory::get();
        while ((currentBehaviour = tok.next()) != "") {
            auto currentBehaviourController = KUKADU_DYNAMIC_POINTER_CAST<T>(
                    factory.loadSkill(currentBehaviour, hardwareInstances));
            retVec.push_back(currentBehaviourController);
        }

        return retVec;

    }

    void KukaduGraphical::trainPerceptualStatesSlot() {

        auto sensingControllerNames = usedSensingBehaviours->text().toStdString();
        auto behaviourControllerNames = usedPlayingBehaviours->text().toStdString();
        playingControllerName = playableSkillsBox->currentText().toStdString();

        string allUsedHardware = webView->page()->mainFrame()->evaluateJavaScript("getRequiredHardware()").toString().toStdString();;

        auto sensingControllers = extractAndGenerateControllers<kukadu::SensingController>(sensingControllerNames, allUsedHardware);
        auto playingControllers = extractAndGenerateControllers<kukadu::Controller>(behaviourControllerNames, allUsedHardware);
        auto toTrainController = extractAndGenerateControllers<kukadu::Controller>({playingControllerName}, allUsedHardware);
        auto nothingSkill = SkillFactory::get().loadSkill("nothing", {});

        currentHapticPlanner = make_shared<HapticPlanner>(resolvePath("$KUKADU_HOME/skills/"), sensingControllers,
                                                          playingControllers,
                                                          toTrainController,
                                                          nothingSkill,
                                                          SkillFactory::get().getGenerator());

    }

    void KukaduGraphical::performPlayingSlot() {

        playingEnded = false;
        keepPlaying = true;
        while (keepPlaying) {
            currentHapticPlanner->performComplexSkill(playingControllerName);
            currentHapticPlanner->updateModels();
        }
        playingEnded = true;

    }

    void KukaduGraphical::deleteSlot() {
        std::string skillName = deleteSkillName->text().toStdString();
        if (!checkSkillNameIsNew(skillName)) {
            deleteIncludesOfSkill(skillName);
            deleteSkillHeaderFile(skillName);
            deleteSkillCppFile(skillName);
            deleteFromSkillFactory(skillName);
        }
    }

    void KukaduGraphical::exitPlayingslot() {

        keepPlaying = false;
        cout << "the playing will be stopped after the current rollout" << endl;

        ros::Rate r(3);
        while (!playingEnded)
            r.sleep();

        playingView->close();

    }

    void KukaduGraphical::selectionChangedSlot(QString text) {
        selectedTeachingHardware = text.toStdString();
    }

    bool KukaduGraphical::checkSkillNameIsNew(std::string skillName) {
        std::string checkline = "#include <kukadu/generated_skills/" + skillName + ".hpp>";
        std::string gskillHFilePath = resolvePath("$KUKADU_HOME/../") + "/kukadu/include/kukadu/generated_skills.hpp";

        QFile mainFile(QString::fromStdString(gskillHFilePath));
        mainFile.open(QIODevice::ReadOnly);
        QTextStream streamIn(&mainFile);

        while (!streamIn.atEnd()) {
            auto line = streamIn.readLine();
            if (line.contains(QString::fromStdString(checkline)))
                return false;
        }

        return true;
    }


    void KukaduGraphical::deleteIncludesOfSkill(std::string skillName) {
        std::string checkline = "#include <kukadu/generated_skills/" + skillName + ".hpp>";

        auto files = getFilesInDirectory(resolvePath("$KUKADU_HOME/src/generated_skills"));
        for (auto &f : files)
            if (f != "." && f != "..")
                deleteLineFromFiles(checkline, resolvePath("$KUKADU_HOME/src/generated_skills")+ "/" + f);

        std::string gskillHFilePath = resolvePath("$KUKADU_HOME/../") + "/kukadu/include/kukadu/generated_skills.hpp";
        deleteLineFromFiles(checkline, gskillHFilePath);
    }

    void KukaduGraphical::deleteLineFromFiles(std::string checkline, std::string filepath) {
        std::string gskillHFilePath = filepath;
        std::string gskillHFilePathTmp = filepath + "2";

        QFile mainFile(QString::fromStdString(gskillHFilePath));
        mainFile.open(QIODevice::ReadOnly);
        QFile mainFileTmp(QString::fromStdString(gskillHFilePathTmp));
        mainFileTmp.open(QIODevice::WriteOnly);
        QTextStream streamIn(&mainFile);
        QTextStream streamOut(&mainFileTmp);

        while (!streamIn.atEnd()) {
            auto line = streamIn.readLine();
            if (!line.contains(QString::fromStdString(checkline)))
                streamOut << line << "\n";
        }

        mainFile.close();
        mainFileTmp.close();

        mainFile.remove();
        mainFileTmp.rename(gskillHFilePath.c_str());
    }

    void KukaduGraphical::deleteSkillHeaderFile(std::string skillName) {
        std::string filePath = resolvePath("$KUKADU_HOME/include/kukadu/generated_skills") + "/" + skillName + ".hpp";
        QFile headerFile(QString::fromStdString(filePath));
        headerFile.remove();
    }

    void KukaduGraphical::deleteSkillCppFile(std::string skillName) {
        std::string filePath = resolvePath("$KUKADU_HOME/src/generated_skills") + "/" + skillName + ".cpp";
        QFile sourceFile(QString::fromStdString(filePath));
        sourceFile.remove();
    }

    void KukaduGraphical::deleteFromSkillFactory(std::string skillName) {
        SkillFactory::removeSkill(skillName);
    }

}
