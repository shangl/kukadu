#include <kukadu/gui/graphical.hpp>

#include <sstream>
#include <iostream>
#include <vector>
#include <QtWidgets/QLabel>
#include <QtWidgets/QListView>
#include <QtWidgets/QLineEdit>
#include <QtWidgets/QTableView>
#include <QtWidgets/QPushButton>
#include <QtWidgets/QGridLayout>
#include <QtWidgets/QHBoxLayout>
#include <QtWidgets/QMessageBox>
#include <QtWebKit/QtWebKit>

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

    QGroupBox* KukaduGraphical::createUI() {
        auto mainView = new QGroupBox();
        auto mainLayout = new QGridLayout();
        auto buttonContainer = new QHBoxLayout();

        QWebSettings::globalSettings()->setAttribute(QWebSettings::DeveloperExtrasEnabled, true);
        QWebSettings::globalSettings()->setAttribute(QWebSettings::LocalContentCanAccessRemoteUrls,true);
        QWebSettings::globalSettings()->setAttribute(QWebSettings::LocalContentCanAccessFileUrls,true);

        auto frame = new QGroupBox();
        frame->setLayout(mainLayout);

        webView = new QWebView(this);
        webView->setMinimumSize(DEFAULT_WIDTH, DEFAULT_HEIGHT-150);
        webView->setMaximumSize(DEFAULT_WIDTH, DEFAULT_HEIGHT-150);
        webView->load(QUrl("file:///home/agyss/blockly\ test/cake/test.html"));

        auto executeButton = new QPushButton("Execute");
        QObject::connect(executeButton, SIGNAL(clicked()),this, SLOT(clickedSlot()));


        mainView->setLayout(mainLayout);
        mainLayout->addLayout(buttonContainer, 1, 0);
        mainLayout->addWidget(webView, 0, 0);
        buttonContainer->addWidget(executeButton);

        return mainView;
    }

    void KukaduGraphical::clickedSlot() {
        QVariant codeVariant = webView->page()->mainFrame()->evaluateJavaScript("getCode()");
        qDebug() << codeVariant.toString();
        system("cd ~/iis_robot_sw/iis_catkin_ws/src/;rm -r test;catkin_create_pkg test geometry_msgs kukadu;cd test;mkdir src;");


        QFile codeFile("/home/agyss/iis_robot_sw/iis_catkin_ws/src/test/src/code.cpp");
        if(codeFile.exists()){
            codeFile.remove();
        }

        codeFile.open(QIODevice::WriteOnly);
        QString code = codeVariant.toString();
        QByteArray codeByteArray = code.toLatin1();
        codeFile.write(codeByteArray.data(), code.length());
        codeFile.close();


        QFile cmakeInFile("/home/agyss/iis_robot_sw/iis_catkin_ws/src/test/CMakeLists.txt");
        QFile cmakeOutFile("/home/agyss/iis_robot_sw/iis_catkin_ws/src/test/CMakeLists.txt1");
        cmakeInFile.open(QIODevice::ReadOnly);
        cmakeOutFile.open(QIODevice::WriteOnly);

        QTextStream streamIn(&cmakeInFile), streamOut(&cmakeOutFile);

        int arraysize = 124;
        QString textToAdd[arraysize];
        textToAdd[3] = QString("set(CMAKE_BUILD_TYPE Debug)");
        textToAdd[5] = QString("include(CheckCXXCompilerFlag)\r\nCHECK_CXX_COMPILER_FLAG(\"-std=c++11\" COMPILER_SUPPORTS_CXX11)\r\nCHECK_CXX_COMPILER_FLAG(\"-std=c++0x\" COMPILER_SUPPORTS_CXX0X)\r\nif(COMPILER_SUPPORTS_CXX11)\r\nset(CMAKE_CXX_FLAGS \"${CMAKE_CXX_FLAGS} -std=c++11\")\r\nadd_definitions(-DCPP11SUPPORTED)\r\nelseif(COMPILER_SUPPORTS_CXX0X)\r\nset(CMAKE_CXX_FLAGS \"${CMAKE_CXX_FLAGS} -std=c++0x\")\r\nadd_definitions(-DCPP11SUPPORTED)\r\nelse()\r\nmessage(STATUS \"The compiler ${CMAKE_CXX_COMPILER} has no C++11 support. Please use a different C++ compiler.\")\r\nendif()");
        textToAdd[123] = QString("add_executable(test src/code.cpp)\r\ntarget_link_libraries(test ${catkin_LIBRARIES} kukadu kukaduvision)");
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
        cmakeOutFile.rename("/home/agyss/iis_robot_sw/iis_catkin_ws/src/test/CMakeLists.txt");
    }
}
