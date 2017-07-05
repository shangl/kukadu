#include <kukadu/gui/graphical.hpp>

#include <sstream>
#include <iostream>
#include <QtWidgets/QLabel>
#include <QtWidgets/QListView>
#include <QtWidgets/QLineEdit>
#include <QtWidgets/QTableView>
#include <QtWidgets/QPushButton>
#include <QtWidgets/QGridLayout>
#include <QtWidgets/QMessageBox>
#include <QtWebKit/QtWebKit>
#include <QtWebKitWidgets/QWebView>
#include <QtWebKitWidgets/QWebPage>
#include <QtWebKitWidgets/QWebFrame>

using namespace std;

namespace kukadu {

    KukaduGraphical::KukaduGraphical() {

        mainTab = new QTabWidget(this);

        auto robotBox = generateWebview();
        mainTab->addTab(robotBox, "Robots");

        mainTab->setMinimumSize(DEFAULT_WIDTH, DEFAULT_HEIGHT);

    }

    KukaduGraphical::~KukaduGraphical() {

    }

    QGroupBox* KukaduGraphical::generateWebview() {
        auto mainGroupBox= new QGroupBox();
        auto layoutOfMainGroupBox= new QGridLayout();

        auto robotListBox = new QWebView(this);


        robotListBox->setMinimumSize(DEFAULT_WIDTH / 3.0, DEFAULT_HEIGHT / 3.0);
        robotListBox->setMaximumSize(DEFAULT_WIDTH / 3.0, DEFAULT_HEIGHT / 3.0);

        layoutOfMainGroupBox->addWidget(robotListBox, 0, 0);

        mainGroupBox->setLayout(layoutOfMainGroupBox);
        mainGroupBox->setMinimumSize(DEFAULT_WIDTH, DEFAULT_HEIGHT);
        mainGroupBox->setMaximumSize(DEFAULT_WIDTH, DEFAULT_HEIGHT);

        return mainGroupBox;
    }
}
