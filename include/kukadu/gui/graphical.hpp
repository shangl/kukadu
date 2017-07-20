#ifndef KUKADU_GRAPHICAL_H
#define KUKADU_GRAPHICAL_H

#include <QtWidgets/QLayout>
#include <QtCore/QString>
#include <QtWidgets/QWidget>
#include <QAbstractTableModel>
#include <QtWidgets/QGroupBox>
#include <QItemSelectionModel>
#include <QtWidgets/QTabWidget>
#include <QtWebKitWidgets/QWebView>
#include <QtWebKitWidgets/QWebPage>
#include <QtWebKitWidgets/QWebFrame>
#include <stdio.h>
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
#include <kukadu/utils/utils.hpp>
#include <json.hpp>

#include <string>
#include <vector>
#include <kukadu/types/kukadutypes.hpp>
#include <kukadu/storage/storagesingleton.hpp>

namespace kukadu {

    class KukaduGraphical : public QWidget {
        Q_OBJECT

    private:

        static auto constexpr DEFAULT_WIDTH = 1800;
        static auto constexpr DEFAULT_HEIGHT = 900;

        QTabWidget* mainTab;
        QWebView* webView;
        QLineEdit* packeNameLineEdit;

        void loadInformationFromDatabase();
        QGroupBox* createUI();
        std::string getCatkinMakeString(const std::string& packageName);
        std::vector< QString > getCodeBlocks();
        std::string getPackageName();
        void writeToFileInPackage(std::string filename, QString content);
        std::string getCurrentSkillName();

    public:

        explicit KukaduGraphical();
        ~KukaduGraphical();

    public slots:
      void simulateSkillSlot();
      void executeSkillSlot();
      void installSkillSlot();
      void onStart();

    };

}

#endif // GRAPHICAL_H
