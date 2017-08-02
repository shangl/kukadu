#ifndef KUKADU_GRAPHICAL_H
#define KUKADU_GRAPHICAL_H

#include <QtWidgets/QTabWidget>
#include <QtWebKitWidgets/QWebView>
#include <QtWidgets/QLineEdit>
#include <QtWidgets/QGroupBox>
#include <string>
#include <vector>
#include <QtCore/QString>


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
        void createProjectInKukadu();
        void writeToFileAtPath(std::string filepath, QString content);
        std::string getCurrentSkillName();
        bool isSkillInstalled();

    public:

        explicit KukaduGraphical();
        ~KukaduGraphical();

    public slots:
      void executeSlot();
      void onStart();

    };

}

#endif // GRAPHICAL_H
