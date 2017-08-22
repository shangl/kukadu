#ifndef KUKADU_GRAPHICAL_H
#define KUKADU_GRAPHICAL_H

#include <string>
#include <vector>
#include <QtCore/QString>
#include <QComboBox>
#include <QtWidgets/QGroupBox>
#include <QtWidgets/QLineEdit>
#include <QtWidgets/QTabWidget>
#include <QtWebKitWidgets/QWebView>
#include <kukadu/generated_skills/KinestheticTeaching.hpp>


namespace kukadu {

    class KukaduGraphical : public QWidget {
    Q_OBJECT

    private:

        static auto constexpr DEFAULT_WIDTH = 1900;
        static auto constexpr DEFAULT_HEIGHT = 1000;

        QTabWidget *mainTab;
        QWebView *webView;
        QLineEdit *kinestheticSkillName;
        QWidget *kinestheticTeachingView;
        QComboBox *hardwareSelector;
        KUKADU_SHARED_PTR<KinestheticTeaching> teachingObject;

        void loadInformationFromDatabase();

        QGroupBox *createUI();

        std::string selectedTeachingHardware;

        std::string getCatkinMakeString(const std::string &packageName);

        std::vector<QString> getCodeBlocks();

        std::string getPackageName();

        void writeToFileInPackage(std::string filename, QString content);

        void createProjectInPackage();

        bool createProjectInKukadu();

        void writeToFileAtPath(std::string filepath, QString content);

        std::string getCurrentSkillName();

        bool isSkillInstalled();

        std::string getExecutionMode();

        void createCatkinPackage();

        void createCMakeLists();

        void loadMetafilesString();

        bool checkSkillNameIsNew(std::string skillName);

    public:

        explicit KukaduGraphical();

        ~KukaduGraphical();

    public slots:

        void executeSlot();

        void kinestethicTeachingSlot();

        void onStart();

        void goToStartPositionSlot();

        void startKinestheticTeachingSlot();

        void finishedExecutionSlot();

        void testTaughtSkillSlot();

        void installSkillSlot();

        void exitViewSlot();

        void loadSlot();

        void saveSlot();

        void stopExecutionSlot();

        void playSlot();

        void selectionChangedSlot(QString text);
    };

}

#endif // GRAPHICAL_H
