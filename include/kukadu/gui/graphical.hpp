#ifndef KUKADU_GRAPHICAL_H
#define KUKADU_GRAPHICAL_H

#include <string>
#include <vector>
#include <QComboBox>
#include <QtCore/QString>
#include <QtWidgets/QGroupBox>
#include <QtWidgets/QLineEdit>
#include <QtWidgets/QTabWidget>
#include <QtWebKitWidgets/QWebView>
#include <kukadu/manipulation/playing/core.hpp>
#include <kukadu/generated_skills/KinestheticTeaching.hpp>

namespace kukadu {

    class KukaduGraphical : public QWidget {
    Q_OBJECT

    private:

        static auto constexpr DEFAULT_WIDTH = 1900;
        static auto constexpr DEFAULT_HEIGHT = 1000;

        QTabWidget* mainTab;

        QWidget* playingView;
        QWidget* kinestheticTeachingView;

        QWebView* webView;

        QComboBox* hardwareSelector;

        QLineEdit* kinestheticSkillName;
        QLineEdit* usedSensingBehaviours;
        QLineEdit* usedPlayingBehaviours;

        KUKADU_SHARED_PTR<KinestheticTeaching> teachingObject;

        bool keepPlaying;
        bool playingEnded;
        std::string playingControllerName;
        KUKADU_SHARED_PTR<Controller> playingController;
        KUKADU_SHARED_PTR<HapticPlanner> currentHapticPlanner;

        std::string selectedTeachingHardware;

        void createCMakeLists();
        void createCatkinPackage();
        void loadMetafilesString();
        void createProjectInPackage();
        void loadInformationFromDatabase();
        void writeToFileAtPath(std::string filepath, QString content);
        void writeToFileInPackage(std::string filename, QString content);

        bool isSkillInstalled();
        bool createProjectInKukadu();
        bool checkSkillNameIsNew(std::string skillName);

        std::string getPackageName();
        std::string getExecutionMode();
        std::string getCurrentSkillName();
        std::string getCatkinMakeString(const std::string &packageName);

        std::vector<QString> getCodeBlocks();

        QGroupBox *createUI();

    public:

        explicit KukaduGraphical();

        ~KukaduGraphical();

    public slots:

        void onStart();
        void playSlot();
        void saveSlot();
        void loadSlot();
        void executeSlot();
        void exitViewSlot();
        void installSkillSlot();
        void stopExecutionSlot();
        void testTaughtSkillSlot();
        void finishedExecutionSlot();
        void goToStartPositionSlot();
        void kinestethicTeachingSlot();
        void startKinestheticTeachingSlot();

        void exitPlayingslot();
        void performPlayingSlot();
        void trainPerceptualStatesSlot();

        void selectionChangedSlot(QString text);

    };

}

#endif // GRAPHICAL_H
