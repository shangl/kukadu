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

#include <string>
#include <vector>
#include <kukadu/types/kukadutypes.hpp>
#include <kukadu/storage/storagesingleton.hpp>

namespace kukadu {

    class KukaduGraphical : public QWidget {
        Q_OBJECT

    private:

        static auto constexpr DEFAULT_WIDTH = 1600;
        static auto constexpr DEFAULT_HEIGHT = 800;

        QTabWidget* mainTab;
        QWebView* webView;

        QGroupBox* createUI();

    public:

        explicit KukaduGraphical();
        ~KukaduGraphical();

    public slots:

      void clickedSlot();

    };

}

#endif // GRAPHICAL_H
