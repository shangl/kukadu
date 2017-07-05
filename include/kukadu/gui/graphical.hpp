#ifndef KUKADU_GRAPHICAL_H
#define KUKADU_GRAPHICAL_H

#include <QtWidgets/QWidget>
#include <QAbstractTableModel>
#include <QtWidgets/QGroupBox>
#include <QItemSelectionModel>
#include <QtWidgets/QTabWidget>

#include <string>
#include <vector>
#include <kukadu/types/kukadutypes.hpp>
#include <kukadu/storage/storagesingleton.hpp>

namespace kukadu {

    class KukaduGraphical : public QWidget {

        Q_OBJECT

    private:

        static auto constexpr DEFAULT_WIDTH = 800;
        static auto constexpr DEFAULT_HEIGHT = 600;

        QTabWidget* mainTab;

        QGroupBox* generateWebview();

    public:

        explicit KukaduGraphical();
        ~KukaduGraphical();

    };

}

#endif // GRAPHICAL_H
