#ifndef KUKADU_GUI_H
#define KUKADU_GUI_H

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

    class DatabaseModel : public QAbstractTableModel {

        Q_OBJECT

    private:

        std::string table;
        std::string idColumn;
        std::string whereClause;
        std::vector<std::string> labelColumns;

        std::vector<int> cachedIds;
        std::vector<std::vector<std::string> > cachedData;

        StorageSingleton& storage;

        int getCount();
        void cacheData();

    public:

        DatabaseModel(StorageSingleton& dataStorage, std::string table, std::string idColumn, std::vector<std::string> labelColumns, std::string whereClause = "");

        int getId(const QModelIndex& index);

        QModelIndex getIndex(int robotId);

        int rowCount(const QModelIndex& parent = QModelIndex()) const ;
        int columnCount(const QModelIndex& parent = QModelIndex()) const;
        QVariant data(const QModelIndex& index, int role = Qt::DisplayRole) const;

        void reset();
        void setWhereClause(std::string whereClause);

    };

    class KukaduGui : public QWidget {

        Q_OBJECT

    private:

        static auto constexpr DEFAULT_WIDTH = 800;
        static auto constexpr DEFAULT_HEIGHT = 600;

        StorageSingleton& storage;

        QTabWidget* mainTab;
        QGroupBox* generateRobotBox();

        void currentChanged(const QModelIndex& current, const QModelIndex& previous);

    public:

        explicit KukaduGui(StorageSingleton& dataStorage);

    };

}

#endif
