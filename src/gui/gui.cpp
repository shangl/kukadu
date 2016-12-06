#include <kukadu/gui/gui.hpp>
#include <kukadu/robot/robot.hpp>

#include <sstream>
#include <iostream>
#include <QtWidgets/QLabel>
#include <QtWidgets/QListView>
#include <QtWidgets/QLineEdit>
#include <QtWidgets/QTableView>
#include <QtWidgets/QPushButton>
#include <QtWidgets/QGridLayout>
#include <QtWidgets/QMessageBox>

using namespace std;

namespace kukadu {

    void KukaduGui::currentChanged(const QModelIndex& current, const QModelIndex& previous) {
        current.row();

    }

    KukaduGui::KukaduGui(StorageSingleton& dataStorage) : storage(dataStorage) {

        mainTab = new QTabWidget(this);

        auto robotBox = generateRobotBox();
        mainTab->addTab(robotBox, "Robots");

        mainTab->setMinimumSize(DEFAULT_WIDTH, DEFAULT_HEIGHT);

    }

    QGroupBox* KukaduGui::generateRobotBox() {

        auto robotBox = new QGroupBox();
        auto robotListBox = new QGroupBox();
        auto robotAddBox = new QGroupBox();

        auto mainLayout = new QGridLayout();

        /*************** robot listing ******************/
        auto boxesLayout = new QGridLayout();
        auto robotList = new QListView();
        robotList->setAutoScroll(true);
        auto robotListModel = new DatabaseModel(storage, "robot", "robot_id", {"robot_id", "robot_name"});
        robotList->setModel(robotListModel);

        auto firstIndex = robotListModel->index(0, 0);
        robotList->setCurrentIndex(firstIndex);

        auto firstId = robotListModel->getId(firstIndex);
        stringstream s;
        s << "robot_id = " << firstId << endl;

        auto jointList = new QListView();
        jointList->setAutoScroll(true);
        auto jointListModel = new DatabaseModel(storage, "robot_joints", "joint_id", {"joint_id", "joint_name"}, s.str());
        jointList->setModel(jointListModel);

        //connect(robotList->selectionModel(), &QItemSelectionModel::currentChanged, this, &KukaduGui::currentChanged);
        connect(robotList->selectionModel(), &QItemSelectionModel::currentChanged,
                [robotListModel, jointListModel](const QModelIndex& current, const QModelIndex& previous) {
                    auto robotId = robotListModel->getId(current);
                    stringstream s;
                    s << "robot_id = " << robotId << endl;
                    jointListModel->setWhereClause(s.str());
                }
        );

        boxesLayout->addWidget(robotList, 0, 0);
        boxesLayout->addWidget(jointList, 0, 1);

        robotListBox->setLayout(boxesLayout);
        robotListBox->setMinimumSize(DEFAULT_WIDTH / 3.0, DEFAULT_HEIGHT / 3.0);
        robotListBox->setMaximumSize(DEFAULT_WIDTH / 3.0, DEFAULT_HEIGHT / 3.0);
        robotListBox->setTitle(QString("Available robots"));

        /*************** robot edit box ******************/
        auto addRobotLayout = new QGridLayout();
        robotAddBox->setTitle(QString("Add / edit a robot"));
        auto robotNameField = new QLineEdit();
        auto addRobotButton = new QPushButton("Add robot");
        auto robotNameLabel = new QLabel("Robot name: ");

        auto jointNameField = new QLineEdit();
        auto addJointButton = new QPushButton("Add joint");
        auto jointNameLabel = new QLabel("Joint name: ");

        connect(addJointButton, &QPushButton::clicked,
                [this, robotNameField, robotListModel, jointListModel](bool checked) {
                    auto robotName = robotNameField->text().toStdString();
                    if(robotName != "") {
                        if(Robot::checkRobotExists(storage, robotName)) {
                            auto selectedRobot = Robot(storage, robotName);
                            auto robotId = selectedRobot.getRobotId();
                            stringstream s;
                            s << "robot_id = " << robotId << endl;
                            jointListModel->setWhereClause(s.str());
                        } else {
                            QMessageBox msgBox;
                            msgBox.setText(QString((string("Cannot add joint to non-existant robot (") + robotName + string("). First create the robot.")).c_str()));
                            msgBox.setStandardButtons(QMessageBox::Ok);
                            msgBox.exec();
                        }
                    } else {
                        QMessageBox msgBox;
                        msgBox.setText(QString((string("No robot defined").c_str())));
                        msgBox.setStandardButtons(QMessageBox::Ok);
                        msgBox.exec();
                    }
                }
        );

        connect(addRobotButton, &QPushButton::clicked,
                [robotListModel, jointListModel](bool checked) {
                    jointListModel;
                }
        );

        robotNameField->setMaximumWidth(DEFAULT_WIDTH / 7);
        robotNameLabel->setMaximumWidth(DEFAULT_WIDTH / 7);

        jointNameField->setMaximumWidth(robotNameField->width());

        addRobotLayout->addWidget(robotNameLabel, 0, 0);
        addRobotLayout->addWidget(robotNameField, 0, 1);
        addRobotLayout->addWidget(addRobotButton, 2, 0);

        addRobotLayout->addWidget(jointNameLabel, 1, 0);
        addRobotLayout->addWidget(jointNameField, 1, 1);
        addRobotLayout->addWidget(addJointButton, 2, 1);

        robotAddBox->setLayout(addRobotLayout);

        robotAddBox->setMinimumSize(DEFAULT_WIDTH / 3.0, DEFAULT_HEIGHT / 3.0);
        robotAddBox->setMaximumSize(DEFAULT_WIDTH / 3.0, DEFAULT_HEIGHT / 3.0);

        /*************** sticking layouts together ******************/
        mainLayout->addWidget(robotListBox, 0, 0);
        mainLayout->addWidget(robotAddBox, 1, 0);

        robotBox->setLayout(mainLayout);
        robotBox->setMinimumSize(DEFAULT_WIDTH, DEFAULT_HEIGHT);
        robotBox->setMaximumSize(DEFAULT_WIDTH, DEFAULT_HEIGHT);

        return robotBox;

    }

    DatabaseModel::DatabaseModel(StorageSingleton& dataStorage, string table, string idColumn, std::vector<string> labelColumns, std::string whereClause) : storage(dataStorage) {

        this->table = table;
        this->idColumn = idColumn;
        this->whereClause = whereClause;
        this->labelColumns = labelColumns;

        cacheData();

    }

    int DatabaseModel::getId(const QModelIndex& index) {
        return cachedIds.at(index.row());
    }

    int DatabaseModel::getCount() {
        return cachedData.size();
    }

    void DatabaseModel::setWhereClause(std::string whereClause) {

        beginResetModel();

        this->whereClause = whereClause;
        cacheData();

        endResetModel();

    }

    void DatabaseModel::cacheData() {

        cachedIds.clear();
        cachedData.clear();
        string rowCountSql{"select "};
        for(const auto& col : labelColumns)
            rowCountSql += col + ", ";

        // retrieve index
        rowCountSql += idColumn + " as databasemodel_id";

        rowCountSql += " from " + table + ((whereClause != "") ? " where " : "") + whereClause;
        auto rowRes = storage.executeQuery(rowCountSql);

        while(rowRes->next()) {
            vector<string> cacheRow;
            for(const auto& col : labelColumns)
                cacheRow.push_back(rowRes->getString(col));
            cachedData.push_back(cacheRow);
            cachedIds.push_back(rowRes->getInt("databasemodel_id"));
        }

    }

    int DatabaseModel::rowCount(const QModelIndex& parent) const {
        return cachedData.size();
    }

    int DatabaseModel::columnCount(const QModelIndex& parent) const {
        return 1;
    }

    QVariant DatabaseModel::data(const QModelIndex& index, int role) const {

        if(role == Qt::DisplayRole) {

            auto& row = cachedData.at(index.row());
            string rowString{""};

            for(int i = 0; i < row.size() - 1; ++i)
                rowString += row.at(i) + " - ";

            rowString += row.back();
            return QString(rowString.c_str());

        }
        return QVariant();
    }

}
