#include <kukadu/gui/gui.hpp>

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

    void showInfoBox(std::string message) {
        QMessageBox msgBox;
        msgBox.setText(QString(message.c_str()));
        msgBox.setStandardButtons(QMessageBox::Ok);
        msgBox.exec();
    }

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
        stringstream s;
        if(firstIndex.row() >= 0) {
            auto firstId = robotListModel->getId(firstIndex);
            s << "robot_id = " << firstId << endl;
        }

        auto jointList = new QListView();
        jointList->setAutoScroll(true);
        auto jointListModel = new DatabaseModel(storage, "robot_joints", "joint_id", {"joint_id", "joint_name"}, s.str());
        jointList->setModel(jointListModel);

        auto deleteRobotButton = new QPushButton("Delete robot");
        auto deleteJointButton = new QPushButton("Delete joint");

        boxesLayout->addWidget(robotList, 0, 0);
        boxesLayout->addWidget(jointList, 0, 1);
        boxesLayout->addWidget(deleteRobotButton, 1, 0);
        boxesLayout->addWidget(deleteJointButton, 1, 1);

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

        /*************** set event handlers ******************/

        // handler for showing another robot
        connect(robotList->selectionModel(), &QItemSelectionModel::currentChanged,
                [this, robotListModel, jointListModel, robotNameField](const QModelIndex& current, const QModelIndex& previous) {
/*
                    auto robotId = robotListModel->getId(current);
                    auto selectedRobot = Robot(storage, robotId);
                    stringstream s;
                    s << "robot_id = " << robotId << endl;
                    jointListModel->setWhereClause(s.str());
                    robotNameField->setText(QString(selectedRobot.getRobotName().c_str()));
*/
                }
        );

        // handler for adding a new joint
        connect(addJointButton, &QPushButton::clicked,
                [this, robotNameField, jointNameField, robotList, robotListModel, jointListModel](bool checked) {
/*
                    auto robotName = robotNameField->text().toStdString();
                    auto jointName = jointNameField->text().toStdString();
                    if(jointName != "") {
                        if(robotName != "") {
                            if(Robot::checkRobotExists(storage, robotName)) {
                                auto selectedRobot = Robot(storage, robotName);
                                auto robotId = selectedRobot.getRobotId();
                                robotList->setCurrentIndex(robotListModel->getIndex(robotId));
                                if(selectedRobot.insertJoint(jointName))
                                    jointListModel->reset();
                                else showInfoBox("The robot " + robotName + " already has a joint named " + jointName);
                            } else showInfoBox("Cannot add joint to non-existent robot (" + robotName + "). First create the robot");
                        } else showInfoBox("No robot name defined");
                    } else showInfoBox("No joint name defined");
*/
                }
        );

        // handler for adding a new robot
        connect(addRobotButton, &QPushButton::clicked,
                [this, robotNameField, robotListModel, jointListModel](bool checked) {
/*
                    auto robotName = robotNameField->text().toStdString();
                    if(robotName != "") {
                        if(Robot::createRobot(storage, robotName))
                            robotListModel->reset();
                        else showInfoBox("Could not create robot (maybe a robot with that name already exists?)");
                    } else showInfoBox("No robot name defined");
*/
                }
        );

        // handler for deleting a robot
        connect(deleteRobotButton, &QPushButton::clicked,
                [this, robotList, jointList, robotListModel, jointListModel](bool checked) {
/*
                    auto robotIndex = robotList->currentIndex();

                    if(robotIndex.row() >= 0) {

                        auto robotId = robotListModel->getId(robotIndex);
                        Robot currentRobot(storage, robotId);
                        if(currentRobot.deleteRobot()) {
                            robotListModel->reset();
                            if(robotListModel->rowCount()) {
                                auto firstIndex = robotListModel->index(0, 0);
                                robotList->setCurrentIndex(firstIndex);
                            }
                        } else showInfoBox("Couldn't delete robot " + currentRobot.getRobotName());

                        // if no robot there, reset also the joint list
                        if(robotListModel->rowCount() == 0)
                            jointListModel->reset();

                    } else showInfoBox("No robot selected");
*/
                }
        );

        // handler for deleting a joint
        connect(deleteJointButton, &QPushButton::clicked,
                [this, robotList, jointList, robotListModel, jointListModel](bool checked) {
/*
                    auto robotIndex = robotList->currentIndex();
                    auto jointIndex = jointList->currentIndex();

                    if(robotIndex.row() >= 0) {

                        if(jointIndex.row() >= 0) {

                            auto robotId = robotListModel->getId(robotIndex);
                            auto jointId = jointListModel->getId(jointIndex);
                            Robot currentRobot(storage, robotId);
                            if(currentRobot.deleteJoint(jointId))
                                jointListModel->reset();
                            else showInfoBox("Couldn't delete joint " + currentRobot.getJointName(jointId));

                        } else showInfoBox("No joint selected");

                    } else showInfoBox("No robot selected");
*/
                }
        );

        // select first element
        if(robotListModel->rowCount())
            robotList->setCurrentIndex(firstIndex);

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

    QModelIndex DatabaseModel::getIndex(int robotId) {
        return index(std::find(cachedIds.begin(), cachedIds.end(), robotId) - cachedIds.begin(), 0);
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

    void DatabaseModel::reset() {
        beginResetModel();
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
