#include <string>
#include <iostream>
#include <armadillo>
#include <kukadu/kukadu.hpp>

#include <QStyle>
#include <QStyleFactory>
#include <QtWidgets/QApplication>

using namespace std;
using namespace arma;
using namespace kukadu;

int main(int argc, char** args) {

    ros::init(argc, args, "kukadu"); ros::NodeHandle* node = new ros::NodeHandle(); usleep(1e6);
    ros::AsyncSpinner spinner(10);
    spinner.start();

    StorageSingleton& storage = StorageSingleton::get();

    QApplication w(argc, args);
    w.setStyle(QStyleFactory::create("Fusion"));

    KukaduGui g(storage);
    g.show();

    return w.exec();

}
