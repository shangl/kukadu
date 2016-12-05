#include <kukadu/gui/gui.hpp>

KukaduGui::KukaduGui(int argc, char** args) : qapp{argc, args} {

    QPushButton button("Hello world!");
    button.show();

    qapp.exec();

}

KukaduGui::KukaduGui(QApplication& app) : qapp{app} {

    QPushButton button("Hello world!");
    button.show();

    qapp.exec();

}

KukaduGui::~KukaduGui() {

}
