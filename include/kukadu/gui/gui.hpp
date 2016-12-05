#ifndef KUKADU_GUI_H
#define KUKADU_GUI_H

#include <QWidget>
#include <QPushButton>
#include <QApplication>

class KukaduGui : public QWidget {

    Q_OBJECT

    const QApplication& qapp;

public:

    explicit KukaduGui(int argc, char** args);
    explicit KukaduGui(QApplication& app);

};

#endif
