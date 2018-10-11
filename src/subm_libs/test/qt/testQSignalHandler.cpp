
#include <iostream>

#include <QtCore/QCoreApplication>

#include <subm_libs/qt/QSignalHandler.hpp>

static void onSigInt() {
    std::cerr << __FUNCTION__ << std::endl;
}

static void onSigTerm() {
    std::cerr << __FUNCTION__ << std::endl;
}

static void onSigHup() {
    std::cerr << __FUNCTION__ << std::endl;
}

using namespace subm_libs::qt;

int main(int ac, char** av) {
    const char *node_name = "mytopic_subscriber2";

    QCoreApplication app(ac, av);
    QCoreApplication::setApplicationName(node_name);
    QCoreApplication::setApplicationVersion("1.0");
    
    QSignalHandler qsh;
    qsh.init();
    
    QObject::connect(&qsh, &QSignalHandler::sigINT, onSigInt);
    QObject::connect(&qsh, SIGNAL(sigINT()), &app, SLOT(quit()));
    QObject::connect(&qsh, &QSignalHandler::sigHUP, onSigHup);
    QObject::connect(&qsh, &QSignalHandler::sigTERM, onSigTerm);
    
    return app.exec();
}
