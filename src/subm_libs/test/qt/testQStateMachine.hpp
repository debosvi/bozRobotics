
#pragma once

#include <subm_libs/qt/QStateMachine.hpp>

using namespace subm_libs::qt;

const QStateIdentifier STATE_1 = 1;
const QStateIdentifier STATE_2 = 2;
const QStateIdentifier STATE_3 = 3;
const QStateIdentifier STATE_4 = 4;

const QTransitionIdentifier TRANSIT_1_2 = 0;
const QTransitionIdentifier TRANSIT_2_FINAL = 1;
const QTransitionIdentifier TRANSIT_3_4 = 0;
const QTransitionIdentifier TRANSIT_4_FINAL = 1;

class MyComplexStateMachine : public QObject {
    Q_OBJECT
    
public: 
    explicit MyComplexStateMachine(QObject *parent=0);
    virtual ~MyComplexStateMachine();
    
    void init();
    void start();
    
    void nextTransition(const int val);
    
private Q_SLOTS:
    void onStateEntered(QStateIdentifier state);
    void onStateExited(QStateIdentifier state);
    void onStarted();
    void onFinished();

    void onNextMachine();

Q_SIGNALS:
    void started();
    void finished();
    
private:
    QStateMachine m1;
    QStateMachine m2;
    QStateMachine* current_m;
};
