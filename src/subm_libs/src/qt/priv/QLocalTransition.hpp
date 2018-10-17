/**
 * \file        QLocalTransition.hpp
 * \author      Vincent de RIBOU <v.debossoreillederibou@akka.eu>
 * \version     0.1
 * \date        17th October 2018
 * \copyright   BozRobotics.
 * \brief       Definition of public QLocalTransition class.
 */

#include <QtCore/QEvent>
#include <QtCore/QAbstractTransition>

namespace subm_libs {
namespace qt {
namespace priv {
  
struct QLocalEvent : public QEvent {
    QLocalEvent(const unsigned int val) : QEvent(QEvent::Type(QEvent::User+1)), _value(val) {}
    unsigned int _value;
};

class QLocalTransition : public QAbstractTransition {
    Q_OBJECT

public:
    QLocalTransition(const unsigned int value) : _value(value) {}

protected:
    bool eventTest(QEvent *e) override     {
        if (e->type() != QEvent::Type(QEvent::User+1)) // QLocalEvent
            return false;
        QLocalEvent *se = static_cast<QLocalEvent*>(e);
        return (_value == se->_value);
    }

    void onTransition(QEvent *) override {}

private:
    unsigned int _value;
};

}  // namespace priv
}  // namespace qt
}  // namespace subm_libs
