/**
 * \file        QStringTransition.hpp
 * \author      Vincent de RIBOU <v.debossoreillederibou@akka.eu>
 * \version     0.1
 * \date        17th October 2018
 * \copyright   BozRobotics.
 * \brief       Definition of public QStringTransition class.
 */

#include <QtCore/QEvent>
#include <QtCore/QAbstractTransition>

namespace subm_libs {
namespace qt {
  
struct QStringEvent : public QEvent {
    QStringEvent(const QString &val) : QEvent(QEvent::Type(QEvent::User+1)), value(val) {}
    QString value;
};

class QStringTransition : public QAbstractTransition {
    Q_OBJECT

public:
    QStringTransition(const QString &value) : m_value(value) {}

protected:
    bool eventTest(QEvent *e) override     {
        if (e->type() != QEvent::Type(QEvent::User+1)) // QStringEvent
            return false;
        QStringEvent *se = static_cast<QStringEvent*>(e);
        return (m_value == se->value);
    }

    void onTransition(QEvent *) override {}

private:
    QString m_value;
};

}  // namespace qt
}  // namespace subm_libs
