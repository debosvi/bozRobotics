/**
 * \file        Singleton.h
 * \author      Vincent de RIBOU <v.debossoreillederibou@akka.eu>
 * \version     0.1
 * \date        20th June 2018
 * \copyright   Renault SAS.
 * \brief       Definition of main application to gateway ROS messages to Ethenet UDP.
 */

#ifndef __ADCCLIBS_SINGLETON_H__
#define __ADCCLIBS_SINGLETON_H__

#include <QtCore/QObject>

template<typename T>
class Singleton
{
protected:
    Singleton()
    {
    }
    virtual ~Singleton() { }
    Singleton(const Singleton &);
    Singleton& operator=(const Singleton &);

public:
    static T *instance(QObject *parent)
    {
        if (!_instance)
        {
            _instance = new T(parent);
        }

        return _instance;
    }

    static T *instance(int argc, char **argv)
    {
        if (!_instance)
        {
            _instance = new T(argc, argv);
        }

        return _instance;
    }

    static T *instance()
    {
        if (!_instance)
        {
            _instance = new T;
        }

        return _instance;
    }
    
    static void release()
    {
        if (_instance)
        {
            delete _instance;
            _instance = NULL;
        }
    }

private:
    static T *_instance;
};

template<typename T>
T *Singleton<T>::_instance = NULL;

#endif // __ADCCLIBS_SINGLETON_H__
