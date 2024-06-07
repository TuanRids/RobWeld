#ifndef ROBOCN_H
#define ROBOCN_H

#include "python/Python.h"
#include <iostream>

class robocn {
private:
    PyObject* pModule;
    PyObject* pRobotInstance;

public:
    robocn() : pModule(nullptr), pRobotInstance(nullptr) {}

    ~robocn() {
        Py_XDECREF(pRobotInstance);
        Py_XDECREF(pModule);
        Py_Finalize();
    }

    void initialize();
    void connect();
    void get_pos();
    void get_tor();
    void set_mov();
};

#endif // ROBOCN_H
