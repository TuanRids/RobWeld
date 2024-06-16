

#include "python/Python.h"
#include <iostream>

class readpysrc {
private:
    PyObject* pModule;
    PyObject* pRobotInstance;
    std::string pvalue;

public:
    readpysrc() : pModule(nullptr), pRobotInstance(nullptr) {}

    ~readpysrc() {
        Py_XDECREF(pRobotInstance);
        Py_XDECREF(pModule);
        Py_Finalize();
    }

    void initialize();
    std::vector<std::vector<double>> get_values_from_python();

};



