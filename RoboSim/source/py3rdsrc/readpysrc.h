

#include "python/Python.h"
#include <iostream>
#pragma warning( push )
#pragma warning( disable : 26819) //3rd party library
#include "nlohmann/json.hpp"
#pragma warning( pop )
using json = nlohmann::json;

class readpysrc {
private:
    PyObject* pModule;
    PyObject* pRobotInstance;
    std::string pvalue;
    std::string pypath;

public:
    readpysrc() : pModule(nullptr), pRobotInstance(nullptr) {}

    ~readpysrc() {
        Py_XDECREF(pRobotInstance);
        Py_XDECREF(pModule);
        Py_Finalize();
    }

    bool initialize();
    std::vector<std::vector<double>> get_values_from_python();
    std::vector<std::vector<double>> get_values_from_exe();

};



