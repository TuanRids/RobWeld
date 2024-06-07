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

    void initialize() {
        PyStatus status;
        PyConfig config;
        PyConfig_InitIsolatedConfig(&config);

        // Set the Python home directory
        status = PyConfig_SetString(&config, &config.home, L"C:\\Users\\ngdtu\\AppData\\Local\\Programs\\Python\\Python312");
        if (PyStatus_Exception(status)) {
            PyConfig_Clear(&config);
            Py_ExitStatusException(status);
        }

        // Initialize Python with the config
        status = Py_InitializeFromConfig(&config);
        if (PyStatus_Exception(status)) {
            PyConfig_Clear(&config);
            Py_ExitStatusException(status);
        }
        PyConfig_Clear(&config);

        // Set up sys.path
        PyRun_SimpleString("import sys");
        PyRun_SimpleString("sys.path.append(\"./pysrc\")");
    }

    void connect() {
        initialize();
        PyObject* pName = PyUnicode_DecodeFSDefault("fs100");
        pModule = PyImport_Import(pName);
        Py_DECREF(pName);

        if (!pModule) {
            PyErr_Print();
            Py_Finalize();
            return;
        }

        PyObject* pClass = PyObject_GetAttrString(pModule, "FS100");
        if (!pClass || !PyCallable_Check(pClass)) {
            if (PyErr_Occurred())
                PyErr_Print();
            std::cerr << "Cannot find class FS100" << std::endl;
            Py_DECREF(pClass);
            Py_DECREF(pModule);
            return;
        }

        PyObject* pArgs = PyTuple_Pack(1, PyUnicode_FromString("192.168.10.2"));
        pRobotInstance = PyObject_CallObject(pClass, pArgs);
        Py_DECREF(pArgs);
        Py_DECREF(pClass);

        if (!pRobotInstance) {
            PyErr_Print();
        }
    }

    void get_pos() {
        if (!pRobotInstance) {
            std::cerr << "Robot is not connected" << std::endl;
            return;
        }

        PyObject* pFunc = PyObject_GetAttrString(pRobotInstance, "read_position");
        if (!pFunc || !PyCallable_Check(pFunc)) {
            if (PyErr_Occurred())
                PyErr_Print();
            std::cerr << "Cannot find method read_position" << std::endl;
            Py_DECREF(pFunc);
            return;
        }

        PyObject* pResult = PyDict_New();
        PyObject* pArgs = PyTuple_Pack(1, pResult);
        PyObject* pValue = PyObject_CallObject(pFunc, pArgs);
        Py_DECREF(pArgs);
        Py_DECREF(pFunc);

        if (!pValue) {
            PyErr_Print();
            return;
        }

        PyObject* pPos = PyDict_GetItemString(pResult, "pos");
        if (pPos) {
            Py_ssize_t size = PyTuple_Size(pPos);
            for (Py_ssize_t i = 0; i < size; i++) {
                PyObject* pItem = PyTuple_GetItem(pPos, i);
                std::cout << "Axis " << i + 1 << ": " << PyLong_AsLong(pItem) << std::endl;
            }
        }
        else {
            std::cerr << "Failed to get position data" << std::endl;
        }

        Py_DECREF(pValue);
        Py_DECREF(pResult);
    }

    void get_tor() {
        if (!pRobotInstance) {
            std::cerr << "Robot is not connected" << std::endl;
            return;
        }

        PyObject* pFunc = PyObject_GetAttrString(pRobotInstance, "read_torque");
        if (!pFunc || !PyCallable_Check(pFunc)) {
            if (PyErr_Occurred())
                PyErr_Print();
            std::cerr << "Cannot find method read_torque" << std::endl;
            Py_DECREF(pFunc);
            return;
        }

        PyObject* pResult = PyDict_New();
        PyObject* pArgs = PyTuple_Pack(1, pResult);
        PyObject* pValue = PyObject_CallObject(pFunc, pArgs);
        Py_DECREF(pArgs);
        Py_DECREF(pFunc);

        if (!pValue) {
            PyErr_Print();
            return;
        }

        PyObject* pTorque = PyDict_GetItemString(pResult, "torque");
        if (pTorque) {
            Py_ssize_t size = PyTuple_Size(pTorque);
            for (Py_ssize_t i = 0; i < size; i++) {
                PyObject* pItem = PyTuple_GetItem(pTorque, i);
                std::cout << "Torque " << i + 1 << ": " << PyLong_AsLong(pItem) << std::endl;
            }
        }
        else {
            std::cerr << "Failed to get torque data" << std::endl;
        }

        Py_DECREF(pValue);
        Py_DECREF(pResult);
    }
};
