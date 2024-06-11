#include <pch.h>
#include "robotcn.h"
#include <vector>
//
//
//
//void robocn::initialize() {
//    PyStatus status;
//    PyConfig config;
//    PyConfig_InitIsolatedConfig(&config);
//
//    // Set the Python home directory
//    status = PyConfig_SetString(&config, &config.home, L"C:\\Users\\Admin\\AppData\\Local\\Programs\\Python\\Python312");
//    if (PyStatus_Exception(status)) {
//        PyConfig_Clear(&config);
//        Py_ExitStatusException(status);
//    }
//
//    // Initialize Python with the config
//    status = Py_InitializeFromConfig(&config);
//    if (PyStatus_Exception(status)) {
//        PyConfig_Clear(&config);
//        Py_ExitStatusException(status);
//    }
//    PyConfig_Clear(&config);
//
//    // Set up sys.path
//    PyRun_SimpleString("import sys");
//    PyRun_SimpleString("sys.path.append(\"./pysrc\")");
//}
//
//void robocn::connect() {
//    initialize();
//    PyObject* pName = PyUnicode_DecodeFSDefault("test_fs100");
//    pModule = PyImport_Import(pName);
//    Py_DECREF(pName);
//
//    if (!pModule) {
//        PyErr_Print();
//        Py_Finalize();
//        return;
//    }
//
//    PyObject* pClass = PyObject_GetAttrString(pModule, "TestFS100Functions");
//    if (!pClass || !PyCallable_Check(pClass)) {
//        if (PyErr_Occurred())
//            PyErr_Print();
//        std::cerr << "Cannot find class TestFS100Functions" << std::endl;
//        Py_DECREF(pClass);
//        Py_DECREF(pModule);
//        return;
//    }
//
//    PyObject* pArgs = PyTuple_New(0);
//    pRobotInstance = PyObject_CallObject(pClass, pArgs);
//    Py_DECREF(pArgs);
//    Py_DECREF(pClass);
//
//    if (!pRobotInstance) {
//        PyErr_Print();
//    }
//}
//
//void robocn::get_pos() {
//    if (!pRobotInstance) {
//        std::cerr << "Robot is not connected" << std::endl;
//        return;
//    }
//
//    PyObject* pFunc = PyObject_GetAttrString(pRobotInstance, "test_0x75_read_position");
//    if (!pFunc || !PyCallable_Check(pFunc)) {
//        if (PyErr_Occurred())
//            PyErr_Print();
//        std::cerr << "Cannot find method test_0x75_read_position" << std::endl;
//        Py_DECREF(pFunc);
//        return;
//    }
//
//    PyObject* pValue = PyObject_CallObject(pFunc, nullptr);
//    if (!pValue) {
//        PyErr_Print();
//    }
//    else {
//        std::cout << "Position read successfully." << std::endl;
//        Py_DECREF(pValue);
//    }
//
//    Py_DECREF(pFunc);
//}
//
//void robocn::get_tor() {
//    if (!pRobotInstance) {
//        std::cerr << "Robot is not connected" << std::endl;
//        return;
//    }
//
//    PyObject* pFunc = PyObject_GetAttrString(pRobotInstance, "test_0x77_read_torque");
//    if (!pFunc || !PyCallable_Check(pFunc)) {
//        if (PyErr_Occurred())
//            PyErr_Print();
//        std::cerr << "Cannot find method test_0x77_read_torque" << std::endl;
//        Py_DECREF(pFunc);
//        return;
//    }
//
//    PyObject* pValue = PyObject_CallObject(pFunc, nullptr);
//    if (!pValue) {
//        PyErr_Print();
//    }
//    else {
//        std::cout << "Torque read successfully." << std::endl;
//        Py_DECREF(pValue);
//    }
//
//    Py_DECREF(pFunc);
//}
//
//void robocn::set_mov() {
//    if (!pRobotInstance) {
//        std::cerr << "Robot is not connected" << std::endl;
//        return;
//    }
//
//    PyObject* pFunc = PyObject_GetAttrString(pRobotInstance, "test_0x8a_mov");
//    if (!pFunc || !PyCallable_Check(pFunc)) {
//        if (PyErr_Occurred())
//            PyErr_Print();
//        std::cerr << "Cannot find method test_0x8a_mov" << std::endl;
//        Py_DECREF(pFunc);
//        return;
//    }
//
//    PyObject* pValue = PyObject_CallObject(pFunc, nullptr);
//    if (!pValue) {
//        PyErr_Print();
//    }
//    else {
//        std::cout << "Movement executed successfully." << std::endl;
//        Py_DECREF(pValue);
//    }
//
//    Py_DECREF(pFunc);
//}
