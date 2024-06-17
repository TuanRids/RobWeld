#include <pch.h>
#include <vector>
#include <windows.h>
#include <shlobj.h>  // For SHBrowseForFolder
#include <iostream>
#include "readpysrc.h"
std::wstring BrowseFolder() {
    BROWSEINFOW bi = { 0 };
    bi.lpszTitle = L"Select Python Installation Directory";
    LPITEMIDLIST pidl = SHBrowseForFolderW(&bi);
    if (pidl != 0) {
        wchar_t path[MAX_PATH];
        if (SHGetPathFromIDListW(pidl, path)) {
            CoTaskMemFree(pidl);
            return std::wstring(path);
        }
        CoTaskMemFree(pidl);
    }
    return L"";
}

void readpysrc::initialize() {
    PyStatus status;
    PyConfig config;
    PyConfig_InitIsolatedConfig(&config);

    wchar_t pythonHome[512];
    DWORD size = GetEnvironmentVariableW(L"PYTHON_HOME", pythonHome, 512);
    if (size == 0) {
        std::wcerr << L"Failed to get PYTHON_HOME environment variable" << std::endl;

        // std::wstring selectedPath = BrowseFolder();
        // C:\Users\Admin\AppData\Local\Programs\Python\Python312
		std::wstring selectedPath = L"C:\\Users\\Admin\\AppData\\Local\\Programs\\Python\\Python312";
        if (selectedPath.empty()) {
            std::wcerr << L"No directory selected. Exiting." << std::endl;
            return;
        }
        wcscpy_s(pythonHome, selectedPath.c_str());
    }

    status = PyConfig_SetString(&config, &config.home, pythonHome);
    if (PyStatus_Exception(status)) {
        PyConfig_Clear(&config);
        Py_ExitStatusException(status);
    }

    status = Py_InitializeFromConfig(&config);
    if (PyStatus_Exception(status)) {
        PyConfig_Clear(&config);
        Py_ExitStatusException(status);
    }
    PyConfig_Clear(&config);

    PyRun_SimpleString("import sys");
    PyRun_SimpleString("sys.path.append(\"./pysrc\")");
}


std::vector<std::vector<double>> readpysrc::get_values_from_python() {
    std::vector<std::vector<double>> get_result;
    initialize();
    PyObject* pName = PyUnicode_DecodeFSDefault("postocpp");
    PyObject* pModule = PyImport_Import(pName);
    Py_DECREF(pName);

    if (!pModule) {
        PyErr_Print();
        Py_Finalize();
        return get_result;
    }

    PyObject* pFunc = PyObject_GetAttrString(pModule, "give_to_cpp");
    if (!pFunc || !PyCallable_Check(pFunc)) {
        if (PyErr_Occurred())
            PyErr_Print();
        std::cerr << "Cannot find method give_to_cpp" << std::endl;
        Py_XDECREF(pFunc);
        Py_DECREF(pModule);
        Py_Finalize();
        return get_result;
    }

    PyObject* pValue = PyObject_CallObject(pFunc, nullptr);
    if (!pValue) {
        PyErr_Print();
    }
    else {
        if (PyList_Check(pValue)) {
            Py_ssize_t outer_size = PyList_Size(pValue);
            for (Py_ssize_t i = 0; i < outer_size; ++i) {
                PyObject* inner_list = PyList_GetItem(pValue, i);
                if (PyList_Check(inner_list)) {
                    std::vector<double> inner_vector;
                    Py_ssize_t inner_size = PyList_Size(inner_list);
                    for (Py_ssize_t j = 0; j < inner_size; ++j) {
                        PyObject* item = PyList_GetItem(inner_list, j);
                        if (PyFloat_Check(item)) {
                            inner_vector.push_back(PyFloat_AsDouble(item));
                        }
                    }
                    get_result.push_back(inner_vector);
                }
            }
        }
        Py_DECREF(pValue);
    }

    Py_DECREF(pFunc);
    Py_DECREF(pModule);
    Py_Finalize();

    return get_result;
}


