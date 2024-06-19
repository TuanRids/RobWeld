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

bool readpysrc::initialize() 
{
    PyStatus status;
    PyConfig config;
    PyConfig_InitIsolatedConfig(&config);

    wchar_t pythonHome[512];
    DWORD size = GetEnvironmentVariableW(L"PYTHON_HOME", pythonHome, 512);
    std::wstring selectedPath;


    if (size == 0) {
        std::wcerr << L"Failed to get PYTHON_HOME environment variable" << std::endl;

        // Read from ini file
        std::ifstream inputFile("robosim_ini.dat");
        json j;
        if (inputFile.is_open()) {
            inputFile >> j;
            inputFile.close();
            if (j.find("pypath") != j.end()) {
                std::string getp = j["pypath"].get<std::string>();
                selectedPath = std::wstring(getp.begin(), getp.end());
                // check if selectedpath is exists or not, if not set selectedpath = empty
                std::error_code ec;
                if (!std::filesystem::exists(selectedPath, ec)) {
                    selectedPath.clear();
                }
            }
        }

        if (selectedPath.empty()) {
            // Browse for path
            selectedPath = BrowseFolder();
            if (selectedPath.empty()) {
                MessageBox(nullptr, "No directory selected. Exiting.", "Error", MB_OK | MB_ICONERROR);
                return false;
            }

            // Save path to JSON file
            j["pypath"] = std::string(selectedPath.begin(), selectedPath.end());
            std::ofstream outputFile("robosim_ini.dat");
            if (!outputFile.is_open()) {
                MessageBox(nullptr, "Failed to open file for writing: robosim_ini.dat", "Error", MB_OK | MB_ICONERROR);
                return false;
            }
            outputFile << j.dump(4);
            outputFile.close();
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
    return true;
}



std::vector<std::vector<double>> readpysrc::get_values_from_python() {
    std::vector<std::vector<double>> get_result;
    if (!initialize())
    {
        MessageBox(nullptr, "Failed to initialize Python", "Error", MB_OK | MB_ICONERROR);
        return get_result;
    }
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

std::vector<std::vector<double>> readpysrc::get_values_from_exe()
{
    std::wstring exePath = L"pysrc\\postocpp.exe";
    std::vector<std::vector<double>> get_result;
    if (exePath.empty()) {
        MessageBox(nullptr, "Failed to get path to postocpp.exe", "Error", MB_OK | MB_ICONERROR);
        return get_result;
    }

    // Initialize structures for CreateProcess
    STARTUPINFOW si;
    PROCESS_INFORMATION pi;
    SECURITY_ATTRIBUTES sa;
    HANDLE g_hChildStd_OUT_Rd = NULL;
    HANDLE g_hChildStd_OUT_Wr = NULL;

    ZeroMemory(&si, sizeof(si));
    si.cb = sizeof(si);
    si.dwFlags |= STARTF_USESTDHANDLES;

    ZeroMemory(&pi, sizeof(pi));

    // Set security attributes for the pipe
    sa.nLength = sizeof(SECURITY_ATTRIBUTES);
    sa.bInheritHandle = TRUE;
    sa.lpSecurityDescriptor = NULL;

    // Create a pipe for the child process's STDOUT
    if (!CreatePipe(&g_hChildStd_OUT_Rd, &g_hChildStd_OUT_Wr, &sa, 0)) {
        std::cerr << "Stdout pipe creation failed\n";
        return get_result;
    }

    // Ensure the read handle to the pipe is not inherited
    if (!SetHandleInformation(g_hChildStd_OUT_Rd, HANDLE_FLAG_INHERIT, 0)) {
        std::cerr << "Stdout pipe set handle information failed\n";
        return get_result;
    }

    // Set the handles for STDOUT and STDERR for the child process
    si.hStdOutput = g_hChildStd_OUT_Wr;
    si.hStdError = g_hChildStd_OUT_Wr;

    // Launch the child process using the Unicode version explicitly
    if (!CreateProcessW(NULL, const_cast<LPWSTR>(exePath.c_str()), NULL, NULL, TRUE, 0, NULL, NULL, &si, &pi)) {
        std::cerr << "CreateProcess failed\n";
        return get_result;
    }

    // Close the handle to the write end of the pipe in the parent process
    CloseHandle(g_hChildStd_OUT_Wr);

    // Read output from the child process's pipe
    DWORD dwRead;
    CHAR chBuf[4096];
    BOOL bSuccess = FALSE;
    std::string result;

    for (;;) {
        bSuccess = ReadFile(g_hChildStd_OUT_Rd, chBuf, sizeof(chBuf), &dwRead, NULL);
        if (!bSuccess || dwRead == 0) break;
        result.append(chBuf, dwRead);
    }

    // Wait for the child process to exit
    WaitForSingleObject(pi.hProcess, INFINITE);

    // Close handles for the child process
    CloseHandle(pi.hProcess);
    CloseHandle(pi.hThread);

    // Parse the result string into a 2D vector of doubles
    std::istringstream iss(result);
    std::string line;
    while (std::getline(iss, line)) {
        std::istringstream lineStream(line);
        std::vector<double> inner_vector;
        double value;
        while (lineStream >> value) {
            inner_vector.push_back(value);
        }
        if (!inner_vector.empty()) {
            get_result.push_back(inner_vector);
        }
    }


    for (auto &ob: get_result) {
		for (auto &i: ob) {
			std::cout << i << " ";
		}
		std::cout << std::endl;
	}

    return get_result;

}


