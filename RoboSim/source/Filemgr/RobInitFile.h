
#ifndef ROBINITFILE_H
#define ROBINITFILE_H

#include <boost/archive/binary_oarchive.hpp>
#include <boost/archive/binary_iarchive.hpp>
#include <boost/serialization/map.hpp>
#include <boost/serialization/string.hpp>
#include <boost/serialization/access.hpp>
#include "pch.h"
#include "ui/statuslogs.h"
#include <locale>


struct INIT_SETTINGS {
    std::map<std::string, std::string> strr;
    std::map<std::string, int> intt;
	std::map<std::string, float> floatt;
	std::map<std::string, bool> booll;
private:
    friend class boost::serialization::access;
    template<class Archive>
    void serialize(Archive& ar, const unsigned int version) {
        ar& strr;
        ar& intt;
        ar& floatt;
        ar& booll;
    }

};

class RobInitFile {
public:
    static RobInitFile& getinstance() {
        static RobInitFile instance; return instance;
    } 

    template<typename T>
    void get_settings(const std::string& key, T& value) {
        //todo need to check for all types
        if (initst.strr.size() == 0) { *sttlogs << "Key not found: " + key; return; }

        if constexpr (std::is_same_v<T, std::string>) { // string
            value = initst.strr.find(key)->second;
        }
        else if constexpr (std::is_same_v<T, int>) { // int
            value = initst.intt.find(key)->second;
        }
        else if constexpr (std::is_same_v<T, float>) { // float
            value = initst.floatt.find(key)->second;
        }
        else if constexpr (std::is_same_v<T, bool>) { // bool
            value = initst.booll.find(key)->second ;
        }
        else if constexpr (std::is_same_v<T, char*>) { // char*
            value = initst.strr.find(key)->second.c_str();
        }
    }

    // check theme
    std::string get_settings(const std::string& key) {
        if (initst.intt.size() != 0) {
            if (initst.intt.find(key)->second == 0)
            {return "Dark";}
            else if (initst.intt.find(key)->second == 1)
            {return "Light";}
            else
            { return "DarkGreen"; }
        }
        *sttlogs << "Key not found: " << key;
        return "Light";
    }
    template<typename T>
    void update_settings(const std::string& key, const T& value) 
    { 
        if constexpr(std::is_same_v<T, std::string>) {
            initst.strr[key] = value;
		}
        else if constexpr(std::is_same_v<T, int>) {
            initst.intt[key] = value;
		}
		else if constexpr(std::is_same_v<T, float>) {
            initst.floatt[key] = value;
		}
		else if constexpr(std::is_same_v<T, bool>) {
            initst.booll[key] = value;
		}
        else if constexpr(std::is_same_v<T, char*>) {
            initst.strr[key] = std::string(value);
		}
    }

    void SaveInit_encode() {
        std::ofstream outputFile("robosim_settings.dat",std::ios::binary);
        if (!outputFile.is_open()) {
            *sttlogs << "Failed to open file for writing: robosim_settings.txt";
            return;
        }
        boost::archive::binary_oarchive oa(outputFile);
        oa << initst;
        outputFile.close();

    }

    
private:
    nui::StatusLogs* sttlogs;
    INIT_SETTINGS initst;
    bool readINIT_Flag = false;
    
    void ReadIniFile_decode() {
        *sttlogs << " Welcome ";
        std::ifstream inputFile("robosim_settings.dat",std::ios::binary);
        if (!inputFile.is_open()) {
            *sttlogs << "Failed to open file for reading: robosim_settings.dat";
            return;
        }
        boost::archive::binary_iarchive ia(inputFile);
        ia >> initst;

        *sttlogs << "Configuration loaded from text format.";
    }
    friend class boost::serialization::access;
    template<class Archive>
    void serialize(Archive& ar, const unsigned int version) {
        ar& initst;
    }
    // Singleton
    RobInitFile() {
        sttlogs = &nui::StatusLogs::getInstance();
        if (!readINIT_Flag) { ReadIniFile_decode(); readINIT_Flag = true; }// Only read once here, no need to check again after this call
    }

};
#endif