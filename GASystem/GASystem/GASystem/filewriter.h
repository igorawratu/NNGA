#ifndef FILEWRITER_H
#define FILEWRITER_H

#include <string>
#include <fstream>

using namespace std;

class FileWriter{
public:
    static void writeToFile(string _fileName, string _dat){
        ofstream file;
        file.open(fileName, ios_base::app);
        file << _dat << endl;
        file.close();
    }
};

#endif