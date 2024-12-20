#ifndef BASEDATASAVER_HPP
#define BASEDATASAVER_HPP

#include <fstream>
#include <vector>
#include <string>
#include <iostream>

class BaseDataSaver {
protected:
    std::string fileName;
    std::ofstream file;
    bool isFileOpen;

public:
    // Constructor
    explicit BaseDataSaver(const std::string &fileName);

    // Destructor
    virtual ~BaseDataSaver();

    // Write a row of std::vector<double> to the file
    virtual void writeRow(const std::vector<double> &rowData);

    // Close the file
    virtual void closeFile();
};

#endif // BASEDATASAVER_HPP
