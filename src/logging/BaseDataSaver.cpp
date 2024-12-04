#include "BaseDataSaver.h"

// Constructor
BaseDataSaver::BaseDataSaver(const std::string &fileName) : fileName(fileName), isFileOpen(false) {
    // file.open(fileName, std::ios::out|std::ios::app);
    file.open(fileName, std::ios::out);
    if (!file.is_open()) {
        std::cerr << "Error: Could not open file " << fileName << std::endl;
    } else {
        isFileOpen = true;
    }
}

// Destructor
BaseDataSaver::~BaseDataSaver() {
    if (isFileOpen) {
        file.close();
    }
}

// Write a row of std::vector<double> to the file
void BaseDataSaver::writeRow(const std::vector<double> &rowData) {
    if (!isFileOpen) {
        std::cerr << "Error: File is not open for writing." << std::endl;
        return;
    }

    for (size_t i = 0; i < rowData.size(); ++i) {
        file << rowData[i];
        if (i < rowData.size() - 1) {
            file << ",";
        }
    }
    file << "\n";
    file.flush();
}

// Close the file
void BaseDataSaver::closeFile() {
    if (isFileOpen) {
        file.close();
        isFileOpen = false;
    }
}
