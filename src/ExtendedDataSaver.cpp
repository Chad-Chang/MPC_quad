#include "ExtendedDataSaver.hpp"

// Constructor
ExtendedDataSaver::ExtendedDataSaver(const std::string &fileName) : BaseDataSaver(fileName) {}

// Write a row of Eigen::VectorXd to the file
void ExtendedDataSaver::writeRow(const Eigen::VectorXd &rowData) {
    if (!isFileOpen) {
        std::cerr << "Error: File is not open for writing." << std::endl;
        return;
    }

    for (int i = 0; i < rowData.size(); ++i) {
        file << rowData[i];
        if (i < rowData.size() - 1) {
            file << ",";
        }
    }
    file << "\n";
    file.flush();
}

// Write an Eigen::MatrixXd to the file
void ExtendedDataSaver::writeMatrix(const Eigen::MatrixXd &matrix) {
    if (!isFileOpen) {
        std::cerr << "Error: File is not open for writing." << std::endl;
        return;
    }

    for (int i = 0; i < matrix.rows(); ++i) {
        for (int j = 0; j < matrix.cols(); ++j) {
            file << matrix(i, j);
            if (j < matrix.cols() - 1) {
                file << ",";
            }
        }
        file << "\n";
        file.flush();
    }
}
