#ifndef EXTENDEDDATASAVER_HPP
#define EXTENDEDDATASAVER_HPP

#include "BaseDataSaver.h"
#include <Eigen/Dense>

class ExtendedDataSaver : public BaseDataSaver {
public:
    // Constructor
    explicit ExtendedDataSaver(const std::string &fileName);

    // Write a row of Eigen::VectorXd to the file
    void writeRow(const Eigen::VectorXd &rowData);

    // Write an Eigen::MatrixXd to the file
    void writeMatrix(const Eigen::MatrixXd &matrix);
};

#endif // EXTENDEDDATASAVER_HPP
