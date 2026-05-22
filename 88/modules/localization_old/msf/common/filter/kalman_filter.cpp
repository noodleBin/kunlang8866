/******************************************************************************
 * Copyright 2023 The Move-X Authors. All Rights Reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *****************************************************************************/
#include "kalman_filter.h"
#include <string>
#include <vector>
#include <Eigen/SVD>  // SVD for observability analysis:
#include "glog/logging.h"
#include "CSVWriter.h"

namespace century {
namespace loc {
void KalmanFilter::AnalyzeQ(const int &DIM_STATE, const double &time,
                            const Eigen::MatrixXd &Q, const Eigen::VectorXd &Y,
                            std::vector<std::vector<double>> &data) {
  // perform SVD analysis, thin for rectangular matrix:
  Eigen::JacobiSVD<Eigen::MatrixXd> svd(
      Q, Eigen::ComputeThinU | Eigen::ComputeThinV);

  // add record:
  std::vector<double> record;

  // a. record timestamp:
  record.emplace_back(time);

  // b. record singular values:
  for (int i = 0; i < DIM_STATE; ++i) {
    record.emplace_back(svd.singularValues()(i, 0));
  }

  // c. record degree of observability:
  Eigen::MatrixXd X =
      (svd.matrixV() * svd.singularValues().asDiagonal().inverse()) *
      (svd.matrixU().transpose() * Y).asDiagonal();

  Eigen::VectorXd degree_of_observability = Eigen::VectorXd::Zero(DIM_STATE);
  for (int i = 0; i < DIM_STATE; ++i) {
    // find max. magnitude response in X:
    Eigen::MatrixXd::Index sv_index;
    X.col(i).cwiseAbs().maxCoeff(&sv_index);

    // associate with corresponding singular value:
    degree_of_observability(sv_index) = svd.singularValues()(i);
  }
  // normalize:
  degree_of_observability =
      1.0 / svd.singularValues().maxCoeff() * degree_of_observability;

  for (int i = 0; i < DIM_STATE; ++i) {
    record.emplace_back(degree_of_observability(i));
  }

  // add to data:
  data.emplace_back(record);
}

void KalmanFilter::WriteAsCSV(const int &DIM_STATE,
                              const std::vector<std::vector<double>> &data,
                              const std::string &filename) {
  // init:
  CSVWriter csv(",");
  csv.enableAutoNewRow(1 + 2 * DIM_STATE);

  // a. write header:
  csv << "T";
  for (int i = 0; i < DIM_STATE; ++i) {
    csv << ("sv" + std::to_string(i + 1));
  }
  for (int i = 0; i < DIM_STATE; ++i) {
    csv << ("doo" + std::to_string(i + 1));
  }

  // b. write contents:
  for (const auto &record : data) {
    // cast timestamp to int:
    csv << static_cast<int>(record.at(0));

    for (size_t i = 1; i < record.size(); ++i) {
      csv << std::fabs(record.at(i));
    }
  }

  // save to persistent storage:
  csv.writeToFile(filename);
}
}  // namespace loc
}  // namespace century
