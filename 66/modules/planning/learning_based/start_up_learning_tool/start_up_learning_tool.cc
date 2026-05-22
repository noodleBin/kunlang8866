/******************************************************************************
 * Copyright 2022 The Century Authors. All Rights Reserved.
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

/**
 * @file
 **/
#include <iostream>

#include <opencv2/ml/ml.hpp>
#include <opencv2/opencv.hpp>

#include "modules/planning/proto/svm_feature_data.pb.h"

#include "cyber/common/file.h"

int main() {
  // create a classifier and set parameters.
  const std::string model_name =
      "/century/modules/planning/learning_based/start_up_learning_tool/"
      "start_up.xml";
  const std::string features_file =
      "/century/modules/planning/learning_based/result/result.bin";
  cv::Ptr<cv::ml::SVM> model = cv::ml::SVM::create();
  model->setType(cv::ml::SVM::C_SVC);
  model->setKernel(cv::ml::SVM::RBF);  // kernel function
  model->setTermCriteria(
      cv::TermCriteria(cv::TermCriteria::MAX_ITER, 1000, FLT_EPSILON));

  // set training data matrix
  century::planning::SvmFeatureData features;
  century::cyber::common::GetProtoFromBinaryFile(features_file, &features);
  std::cout << "features.svm_feature_size(): " << features.featrue().size()
            << std::endl;
  int trainingData[features.featrue().size()][20];
  int* labels = new int[features.featrue().size()];
  for (int i = 0; i < features.featrue().size(); ++i) {
    trainingData[i][0] = features.featrue(i).one_level_unknown_obs();
    trainingData[i][1] = features.featrue(i).one_level_typed_obs();
    trainingData[i][2] = features.featrue(i).two_level_unknown_obs();
    trainingData[i][3] = features.featrue(i).two_level_typed_obs();
    trainingData[i][4] = features.featrue(i).three_level_unknown_obs();
    trainingData[i][5] = features.featrue(i).three_level_typed_obs();
    trainingData[i][6] = features.featrue(i).four_level_unknown_obs();
    trainingData[i][7] = features.featrue(i).four_level_typed_obs();
    trainingData[i][8] = features.featrue(i).five_level_unknown_obs();
    trainingData[i][9] = features.featrue(i).five_level_typed_obs();
    trainingData[i][10] = features.featrue(i).dynamic_one_level_unknown_obs();
    trainingData[i][11] = features.featrue(i).dynamic_one_level_typed_obs();
    trainingData[i][12] = features.featrue(i).dynamic_two_level_unknown_obs();
    trainingData[i][13] = features.featrue(i).dynamic_two_level_typed_obs();
    trainingData[i][14] = features.featrue(i).dynamic_three_level_unknown_obs();
    trainingData[i][15] = features.featrue(i).dynamic_three_level_typed_obs();
    trainingData[i][16] = features.featrue(i).dynamic_four_level_unknown_obs();
    trainingData[i][17] = features.featrue(i).dynamic_four_level_typed_obs();
    trainingData[i][18] = features.featrue(i).dynamic_five_level_unknown_obs();
    trainingData[i][19] = features.featrue(i).dynamic_five_level_typed_obs();

    labels[i] = features.featrue(i).label();
    std::cout
        << "trainingData[" << i << "][0] = " << trainingData[i][0]
        << "         trainingData[" << i << "][1] = " << trainingData[i][1]
        << "         trainingData[" << i << "][2] = " << trainingData[i][2]
        << "         trainingData[" << i << "][3] = " << trainingData[i][3]
        << "         trainingData[" << i << "][4] = " << trainingData[i][4]
        << "         trainingData[" << i << "][5] = " << trainingData[i][5]
        << "         trainingData[" << i << "][6] = " << trainingData[i][6]
        << "         trainingData[" << i << "][7] = " << trainingData[i][7]
        << "         trainingData[" << i << "][8] = " << trainingData[i][8]
        << "         trainingData[" << i << "][9] = " << trainingData[i][9]
        << "         trainingData[" << i << "][10] = " << trainingData[i][10]
        << "         trainingData[" << i << "][11] = " << trainingData[i][11]
        << "         trainingData[" << i << "][12] = " << trainingData[i][12]
        << "         trainingData[" << i << "][13] = " << trainingData[i][13]
        << "         trainingData[" << i << "][14] = " << trainingData[i][14]
        << "         trainingData[" << i << "][15] = " << trainingData[i][15]
        << "         trainingData[" << i << "][16] = " << trainingData[i][16]
        << "         trainingData[" << i << "][17] = " << trainingData[i][17]
        << "         trainingData[" << i << "][18] = " << trainingData[i][18]
        << "         trainingData[" << i << "][19] = " << trainingData[i][19]
        << "         labels[" << i << "]=" << labels[i];
  }

  cv::Mat trainingDataMat(features.featrue().size(), 20, CV_32SC1,
                          trainingData);
  std::cout << "type = " << trainingDataMat.type() << std::endl;
  trainingDataMat.convertTo(trainingDataMat, CV_32FC1);
  std::cout << "trainingDataMat.rows: " << trainingDataMat.rows << std::endl;
  std::cout << "trainingDataMat.cols: " << trainingDataMat.cols << std::endl;
  std::cout << "type = " << trainingDataMat.type() << std::endl;

  cv::Mat labelsMat(features.featrue().size(), 1, CV_32SC1, labels);

  // Create a TrainData instance
  cv::Ptr<cv::ml::TrainData> tData =
      cv::ml::TrainData::create(trainingDataMat, cv::ml::ROW_SAMPLE, labelsMat);
  // Each training data is a row of the training data matrix, so it is
  // ROW_SAMPLE
  std::cout << "gamma value1: " << std::fixed << std::setprecision(9)
            << model->getGamma() << std::endl;
  std::cout << "C value1: " << std::fixed << std::setprecision(9)
            << model->getC() << std::endl;

  // training classifiers
  // model->train(tData);
  model->trainAuto(tData);
  model->save(model_name);
  std::cout << "gamma value: " << std::fixed << std::setprecision(9)
            << model->getGamma() << std::endl;
  std::cout << "C value: " << std::fixed << std::setprecision(9)
            << model->getC() << std::endl;
  delete[] labels;
  return 0;
}
