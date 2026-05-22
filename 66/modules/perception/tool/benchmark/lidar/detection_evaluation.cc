/******************************************************************************
 * Copyright 2019 The Century Authors. All Rights Reserved.
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

#include "modules/perception/tool/benchmark/lidar/detection_evaluation.h"

namespace century {
namespace perception {
namespace benchmark {

static constexpr size_t kPaddingLength = 12;

bool DetectionEvaluation::init(const std::string& clouds,
                               const std::string& results,
                               const std::string& groundtruths, bool is_folder,
                               unsigned int loader_thread_num,
                               unsigned int eval_thread_num,
                               unsigned int eval_parrallel_num,
                               const std::string& reserve) {
  std::vector<std::string> strs;
  strs.emplace_back(clouds);
  strs.emplace_back(results);
  strs.emplace_back(groundtruths);

  if (is_folder) {
    _loader.init_loader_with_folder(strs);
  } else {
    _loader.init_loader_with_list(strs);
  }

  unsigned int prefetch_size = std::max(10u, eval_parrallel_num);
  _loader.set(50 /*cache size*/, prefetch_size, loader_thread_num);
  if (_thread_pool == nullptr) {
    _thread_pool.reset(new ctpl::thread_pool(eval_thread_num));
  } else {
    _thread_pool->stop(true);
    _thread_pool->start(eval_thread_num);
  }
  _eval_parrallel_num = eval_parrallel_num;

  _frame_metrics.clear();
  _frame_metrics.reserve(_loader.size());

  _lidar_option.parse_from_string(reserve);
  _lidar_option.set_options();
  _meta_stat.reset();
  _self_stat.reset();

  _initialized = true;

  return true;
}

void DetectionEvaluation::run_evaluation() {
  // sequential loading and accumulate
  while (true) {
    bool has_more_data = true;
    std::vector<std::shared_ptr<FrameStatistics>> frames;
    std::vector<std::future<void>> status;
    unsigned int start_id = static_cast<unsigned int>(_frame_metrics.size());
    std::shared_ptr<FrameStatistics> frame_ptr;
    for (unsigned int i = 0; i < _eval_parrallel_num; ++i) {
      if (!_loader.query_next(frame_ptr)) {
        has_more_data = false;
        break;
      } else {
        frames.emplace_back(frame_ptr);
        _frame_metrics.emplace_back(FrameMetrics());
        std::future<void> future_task = _thread_pool->push([frame_ptr](int id) {
          frame_ptr->find_association();
          frame_ptr->cal_meta_statistics();
        });
        status.emplace_back(std::move(future_task));
      }
    }
    for (unsigned int i = 0; i < frames.size(); ++i) {
      status[i].wait();
      _meta_stat += frames[i]->get_meta_statistics();
      frames[i]->get_meta_statistics().get_2017_detection_precision_and_recall(
          &_frame_metrics[start_id + i].detection_precision_2017,
          &_frame_metrics[start_id + i].detection_recall_2017);
      frames[i]->get_meta_statistics().get_2017_detection_visible_recall(
          &_frame_metrics[start_id + i].detection_visible_recall_2017);
      frames[i]->get_meta_statistics().get_2017_aad(
          &_frame_metrics[start_id + i].aad_2017);
      _frame_metrics[start_id + i].frame_name = frames[i]->get_name();
      _frame_metrics[start_id + i].jaccard_index_percentile =
          frames[i]->jaccard_index_percentile();

      _self_stat.add_objects(frames[i]->get_objects(), start_id + i);
    }

    if (!has_more_data) {
      break;
    }
  }

  // globally evaluation
  _meta_stat.get_2017_detection_precision_and_recall(&_detection_precision_2017,
                                                     &_detection_recall_2017);
  _meta_stat.get_2017_detection_visible_recall(&_detection_visible_recall_2017);
  _meta_stat.get_2016_detection_precision_and_recall(&_detection_precision_2016,
                                                     &_detection_recall_2016);
  _meta_stat.get_2017_detection_ap_aos(&_detection_ap, &_detection_aos,
                                       &_detection_curve_samples);
  _meta_stat.get_2017_detection_ap_per_type(&_detection_ap_per_type,
                                            &_detection_curve_samples_per_type);
  _meta_stat.get_2017_classification_accuracy(&_classification_accuracy_2017);
  _meta_stat.get_2016_classification_accuracy(&_classification_accuracy_2016);
  _meta_stat.get_classification_confusion_matrix(
      &_classification_confusion_matrix_gt_major,
      &_classification_confusion_matrix_det_major,
      &_classification_confusion_matrix_det_major_with_fp);

  _meta_stat.get_2017_aad(&_aad_2017);
  std::sort(_frame_metrics.begin(), _frame_metrics.end());
  // self-evaluation metrics
  _self_stat.get_classification_type_change_rates(
      &_classification_change_rate_per_class, &_classification_change_rate);
}

namespace {

void PadString(std::string* str, size_t num, char c = ' ') {
  if (num > str->size()) {
    str->insert(str->size(), num - str->size(), c);
  }
}

void OutputFrameMetrics(std::ostream& out, const DetectionEvaluation& eval,
                        const std::string& range_string) {
  out << std::fixed << std::setprecision(4);
  out << std::string(80, '=') << std::endl;
  out << "Recall    (" << range_string << ")  ";
  out << "Precision (" << range_string << ")  "
      << "Visible recall(" << range_string << ")  "
      << "Jaccard index percentile "
      << FrameStatistics::get_jaccard_index_percentile() << std::endl;
  for (auto& frame : eval._frame_metrics) {
    out << "          ";
    for (unsigned int i = 0; i < MetaStatistics::get_range_dim(); ++i) {
      out << frame.detection_recall_2017[i] << "     ";
    }
    out << "          ";
    for (unsigned int i = 0; i < MetaStatistics::get_range_dim(); ++i) {
      out << frame.detection_precision_2017[i] << "     ";
    }
    out << "          ";
    for (unsigned int i = 0; i < MetaStatistics::get_range_dim(); ++i) {
      out << frame.detection_visible_recall_2017[i] << "     ";
    }
    out << frame.jaccard_index_percentile << "     ";
    out << std::left << frame.frame_name << std::endl;
  }
  out << eval._meta_stat << std::endl;
}

void OutputDetectionKpi(std::ostream& out, const DetectionEvaluation& eval,
                        const std::string& range_string,
                        const std::string& full_range_string) {
  std::string str_buf;
  const std::string section_split_str(80, '=');

  out << section_split_str << std::endl;
  out << "2017 Detection KPI :  " << full_range_string << std::endl;
  out << "kpi_recall         :  ";
  for (unsigned int i = 0; i <= MetaStatistics::get_range_dim(); ++i) {
    str_buf = std::to_string(eval._detection_recall_2017[i]);
    PadString(&str_buf, kPaddingLength);
    out << str_buf;
  }
  out << std::endl;
  out << "kpi_precision      :  ";
  for (unsigned int i = 0; i <= MetaStatistics::get_range_dim(); ++i) {
    str_buf = std::to_string(eval._detection_precision_2017[i]);
    PadString(&str_buf, kPaddingLength);
    out << str_buf;
  }
  out << std::endl;
  out << "kpi_visible_recall :  ";
  for (unsigned int i = 0; i <= MetaStatistics::get_range_dim(); ++i) {
    str_buf = std::to_string(eval._detection_visible_recall_2017[i]);
    PadString(&str_buf, kPaddingLength);
    out << str_buf;
  }
  out << std::endl << std::endl;

  out << "2016 Detection KPI :  " << range_string << std::endl;
  out << "kpi_recall         :  ";
  for (unsigned int i = 0; i < MetaStatistics::get_range_dim(); ++i) {
    str_buf = std::to_string(eval._detection_recall_2016[i]);
    PadString(&str_buf, kPaddingLength);
    out << str_buf;
  }
  out << std::endl;
  out << "kpi_precision      :  ";
  for (unsigned int i = 0; i < MetaStatistics::get_range_dim(); ++i) {
    str_buf = std::to_string(eval._detection_precision_2016[i]);
    PadString(&str_buf, kPaddingLength);
    out << str_buf;
  }
  out << std::endl;
}

void OutputClassification2017(std::ostream& out,
                              const DetectionEvaluation& eval,
                              const std::string& full_range_string) {
  std::string str_buf;
  const std::string section_split_str(80, '=');

  out << section_split_str << std::endl;
  out << "2017 Classification KPI:  " << full_range_string << std::endl;
  for (unsigned int i = 0; i < eval._classification_accuracy_2017.size(); ++i) {
    out << std::left << std::setw(10) << translate_type_index_to_string(i)
        << " accuracy    :  ";
    for (std::size_t j = 0; j < eval._classification_accuracy_2017[0].size();
         ++j) {
      str_buf = std::to_string(eval._classification_accuracy_2017[i][j]);
      PadString(&str_buf, kPaddingLength);
      out << str_buf;
    }
    out << std::endl;
  }
  out << std::endl;

  out << "Classification confusion matrix (Row: groundtruth, Col: estimated)"
      << std::endl;
  for (unsigned int i = 0;
       i < eval._classification_confusion_matrix_gt_major.size(); ++i) {
    out << std::left << std::setw(10) << translate_type_index_to_string(i)
        << " ";
    for (std::size_t j = 0;
         j < eval._classification_confusion_matrix_gt_major[0].size(); ++j) {
      out << eval._classification_confusion_matrix_gt_major[i][j] << " ";
    }
    out << std::endl;
  }
  out << std::endl;

  out << "Classification confusion matrix (Row: estimated, Col: groundtruth)"
      << std::endl;
  for (unsigned int i = 0;
       i < eval._classification_confusion_matrix_det_major.size(); ++i) {
    out << std::left << std::setw(10) << translate_type_index_to_string(i)
        << " ";
    for (std::size_t j = 0;
         j < eval._classification_confusion_matrix_det_major[0].size(); ++j) {
      out << eval._classification_confusion_matrix_det_major[i][j] << " ";
    }
    out << std::endl;
  }
  out << std::endl;

  out << "Classification confusion matrix with fp (Row: estimated, Col: "
         "groundtruth)"
      << std::endl;
  for (unsigned int i = 0;
       i < eval._classification_confusion_matrix_det_major_with_fp.size();
       ++i) {
    out << std::left << std::setw(10) << translate_type_index_to_string(i)
        << " ";
    for (std::size_t j = 0;
         j < eval._classification_confusion_matrix_det_major_with_fp[0].size();
         ++j) {
      out << eval._classification_confusion_matrix_det_major_with_fp[i][j]
          << " ";
    }
    out << std::endl;
  }
}

void OutputClassification2016(std::ostream& out,
                              const DetectionEvaluation& eval,
                              const std::string& range_string) {
  std::string str_buf;
  const std::string section_split_str(80, '=');

  out << section_split_str << std::endl;
  out << "2016 Classification KPI:  " << range_string << std::endl;
  for (unsigned int i = 0; i < eval._classification_accuracy_2016.size(); ++i) {
    out << std::left << std::setw(10) << translate_type_index_to_string(i)
        << " accuracy    :  ";
    for (std::size_t j = 0; j < eval._classification_accuracy_2016[i].size();
         ++j) {
      str_buf = std::to_string(eval._classification_accuracy_2016[i][j]);
      PadString(&str_buf, kPaddingLength);
      out << str_buf;
    }
    out << std::endl;
  }
  out << section_split_str << std::endl;

  out << "Classification change rate: " << std::setw(10)
      << eval._classification_change_rate << std::endl;
  for (unsigned int i = 0;
       i < eval._classification_change_rate_per_class.size(); ++i) {
    out << std::left << std::setw(10) << translate_type_index_to_string(i)
        << " ";
    for (std::size_t j = 0;
         j < eval._classification_change_rate_per_class[i].size(); ++j) {
      out << eval._classification_change_rate_per_class[i][j] << " ";
    }
    out << std::endl;
  }
}

void OutputApPerType(std::ostream& out, const DetectionEvaluation& eval) {
  const std::string section_split_str(80, '=');
  out << section_split_str << std::endl;
  for (unsigned int i = 0; i < MetaStatistics::get_type_dim(); ++i) {
    out << MetaStatistics::get_type(i) << " ap "
        << eval._detection_ap_per_type[i] << std::endl;
    out << "SPR-Curve samples <precision recall confidence similarity>"
        << std::endl;
    for (auto& tuple : eval._detection_curve_samples_per_type[i]) {
      out << "sprc_sample:         " << tuple.precision << "  " << tuple.recall
          << "  " << tuple.confidence << "  " << tuple.similarity << std::endl;
    }
    out << "---------------------------------------------------" << std::endl;
  }
}

void OutputAadAndSummary(std::ostream& out, const DetectionEvaluation& eval,
                         const std::string& range_string) {
  const std::string section_split_str(80, '=');
  out << section_split_str << std::endl;
  out << "each range AAD (" << range_string << ") "
      << "  all region AAD" << std::endl;
  out << "                  ";
  for (unsigned int i = 0; i < MetaStatistics::get_range_dim(); ++i) {
    out << eval._aad_2017[i] << "     ";
  }
  out << "          " << eval._aad_2017[MetaStatistics::get_range_dim()];
  out << std::endl;

  out << section_split_str << std::endl;
  out << "aos " << eval._detection_aos << std::endl;
  out << "ap " << eval._detection_ap << std::endl << std::endl;
  out << "SPR-Curve samples <precision recall confidence similarity>"
      << std::endl;
  for (auto& tuple : eval._detection_curve_samples) {
    out << "sprc_sample:         " << tuple.precision << "  " << tuple.recall
        << "  " << tuple.confidence << "  " << tuple.similarity << std::endl;
  }
}
}  // namespace

std::ostream& operator<<(std::ostream& out, const DetectionEvaluation& rhs) {
  std::string range_string;
  std::string str_buf;
  for (unsigned int i = 0; i < MetaStatistics::get_range_dim(); ++i) {
    str_buf = MetaStatistics::get_range(i);
    PadString(&str_buf, kPaddingLength);
    range_string += str_buf;
  }
  std::string full_range_string =
      range_string + " " +
      MetaStatistics::get_range(MetaStatistics::get_range_dim());

  OutputFrameMetrics(out, rhs, range_string);
  OutputDetectionKpi(out, rhs, range_string, full_range_string);
  OutputClassification2017(out, rhs, full_range_string);
  OutputClassification2016(out, rhs, range_string);
  OutputApPerType(out, rhs);
  OutputAadAndSummary(out, rhs, range_string);

  return out;
}

}  // namespace benchmark
}  // namespace perception
}  // namespace century
