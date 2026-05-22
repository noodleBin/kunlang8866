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
#pragma once
#include <fstream>
#include <iostream>
#include <sstream>
#include <string>
#include <typeinfo>

class CSVWriter {
 public:
  CSVWriter() {
    this->firstRow_ = true;
    this->seperator_ = ";";
    this->columnNum_ = -1;
    this->valueCount_ = 0;
  }

  explicit CSVWriter(int numberOfColums) {
    this->firstRow_ = true;
    this->seperator_ = ";";
    this->columnNum_ = numberOfColums;
    this->valueCount_ = 0;
  }

  explicit CSVWriter(std::string seperator_) {
    this->firstRow_ = true;
    this->seperator_ = seperator_;
    this->columnNum_ = -1;
    this->valueCount_ = 0;
  }

  CSVWriter(std::string seperator_, int numberOfColums) {
    this->firstRow_ = true;
    this->seperator_ = seperator_;
    this->columnNum_ = numberOfColums;
    this->valueCount_ = 0;
    std::cout << this->seperator_ << std::endl;
  }

  CSVWriter& add(const char* str) { return this->add(std::string(str)); }

  CSVWriter& add(char* str) { return this->add(std::string(str)); }

  CSVWriter& add(std::string str) {
    // if " character was found, escape it
    size_t position = str.find("\"", 0);
    bool foundQuotationMarks = position != std::string::npos;
    while (position != std::string::npos) {
      str.insert(position, "\"");
      position = str.find("\"", position + 2);
    }
    if (foundQuotationMarks) {
      str = "\"" + str + "\"";
    } else if (str.find(this->seperator_) != std::string::npos) {
      // if seperator_ was found and string was not escapted before, surround
      // string with "
      str = "\"" + str + "\"";
    }
    return this->add<std::string>(str);
  }

  template <typename T>
  CSVWriter& add(T str) {
    if (this->columnNum_ > -1) {
      // if autoNewRow is enabled, check if we need a line break
      if (this->valueCount_ == this->columnNum_) {
        this->newRow();
      }
    }
    if (valueCount_ > 0) this->ss_ << this->seperator_;
    this->ss_ << str;
    this->valueCount_++;

    return *this;
  }

  template <typename T>
  CSVWriter& operator<<(const T& t) {
    return this->add(t);
  }

  void operator+=(CSVWriter& csv) {  // NOLINT
    this->ss_ << std::endl << csv;
  }

  std::string toString() { return ss_.str(); }

  friend std::ostream& operator<<(std::ostream& os, CSVWriter& csv) {
    return os << csv.toString();
  }

  CSVWriter& newRow() {
    if (!this->firstRow_ || this->columnNum_ > -1) {
      ss_ << std::endl;
    } else {
      // if the row is the first row, do not insert a new line
      this->firstRow_ = false;
    }
    valueCount_ = 0;
    return *this;
  }

  bool writeToFile(std::string filename) {
    return writeToFile(filename, false);
  }

  bool writeToFile(std::string filename, bool append) {
    std::ofstream file;
    if (append)
      file.open(filename.c_str(), std::ios::out | std::ios::app);
    else
      file.open(filename.c_str(), std::ios::out | std::ios::trunc);
    if (!file.is_open()) return false;
    if (append) file << std::endl;
    file << this->toString();
    file.close();
    return file.good();
  }

  void enableAutoNewRow(int numberOfColumns) {
    this->columnNum_ = numberOfColumns;
  }

  void disableAutoNewRow() { this->columnNum_ = -1; }

 protected:
  bool firstRow_;
  std::string seperator_;
  int columnNum_;
  int valueCount_;
  std::stringstream ss_;
};
