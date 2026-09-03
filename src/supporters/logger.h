/**
 * @file logger.h
 * @brief Data Logger
 * @author Ryotaro Onuki <kerikun11+github@gmail.com>
 * @copyright Copyright 2021 Ryotaro Onuki
 */
#pragma once

#include <cstdio>
#include <string>
#include <vector>
#include "app_log.h"

class Logger {
 public:
  Logger() {}
  void clear() { buf.clear(); }
  void init(const std::vector<std::string>& labels) {
    clear();
    this->labels = labels;
  }
  void push(const std::vector<float>& data) {
    if (buf.size() < 2000) { // Limit memory on STM32
      buf.push_back(data);
    }
  }
  void print() const {
    BTSerial.println("# LOG BEGIN");
    for (size_t i = 0; i < labels.size(); i++) {
      BTSerial.print(labels[i].c_str());
      if (i + 1 < labels.size()) BTSerial.print(",");
    }
    BTSerial.println();
    for (const auto& row : buf) {
      for (size_t i = 0; i < row.size(); i++) {
        BTSerial.print(row[i], 3);
        if (i + 1 < row.size()) BTSerial.print(",");
      }
      BTSerial.println();
    }
    BTSerial.println("# LOG END");
  }

 private:
  std::vector<std::string> labels;
  std::vector<std::vector<float>> buf;
};
