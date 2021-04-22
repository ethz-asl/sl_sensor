#pragma once
#include <stdio.h>
#include <tf/transform_datatypes.h>
#include <Eigen/Core>
#include <algorithm>
#include <cstdint>
#include <fstream>
#include <iostream>
#include <memory>
#include <sstream>
#include <string>

struct TumPose
{
  uint64_t timestamp = 0;
  double x = 0.0f;
  double y = 0.0f;
  double z = 0.0f;
  double qx = 0.0f;
  double qy = 0.0f;
  double qz = 0.0f;
  double qw = 1.0f;

  TumPose(uint64_t _timestamp = 0, double _x = 0.0f, double _y = 0.0f, double _z = 0.0f, double _qx = 0.0f,
          double _qy = 0.0f, double _qz = 0.0f, double _qw = 1.0f)
    : timestamp(_timestamp), x(_x), y(_y), z(_z), qx(_qx), qy(_qy), qz(_qz), qw(_qw)
  {
  }

  TumPose(uint64_t ts, const Eigen::Matrix4f& mat)
  {
    set(ts, mat);
  }

  void GetTransformationMatrix(Eigen::Matrix4f& mat)
  {
    tf::Matrix3x3 R;
    tf::Quaternion q(qx, qy, qz, qw);
    R.setRotation(q);

    mat = Eigen::Matrix4f::Identity();

    auto row_1 = R.getRow(0);
    auto row_2 = R.getRow(1);
    auto row_3 = R.getRow(2);

    mat(0, 0) = row_1.getX();
    mat(0, 1) = row_1.getY();
    mat(0, 2) = row_1.getZ();

    mat(1, 0) = row_2.getX();
    mat(1, 1) = row_2.getY();
    mat(1, 2) = row_2.getZ();

    mat(2, 0) = row_3.getX();
    mat(2, 1) = row_3.getY();
    mat(2, 2) = row_3.getZ();

    mat(0, 3) = x;
    mat(1, 3) = y;
    mat(2, 3) = z;
  }

  void set(uint64_t ts, const Eigen::Matrix4f& mat)
  {
    timestamp = ts;

    tf::Matrix3x3 tf3d;
    tf3d.setValue(mat(0, 0), mat(0, 1), mat(0, 2), mat(1, 0), mat(1, 1), mat(1, 2), mat(2, 0), mat(2, 1), mat(2, 2));

    tf::Quaternion quat;
    tf3d.getRotation(quat);

    x = mat(0, 3);
    y = mat(1, 3);
    z = mat(2, 3);
    qx = quat.getX();
    qy = quat.getY();
    qz = quat.getZ();
    qw = quat.getW();
  }

  void ScaleTransform(double scale)
  {
    x *= scale;
    y *= scale;
    z *= scale;
  }
};

class TumCsvReader
{
public:
  TumCsvReader(const std::string& filename)
  {
    // Create an input filestream

    stream_ptr_ = std::make_unique<std::ifstream>(filename);

    // Make sure the file is open
    if (!stream_ptr_->is_open())
    {
      throw std::runtime_error("Could not open file " + filename);
    }
  }

  bool GetNextRow(TumPose& pose)
  {
    std::string line = "";
    std::string word = "";

    // read an entire row and
    // store it in a string variable 'line'
    // Automatically takes care of line break '\n'
    std::getline(*stream_ptr_, line);

    // used for breaking words
    std::stringstream stream(line);

    // read every column data of a row and
    // store it in a string variable, 'word'
    std::vector<std::string> row_entries = {};
    while (std::getline(stream, word, delimiter_))
    {
      row_entries.push_back(word);
    }

    // Store data in pose
    if (row_entries.size() == 8)
    {
      pose.timestamp = std::stoull(row_entries.at(0));
      pose.x = std::stod(row_entries.at(1));
      pose.y = std::stod(row_entries.at(2));
      pose.z = std::stod(row_entries.at(3));
      pose.qx = std::stod(row_entries.at(4));
      pose.qy = std::stod(row_entries.at(5));
      pose.qz = std::stod(row_entries.at(6));
      pose.qw = std::stod(row_entries.at(7));

      return true;
    }
    else
    {
      return false;
    }
  }

private:
  char delimiter_ = ' ';

  std::unique_ptr<std::ifstream> stream_ptr_;
};

class TumCsvWriter
{
public:
  TumCsvWriter(const std::string& filename)
  {
    // Create an output filestream, create file if it does not exist, clear file if it exists
    stream_.open(filename, std::ofstream::out | std::ofstream::trunc);
  }

  void WriteNextRow(const TumPose& pose)
  {
    stream_ << std::to_string(pose.timestamp) << delimiter_;
    stream_ << std::to_string(pose.x) << delimiter_;
    stream_ << std::to_string(pose.y) << delimiter_;
    stream_ << std::to_string(pose.z) << delimiter_;
    stream_ << std::to_string(pose.qx) << delimiter_;
    stream_ << std::to_string(pose.qy) << delimiter_;
    stream_ << std::to_string(pose.qz) << delimiter_;
    stream_ << std::to_string(pose.qw) << '\n';
  }

private:
  char delimiter_ = ' ';

  std::fstream stream_;
};

class FrameTimingsCsvWriter
{
public:
  FrameTimingsCsvWriter(const std::string& filename)
  {
    // Create an output filestream, create file if it does not exist, clear file if it exists
    stream_.open(filename, std::ofstream::out | std::ofstream::trunc);
  }

  void WriteNextRow(const std::vector<uint64_t>& timestamp_vec)
  {
    int N = timestamp_vec.size();

    for (int i = 0; i < N; i++)
    {
      stream_ << std::to_string(timestamp_vec[i]) << ((i != (N - 1)) ? delimiter_ : '\n');
    }
  }

private:
  char delimiter_ = ' ';

  std::fstream stream_;
};