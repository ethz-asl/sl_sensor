#include "sl_sensor_slstudio/codec2p1tpu.h"
#include <math.h>

#include "sl_sensor_slstudio/cvtools.h"
#include "sl_sensor_slstudio/pstools.h"

#include <opencv2/imgproc/imgproc_c.h>

//#include <opencv2/video/tracking.hpp>

#ifndef M_PI
#define M_PI 3.14159265359
#endif

static unsigned int nPhases = 12;
// static unsigned int nPhases = 16;

namespace sl_sensor {
namespace slstudio {
// Encoder
EncoderPhaseShift2p1Tpu::EncoderPhaseShift2p1Tpu(unsigned int _screenCols, unsigned int _screenRows,
                                                 CodecDir _dir)
    : Encoder(_screenCols, _screenRows, _dir) {
  // Set N
  N = 5;

  const float pi = M_PI;

  if (_dir == CodecDirHorizontal) {
    // Precompute horizontally encoding patterns
    for (unsigned int i = 0; i < 2; i++) {
      float phase = pi / 2.0 * i;
      float pitch = (float)screenCols / (float)nPhases;
      cv::Mat patternI;
      patternI = pstools::computePhaseVector(screenCols, phase, pitch);
      patternI = patternI.t();
      patterns.push_back(patternI);
    }

    // Precompute horizontally phase cue patterns
    for (unsigned int i = 0; i < 2; i++) {
      float phase = pi / 2.0 * i;
      float pitch = screenCols;
      cv::Mat patternI;
      patternI = pstools::computePhaseVector(screenCols, phase, pitch);
      patternI = patternI.t();
      patterns.push_back(patternI);
    }
  }

  if (_dir == CodecDirVertical) {
    // Precompute vertically encoding patterns
    for (unsigned int i = 0; i < 2; i++) {
      float phase = pi / 2.0 * i;
      float pitch = (float)screenRows / (float)nPhases;
      cv::Mat patternI;
      patternI = pstools::computePhaseVector(screenRows, phase, pitch);
      patterns.push_back(patternI);
    }

    // Precompute vertically phase cue patterns
    for (unsigned int i = 0; i < 2; i++) {
      float phase = pi / 2.0 * i;
      float pitch = screenRows;
      cv::Mat patternI;
      patternI = pstools::computePhaseVector(screenRows, phase, pitch);
      patterns.push_back(patternI);
    }
  }

  // Insert flat image as second pattern
  cv::Mat patternI;
  patternI.create(5, 5, CV_8UC3);
  patternI.setTo(0.6 * 255.0);
  auto it = patterns.begin();
  patterns.insert(it + 1, patternI);
}

cv::Mat EncoderPhaseShift2p1Tpu::getEncodingPattern(size_t depth) { return patterns[depth]; }

// Decoder
DecoderPhaseShift2p1Tpu::DecoderPhaseShift2p1Tpu(unsigned int _screenCols, unsigned int _screenRows,
                                                 CodecDir _dir)
    : Decoder(_screenCols, _screenRows, _dir) {
  N = 5;
  frames.resize(N);
}

void DecoderPhaseShift2p1Tpu::setFrame(unsigned int depth, cv::Mat frame) { frames[depth] = frame; }

void DecoderPhaseShift2p1Tpu::decodeFrames(cv::Mat &up, cv::Mat &vp, cv::Mat &mask,
                                           cv::Mat &shading) {
  const float pi = M_PI;

  cv::Mat_<float> I1(frames[0]);
  cv::Mat_<float> I2(frames[1]);
  cv::Mat_<float> I3(frames[2]);
  cv::Mat_<float> I4(frames[3]);
  cv::Mat_<float> I5(frames[4]);

  // cv::Mat mag;
  // cv::magnitude(I1 - I2, I3 - I2, mag);

  if (dir & CodecDirVertical) {
    // Vertical decoding
    cv::phase(I1 - I2, I3 - I2, vp);
    cv::Mat vpCue;
    cv::phase(I4 - I2, I5 - I2, vpCue);
    vp = pstools::unwrapWithCue(vp, vpCue, nPhases);
    vp *= screenRows / (2 * pi);
  }

  if (dir & CodecDirHorizontal) {
    // Horizontal decoding
    cv::phase(I1 - I2, I3 - I2, up);
    cv::Mat upCue;
    cv::phase(I4 - I2, I5 - I2, upCue);
    up = pstools::unwrapWithCue(up, upCue, nPhases);
    up *= screenCols / (2 * pi);
  }

  //    cvtools::writeMat(shading, "shading.mat");

  // Create mask from modulation image and erode
  shading = 2.0 * frames[1];
  mask.create(shading.size(), cv::DataType<bool>::type);
  mask.setTo(true);
  mask = (shading > 80) & (shading < 254);

  /**
  // draw vector on shading
  cv::Size frameSize = lastShading->size();
  cv::Point2f center(frameSize.width / 2, frameSize.height / 2);
  // cv::line(shading, center, center+30*shift, cv::Scalar(255), 5);

  // mask outlier-prone regions
  cv::Mat dx, dy;
  cv::Sobel(I3, dx, -1, 1, 0, 3);
  cv::Sobel(I3, dy, -1, 0, 1, 3);
  cv::Mat edgesShading = abs(dx) + abs(dy);

  cv::Sobel(up, dx, -1, 1, 0, 3);
  cv::Sobel(up, dy, -1, 0, 1, 3);
  cv::Mat edgesUp = abs(dx) + abs(dy);

  cv::Sobel(up, dx, -1, 1, 0, 3);
  cv::Sobel(up, dy, -1, 0, 1, 3);

  mask = mask & (edgesShading < 100) & (edgesUp < 130);
  **/
}

DecoderPhaseShift2p1Tpu::~DecoderPhaseShift2p1Tpu() {
  /**
  cv::FileStorage fs("shiftHistory.xml", cv::FileStorage::WRITE);
  if (!fs.isOpened())
    std::cerr << "Could not write shiftHistory.xml" << std::endl;

  fs << "shiftHistory" << cv::Mat(shiftHistory);
  fs.release();
  **/
}

void DecoderPhaseShift2p1Tpu::decode_keypoints(const std::vector<cv::Mat> &image_sequence,
                                               std::vector<cv::KeyPoint> &keypoint_vec,
                                               std::vector<float> &phase_vector) {
  phase_vector.clear();

  for (const auto &kp : keypoint_vec) {
    for (int i = 0; i < 5; i++) {
      cv::Mat single_pixel = (cv::Mat_<float>(1, 1) << image_sequence[i].at<float>(kp.pt) * 255.0f);
      // std::cout << "Pix " << i << ": " << single_pixel.at<float>(0, 0) << std::endl;
      this->setFrame(i, single_pixel);
    }

    // We currently only support vertical decoding
    cv::Mat up;
    cv::Mat vp(1, 1, CV_32FC1);
    cv::Mat mask(1, 1, cv::DataType<bool>::type);
    cv::Mat shading(1, 1, CV_8U);

    this->decodeFrames(up, vp, mask, shading);

    float phase = NAN;

    if (mask.at<bool>(0, 0) == true) {
      phase = vp.at<float>(0, 0);
    }

    // std::cout << "Phase " << phase << std::endl;

    phase_vector.push_back(phase);
  }
}

}  // namespace slstudio
}  // namespace sl_sensor