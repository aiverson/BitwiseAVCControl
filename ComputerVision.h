/* 
 * File:   ComputerVision.h
 * Author: alex
 *
 * Created on June 17, 2014, 10:44 AM
 */

#ifndef COMPUTERVISION_H
#define	COMPUTERVISION_H

#include "opencv2/opencv.hpp"
#include "BalloonLocation.h"

class ComputerVision {
 public:
  ComputerVision();
  ComputerVision(const ComputerVision& orig);
  virtual ~ComputerVision();

  static void *RunCV(void *tid);

 private:
  double avgCaptureTime;
  double avgConversionTime,
    avgSplitTime,
    avgProcessingTime,
    avgDisplayTime;

long captureTime,
  conversionTime,
  splitTime,
  processingTime,
  displayTime;

int nFrames;

constexpr static double areaRatio = 0.65;

void InitGUI();
void RecordTime(long delta, double *avgTime);
long GetTimeDelta(struct timeval timea, struct timeval timeb);
void Log(const char* msg, ...);
void CaptureFrame(cv::VideoCapture &camera, cv::Mat &frame_host, cv::gpu::GpuMat &frame, cv::Mat &debugOverlay);
void ConvertToHSV(cv::gpu::GpuMat &frame, cv::gpu::GpuMat &hue, cv::gpu::GpuMat &sat, cv::gpu::GpuMat &val);
void ProcessFrame(cv::gpu::GpuMat &hue, cv::gpu::GpuMat &sat, cv::gpu::GpuMat &balloonyness, cv::Mat &debugOverlay);
void DisplayOutput(cv::Mat frame, cv::gpu::GpuMat hue, cv::gpu::GpuMat sat, cv::gpu::GpuMat val, cv::gpu::GpuMat balloonyness, cv::Mat debugOverlay);
void CvMain();

};

#endif	/* COMPUTERVISION_H */

