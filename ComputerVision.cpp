/* 
 * File:   ComputerVision.cpp
 * Author: alex
 * 
 * Created on June 17, 2014, 10:44 AM
 */

#include "ComputerVision.h"
#include "BalloonLocation.h"

#include "opencv2/gpu/gpu.hpp"
#include <sys/time.h>
#include <cstdio>
#include <cmath>
#include <cstdarg>
#include <pthread.h>

using namespace cv;

#define FEED_SIZE 2
#define PER_FRAME_TIME_LOGGING 0
#define SHOW_FEED_WINDOW 0
#define SHOW_OTHER_WINDOWS 0
#define SHOW_OUTPUT_WINDOW 1
#define DRAW_DEBUG_DATA 1

#if (FEED_SIZE == 1)

const int FEED_WIDTH = 320;
const int FEED_HEIGHT = 240;

#endif

#if (FEED_SIZE == 2)

const int FEED_WIDTH = 640;
const int FEED_HEIGHT = 480;

#endif

#if (FEED_SIZE == 3)

const int FEED_WIDTH = 1280;
const int FEED_HEIGHT = 960;

#endif

#if (FEED_SIZE == 4)

const int FEED_WIDTH = 1920;
const int FEED_HEIGHT = 1080;

#endif

constexpr double BALLOON_RADIUS = 0.5;
constexpr double PIXEL_ANGLE = 3.1415926/2 / FEED_WIDTH;

using namespace cv;


ComputerVision::ComputerVision() {
  avgCaptureTime = 0;
  avgConversionTime = 0;
  avgSplitTime = 0;
  avgProcessingTime = 0;
  avgDisplayTime = 0;

  captureTime = 0;
  conversionTime = 0;
  splitTime = 0;
  processingTime = 0;
  displayTime = 0;

  nFrames = 0;

}

ComputerVision::ComputerVision(const ComputerVision& orig) {
}

ComputerVision::~ComputerVision() {
}



void ComputerVision::InitGUI() {
#if (SHOW_FEED_WINDOW == 1)
  namedWindow("feed");
#endif
#if (SHOW_OTHER_WINDOWS == 1)
  namedWindow("hue");
  namedWindow("sat");
  namedWindow("val");
  namedWindow("balloonyness");
#endif
#if (SHOW_OUTPUT_WINDOW == 1)
  namedWindow("debugOverlay");
#endif
}

void ComputerVision::RecordTime(long delta, double *avgTime) {
  *avgTime = (*avgTime * nFrames + delta) / (nFrames + 1);
}

long ComputerVision::GetTimeDelta(struct timeval timea, struct timeval timeb) {
  return 1000000 * (timeb.tv_sec - timea.tv_sec) +
    (int(timeb.tv_usec) - int(timea.tv_usec));
}

void ComputerVision::Log(const char* msg, ...) {
#if (PER_FRAME_TIME_LOGGING == 1)
  va_list args;
  va_start(args, msg);
  printf(msg, args);
#endif
}

void ComputerVision::CaptureFrame(VideoCapture &camera, Mat &frame_host, gpu::GpuMat &frame, Mat &debugOverlay) {
  struct timeval timea, timeb;

  gettimeofday(&timea, NULL);
  camera >> frame_host;
  debugOverlay = frame_host.clone();
  frame.upload(frame_host);
  gettimeofday(&timeb, NULL);

  captureTime = GetTimeDelta(timea, timeb);
  Log("capture frame time used:\t%ld\n", captureTime);
}

void ComputerVision::ConvertToHSV(gpu::GpuMat &frame, gpu::GpuMat &hue, gpu::GpuMat &sat, gpu::GpuMat &val) {
  struct timeval timea, timeb;
  gpu::GpuMat hsv;

  vector<gpu::GpuMat> hsvplanes(3);
  hsvplanes[0] = hue;
  hsvplanes[1] = sat;
  hsvplanes[2] = val;

  gettimeofday(&timea, NULL);
  gpu::cvtColor(frame, hsv, CV_BGR2HSV);
  gettimeofday(&timeb, NULL);

  conversionTime = GetTimeDelta(timea, timeb);
  Log("color conversion time used:\t%ld\n", conversionTime);

  gettimeofday(&timea, NULL);
  gpu::split(hsv, hsvplanes);
  hue = hsvplanes[0];
  sat = hsvplanes[1];
  val = hsvplanes[2];
  gettimeofday(&timeb, NULL);

  splitTime = GetTimeDelta(timea, timeb);
  Log("split planes time used:   \t%ld\n", splitTime);
}

void ComputerVision::ProcessFrame(gpu::GpuMat &hue, gpu::GpuMat &sat, gpu::GpuMat &balloonyness, Mat &debugOverlay) {
  struct timeval timea, timeb;
  gpu::GpuMat huered, scalehuered, scalesat, thresh;
  Mat thresh_host;
  vector< vector< Point > > contours;

  gettimeofday(&timea, NULL);

  gpu::absdiff(hue, Scalar(90), huered);
  gpu::divide(huered, Scalar(4), scalehuered);
  gpu::divide(sat, Scalar(16), scalesat);
  gpu::multiply(scalehuered, scalesat, balloonyness);
  gpu::threshold(balloonyness, thresh, 200, 255, THRESH_BINARY);
  thresh.download(thresh_host);

  findContours(thresh_host, contours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_NONE);

#if (DRAW_DEBUG_DATA == 1)
  drawContours(debugOverlay, contours, -1, Scalar(255, 0, 0));
#endif

  vector< Point2f > circleCenters(contours.size());
  vector< float > circleRadii(contours.size());
  Point2f center;
  float radius;
  tempLocation = {-1, 0, 0, timea};
  for (int n = 0; n < contours.size(); ++n) {
    minEnclosingCircle(contours[n], center, radius);

#if (DRAW_DEBUG_DATA == 1)
    circle(debugOverlay, center, radius, Scalar(0, 255, 255));
#endif

    if (contourArea(contours[n]) >= areaRatio * radius*radius*3.1415926) {
      circle(debugOverlay, center, radius, Scalar(0, 255, 0), 2);
      double range = BALLOON_RADIUS / sin(radius*PIXEL_ANGLE);
      if (range < tempLocation.range || tempLocation.range <= 0) {
	tempLocation.range = range;
	printf("center.y: %f\n", center.y);
	tempLocation.phi = (-center.y + FEED_HEIGHT/2) * PIXEL_ANGLE;
	tempLocation.theta = (center.x - FEED_WIDTH/2) * PIXEL_ANGLE;
	gettimeofday(&tempLocation.timestamp, NULL);
      }
    }
  }

  gettimeofday(&timeb, NULL);
  processingTime = GetTimeDelta(timea, timeb);
  Log("frame processing time used:\t%ld\n", processingTime);
}

void ComputerVision::DisplayOutput(Mat frame, gpu::GpuMat hue, gpu::GpuMat sat, gpu::GpuMat val, gpu::GpuMat balloonyness, Mat debugOverlay) {
  struct timeval timea, timeb;

  gettimeofday(&timea, NULL);

#if (SHOW_FEED_WINDOW == 1)
  imshow("feed", frame);
#endif
#if (SHOW_OTHER_WINDOWS ==1)
  Mat hue_host, sat_host, val_host, balloonyness_host;
  hue.download(hue_host);
  sat.download(sat_host);
  val.download(val_host);
  balloonyness.download(balloonyness_host);
  imshow("hue", hue_host);
  imshow("sat", sat_host);
  imshow("val", val_host);
  imshow("balloonyness", balloonyness_host);
#endif
#if (SHOW_OUTPUT_WINDOW == 1)
  imshow("debugOverlay", debugOverlay);
#endif

  gettimeofday(&timeb, NULL);
  displayTime = GetTimeDelta(timea, timeb);
  Log("display frame time used:\t%ld\n", displayTime);
}

void ComputerVision::CvMain() {
  struct timeval timea, timeb, startTime, endTime;
  gettimeofday(&startTime, NULL);

  Mat frame_host, thresh_host, debugOverlay;
  gpu::GpuMat frame, hsv, hue, sat, val, huered, scalehuered, scalesat, balloonyness, thresh;


  VideoCapture camera(0);
  camera.set(CV_CAP_PROP_FRAME_WIDTH, FEED_WIDTH);
  camera.set(CV_CAP_PROP_FRAME_HEIGHT, FEED_HEIGHT);

  Log("optimized code: %d\n", useOptimized());
  Log("cuda devices: %d\n", gpu::getCudaEnabledDeviceCount());
  Log("current device: %d\n", gpu::getDevice());

  InitGUI();
  Log("starting balloon recognition\n");

  while(true) {
    CaptureFrame(camera, frame_host, frame, debugOverlay);
    ConvertToHSV(frame, hue, sat, val);
    ProcessFrame(hue, sat, balloonyness, debugOverlay);
    DisplayOutput(frame_host, hue, sat, val, balloonyness, debugOverlay);
//    printf("CV locking mutex\n");
    pthread_mutex_lock(&locationLock);
//    printf("CV locked mutex\n");
    location = tempLocation;
    pthread_mutex_unlock(&locationLock);
//    printf("CV unlocked mutex\n");

    RecordTime(captureTime, &avgCaptureTime);
    RecordTime(conversionTime, &avgConversionTime);
    RecordTime(splitTime, &avgSplitTime);
    RecordTime(processingTime, &avgProcessingTime);
    RecordTime(displayTime, &avgDisplayTime);

    ++nFrames;

    if (waitKey(30) >= 0) {
      break;
    }
  }

  gettimeofday(&endTime, NULL);
  long totalTimeUsec = GetTimeDelta(startTime, endTime);
  double totalTimeSec = double(totalTimeUsec)/1000000.0;

  printf("key press detected. printing statistics.\n");
  printf("%d frames captured over %ld microseconds (%lf seconds)\n", nFrames,
	 totalTimeUsec, totalTimeSec);
  printf("ran at %lf Frames per Second\n", nFrames/totalTimeSec);
  printf("average capture frame time used:\t%lf\n", avgCaptureTime);
  printf("average color conversion time used:\t%lf\n", avgConversionTime);
  printf("average split planes time used:  \t%lf\n", avgSplitTime);
  printf("average frame processing time used:\t%lf\n", avgProcessingTime);
  printf("average display frame time used:\t%lf\n", avgDisplayTime);
  printf("terminating...\n");

  pthread_exit(NULL);
}

void *ComputerVision::RunCV(void *tid) {
  ComputerVision vision{};
  vision.CvMain();
}
