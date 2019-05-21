#include <iostream>
#include <vector>
#include <opencv2/core/core.hpp>

#include "constants.hpp"

using namespace std;
using namespace cv;
using namespace cv::cuda;

namespace LGR {
  struct HSVvalues {
    int HMin = 0;
    int HMax = 255;
    int SMin = 0;
    int SMax = 255;
    int VMin = 0;
    int VMax = 255;
    
    Mat HSVMin , HSVMax;
    GpuMat HSVMinG, HSVMaxG;
        
    void calcValues();
    
    void print();
    void reset();
  };
  
  void HSVvalues::calcValues() {
    HSVMin.create(FRAME_HEIGHT, FRAME_WIDTH, CV_8UC4);
    HSVMax.create(FRAME_HEIGHT, FRAME_WIDTH, CV_8UC4);
    
    Mat c1(FRAME_HEIGHT, FRAME_WIDTH, CV_8U);
    
    vector<Mat> minChannels(4, c1);
    vector<Mat> maxChannels(4, c1);
    
    minChannels[0] = HMin;
    minChannels[1] = SMin;
    minChannels[2] = VMin;
    minChannels[3] = 0;
    
    maxChannels[0] = HMax;
    maxChannels[1] = SMax;
    maxChannels[2] = VMax;
    maxChannels[3] = 0;
    
    merge(minChannels, HSVMin);
    merge(maxChannels, HSVMax);
    
    HSVMinG.upload(HSVMin);
    HSVMaxG.upload(HSVMax);
  }
  
  void HSVvalues::reset() {
    HMin = 0;
    HMax = 255;
    SMin = 0;
    SMax = 255;
    VMin = 0;
    VMax = 255;
    
    calcValues();
  }
  
  void HSVvalues::print() {
    cout << "HMin: " << HMin << ", HMax: " << HMax
    << "SMin: " << SMin << ", SMax: " << SMax
    << "VMin: " << VMin << ", VMax: " << VMax << endl;
  }
}
