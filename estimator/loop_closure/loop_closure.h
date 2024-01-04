
#ifndef __LOOP_CLOSURE__
#define __LOOP_CLOSURE__

#include <iostream>
#include <vector>
#include <string>

#include "DBoW2.h"
#include "DVision.h"
#include "DLoopDetector.h"
#include "demoDetector.h"

#include <opencv2/highgui.hpp>

using namespace DLoopDetector;
using namespace DBoW2;
using namespace DVision;
using namespace std;

class LoopClosure
{
public:
	LoopClosure(const char *voc_file, int _image_w, int _image_h);

    bool startLoopClosure(std::vector<cv::KeyPoint> &keys, std::vector<BRIEF::bitset> &descriptors,
                          std::vector<cv::Point2f> &cur_pts,
                          std::vector<cv::Point2f> &old_pts,
                          int &old_index);
    void eraseIndex(std::vector<int> &erase_index);
	
	demoDetector<BriefVocabulary, BriefLoopDetector, FBrief::TDescriptor> demo;
	int IMAGE_W;
	int IMAGE_H;
};
#endif
