#ifndef FERATUREEXTRACTION_LOADIMAGES_H
#define FERATUREEXTRACTION_LOADIMAGES_H

#include <iostream>
#include <fstream>
#include <algorithm>
#include <chrono>
#include <opencv2/opencv.hpp>

using namespace std;

void LoadImages(const string &strFile,vector<string> &vstrImages, vector<double> &vTimeStamps);
#endif //FERATUREEXTRACTION_LOADIMAGES_H
