#include <sstream>
#include "LoadImages.h"


void LoadImages(const string &strFile,vector<string> &vstrImages, vector<double> &vTimeStamps)
{
    string sRGB;
    double t;
    ifstream f;
    f.open(strFile);
    while(!f.eof())
    {
        string s;
        getline(f,s);
        if(!s.empty())
        {
            stringstream ss;
            ss << s;
            ss >> t;
            vTimeStamps.push_back(t);
            ss >> sRGB;
            vstrImages.push_back(sRGB);
        }
    }
}
