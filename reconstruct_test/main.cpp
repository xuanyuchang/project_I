
#include"Application.hpp"

#include<opencv2/core/core.hpp>
#include<opencv2/imgproc/imgproc.hpp>
#include<opencv2/calib3d/calib3d.hpp>
#include<opencv2/highgui/highgui.hpp>
#include"CalibrationData.hpp"
#include"structured_light.hpp"

#include<QString>

using namespace cv;
using namespace std;


int main(int argc, char *argv[])
{
    Application app(argc, argv);
    APP->loadimage();
    APP->load_calib();

    APP->reconstruct();

    return app.exec();
}

