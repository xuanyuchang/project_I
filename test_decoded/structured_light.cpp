/*
Copyright (c) 2012, Daniel Moreno and Gabriel Taubin
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:
    * Redistributions of source code must retain the above copyright
      notice, this list of conditions and the following disclaimer.
    * Redistributions in binary form must reproduce the above copyright
      notice, this list of conditions and the following disclaimer in the
      documentation and/or other materials provided with the distribution.
    * Neither the name of the Brown University nor the
      names of its contributors may be used to endorse or promote products
      derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL DANIEL MORENO AND GABRIEL TAUBIN BE LIABLE FOR ANY
DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

#include "structured_light.hpp"

#include <iostream>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

namespace sl
{
    const float PIXEL_UNCERTAIN = std::numeric_limits<float>::quiet_NaN();
    const unsigned short BIT_UNCERTAIN = 0xffff;
};

bool sl::decode_pattern(const std::vector<cv::Mat> & images, cv::Mat & pattern_image, cv::Mat & min_max_image, cv::Size const& projector_size, unsigned flags, const cv::Mat & direct_light, unsigned m)
{
    bool binary   = (flags & GrayPatternDecode)!=GrayPatternDecode;//flags没有GrayPatternDecode，则为1
    bool robust   = (flags & RobustDecode)==RobustDecode;//flags有RobustDecode，则为1

    std::cout << " --- decode_pattern START ---\n";

    //delete previous data
    pattern_image = cv::Mat();
    min_max_image = cv::Mat();
    bool init = true;

    std::cout << "Decode: " << (binary?"Binary ":"Gray ")
                            << (robust?"Robust ":"") 
                            << std::endl;

    int total_images = static_cast<int>(images.size());//42
    int total_patterns = total_images/2 - 1;//20
    int total_bits = total_patterns/2;//编码的bit数，几位二进制[10]bit

    if (2+4*total_bits!=total_images)
    {   //error
        std::cout << "[sl::decode_pattern] ERROR: cannot detect pattern and bit count from image set.\n";
        return false;
    }

    const unsigned bit_count[] = {0, total_bits, total_bits};  //pattern bits图案的比特数 【0 10 10】
    const unsigned set_size[]  = {1, total_bits, total_bits};  //number of image pairs图像对的个数 【1 10 10】
    const unsigned COUNT = 2*(set_size[0]+set_size[1]+set_size[2]); //total image count总图像数  【42】
    ///？？？（1024*768）->(0,128)
    const int pattern_offset[2] = {((1<<total_bits)-projector_size.width)/2, ((1<<total_bits)-projector_size.height)/2};

    if (images.size()<COUNT)
    {   //error
        std::cout << "Image list size does not match set size, please supply exactly " << COUNT << " image names.\n";
        return false;
    }

    //load every image pair and compute the maximum, minimum, and bit code
    unsigned set = 0;
    unsigned current = 0;
    for (unsigned t=0; t<COUNT; t+=2, current++)
    {
        //进行第二通道的赋值：channel=1;(进行pattern的水平方向计算)
        if (current==set_size[set])
        {
            set++;
            current = 0;
        }
        //跳过第一对图像
        if (set==0)
        {   //skip
            continue;
        }

        unsigned bit = bit_count[set] - current - 1; //current bit: from 0 to (bit_count[set]-1)从第0位到第9位
        unsigned channel = set - 1;//channel为0

        //load images
        const cv::Mat & gray_image1 = images.at(t+0);//第一对图像的第一个
        if (gray_image1.rows<1)
        {
            std::cout << "Failed to load " << images.at(t+0) << std::endl;
            return false;
        }
        const cv::Mat & gray_image2 = images.at(t+1);//第一对图像的第二个
        if (gray_image2.rows<1)
        {
            std::cout << "Failed to load " << images.at(t+1) << std::endl;
            return false;
        }

        //initialize data structures
        if (init)
        {
            //sanity check
            if (gray_image1.size()!=gray_image2.size())
            {   //different size
                std::cout << " --> Initial images have different size: \n";
                return false;
            }
            if (robust && gray_image1.size()!=direct_light.size())
            {   //different size
                std::cout << " --> Direct Component image has different size: \n";
                return false;
            }
            pattern_image = cv::Mat(gray_image1.size(), CV_32FC2);//二通道的数据
            min_max_image = cv::Mat(gray_image1.size(), CV_8UC2);//二通道的数据
        }

        //sanity check
        if (gray_image1.size()!=pattern_image.size())
        {   //different size
            std::cout << " --> Image 1 has different size, image pair " << t << " (skipped!)\n";
            continue;
        }
        if (gray_image2.size()!=pattern_image.size())
        {   //different size
            std::cout << " --> Image 2 has different size, image pair " << t << " (skipped!)\n";
            continue;
        }

        //compare
        for (int h=0; h<pattern_image.rows; h++)
        {
            const unsigned char * row1 = gray_image1.ptr<unsigned char>(h);//取第一行第一个像素的指针
            const unsigned char * row2 = gray_image2.ptr<unsigned char>(h);
            const cv::Vec2b * row_light = (robust ? direct_light.ptr<cv::Vec2b>(h) : NULL);//Ld和Lg
            cv::Vec2f * pattern_row = pattern_image.ptr<cv::Vec2f>(h);
            cv::Vec2b * min_max_row = min_max_image.ptr<cv::Vec2b>(h);

            for (int w=0; w<pattern_image.cols; w++)
            {
                cv::Vec2f & pattern = pattern_row[w];//pattern_image图像的像素值
                cv::Vec2b & min_max = min_max_row[w];//min_max_image图像的像素值
                unsigned char value1 = row1[w];//第一张图片的像素值
                unsigned char value2 = row2[w];//相反的图片的像素值

                if (init)
                {
                    pattern[0] = 0.f; //vertical垂直方向的投影图案
                    pattern[1] = 0.f; //horizontal水平方向的投影图案
                }

                //min/max
                if (init || value1<min_max[0] || value2<min_max[0])
                {
                    min_max[0] = (value1<value2?value1:value2);//最小值（同一像素的两张图片比较）
                }
                if (init || value1>min_max[1] || value2>min_max[1])
                {
                    min_max[1] = (value1>value2?value1:value2);//最大值（同一像素的两张图片比较）
                }
                
                if (!robust)//不是论文所用的算法，只是简单的判断一个像素是1或者是0
                {   // [simple] pattern bit assignment
                    if (value1>value2)
                    {   //set bit n to 1
                        pattern[channel] += (1<<bit);
                    }
                }
                else
                {   // [robust] pattern bit assignment论文中的方法，具有鲁棒性
                    if (row_light && (init || /*(将或变成与，是正确的)*/pattern[channel]!=PIXEL_UNCERTAIN))//这里的判断不对，如果存在一位编码为uncertain，pixel也为uncertain，其他{位}还会进入判断
                    {
                        const cv::Vec2b & L = row_light[w];
                        //【进行robust 像素分类】（ pixel classification ）
                        unsigned short p = get_robust_bit(value1, value2, L[0], L[1], m);
                        if (p==BIT_UNCERTAIN)
                        {
                            pattern[channel] = PIXEL_UNCERTAIN;//如果存在一位编码为uncertain,那么其他位编码怎么移位依然是uncertain（不对）
                        }
                        else
                        {
                            pattern[channel] += (p<<bit);//从第一对图像开始，直到第十对图像；其中channel=0是纵向投影的解码过程；解码值（gray）存入pattern[channel]中
                        }
                    }
                }

            }   //for each column
        }   //for each row

        init = false;

    }   //for all image pairs

    if (!binary)
    {   //not binary... it must be gray code得到图案的解码值，pattern_image
        convert_pattern(pattern_image, projector_size, pattern_offset, binary);//第一个参数：判断图像是1/255来移位得到的gray码的值；
    }                                                                          //第二个参数：投影仪尺寸：1024*768；
                                                                               //第三个参数：图案的补偿：（0，128）
    std::cout << " --- decode_pattern END ---\n";                              //第四个参数：标记：是否是二进制编码：这里是false

    return true;
}

unsigned short sl::get_robust_bit(unsigned value1, unsigned value2, unsigned Ld, unsigned Lg, unsigned m)
{
    if (Ld < m)
    {
        return BIT_UNCERTAIN;
    }
    if (Ld>Lg)
    {
        return (value1>value2 ? 1 : 0);
    }
    if (value1<=Ld && value2>=Lg)
    {
        return 0;
    }
    if (value1>=Lg && value2<=Ld)
    {
        return 1;
    }
    return BIT_UNCERTAIN;
}

void sl::convert_pattern(cv::Mat & pattern_image, cv::Size const& projector_size, const int offset[2], bool binary)
{
    if (pattern_image.rows==0)
    {   //no pattern image
        return;
    }
    if (pattern_image.type()!=CV_32FC2)
    {
        return;
    }

    if (binary)
    {
       std::cout << "Converting binary code to gray\n";
    }
    else
    {
        std::cout << "Converting gray code to binary\n";
    }

   cv::Mat image=cv::Mat(pattern_image.rows,pattern_image.cols,CV_32FC1);//////测试用

    for (int h=0; h<pattern_image.rows; h++)
    {
        cv::Vec2f * pattern_row = pattern_image.ptr<cv::Vec2f>(h);
        float *data=image.ptr<float>(h);///测试用

        for (int w=0; w<pattern_image.cols; w++)
        {
            cv::Vec2f & pattern = pattern_row[w];
            if (binary)//如果是普通的二进制编码方式，(在本程序中不会执行)
            {
                if (!INVALID(pattern[0]))//判断此数是否可用，不是0/0即可用
                {
                    int p = static_cast<int>(pattern[0]);
                    pattern[0] = binaryToGray(p, offset[0]) + (pattern[0] - p);//将普通二进制编码转换为格雷码
                }
                if (!INVALID(pattern[1]))
                {
                    int p = static_cast<int>(pattern[1]);
                    pattern[1] = binaryToGray(p, offset[1]) + (pattern[1] - p);
                }
            }
            else
            {
                if (!INVALID(pattern[0]))//检查是否为uncertain像素点
                {
                    int p = static_cast<int>(pattern[0]);//p代表的是像素的格雷码值或uncertain的像素点
                    ///offset[0]=0;
                    int code = grayToBinary(p, 0/*offset[0]*/);///将格雷码转换为普通二进制编码？？？？？？？

                    if (code<0) {code = 0;}
                    //else if (code>=projector_size.width) {code = projector_size.width - 1;}//？？？？？如果码字大于2^10,则码字更正为projector_size.width - 1（这里的目的是控制码字的数值，其中码长又投影仪投射的条纹个数来确定的，因此需要控制（且根据投影仪的图像大小就可控制，如何编码，在投影仪方？？？））

                    pattern[0] = code + (pattern[0] - p);//pattern存储的时解码后的，不包括uncertain像素点的码值大小，(颜色)范围：pattern[0]=(0,projector_size.width-1);(颜色)范围:pattern[1]=(0,projector_size.height - 1)
                  data[w]=pattern[0]; ///测试用
                }
                if (!INVALID(pattern[1]))
                {
                    int p = static_cast<int>(pattern[1]);

                    int code = grayToBinary(p, 0/*offset[1]*/);///

                    if (code<0) {code = 0;}
                    //else if (code>=projector_size.height) {code = projector_size.height - 1;}

                    pattern[1] = code + (pattern[1] - p);
                    //data[w]=pattern[1];///
                }
            }
        }
    }
    ///测试用cv::imshow("pattern_image",image);
   std::cout/*<<image.row(767)*/<<image.row(300); ///测试用
}

cv::Mat sl::estimate_direct_light(const std::vector<cv::Mat> & images, float b)
{
    static const unsigned COUNT = 10; // max number of images

    unsigned count = static_cast<int>(images.size());
    if (count<1)
    {   //no images
        return cv::Mat();
    }
    
    std::cout << " --- estimate_direct_light START ---\n";

    if (count>COUNT)
    {
        count = COUNT;
        std::cout << "WARNING: Using only " << COUNT << " of " << count << std::endl;
    }

    for (unsigned i=0; i<count; i++)
    {
        if (images.at(i).type()!=CV_8UC1)
        {   //error
            std::cout << "Gray images required\n";
            return cv::Mat();
        }
    }

    cv::Size size = images.at(0).size();

    //initialize direct light image
    cv::Mat direct_light(size, CV_8UC2);//二通道的图像

    double b1 = 1.0/(1.0 - b);
    double b2 = 2.0/(1.0 - b*1.0*b);

    for (unsigned h=0; static_cast<int>(h)<size.height; h++)
    {
        unsigned char const* row[COUNT];
        for (unsigned i=0; i<count; i++)
        {
            row[i] = images.at(i).ptr<unsigned char>(h);//得到每一张图像的第一行的第一个指针
        }
        cv::Vec2b * row_light = direct_light.ptr<cv::Vec2b>(h);

        for (unsigned w=0; static_cast<int>(w)<size.width; w++)
        {
            unsigned Lmax = row[0][w];
            unsigned Lmin = row[0][w];
            for (unsigned i=0; i<count; i++)
            {
                if (Lmax<row[i][w]) Lmax = row[i][w];
                if (Lmin>row[i][w]) Lmin = row[i][w];
            }

            int Ld = static_cast<int>(b1*(Lmax - Lmin) + 0.5);
            int Lg = static_cast<int>(b2*(Lmin - b*Lmax) + 0.5);
            //if(Lg>0) std::cout<<Lg<<std::endl;
            row_light[w][0] = (Lg>0 ? static_cast<unsigned>(Ld) : Lmax);
            row_light[w][1] = (Lg>0 ? static_cast<unsigned>(Lg) : 0);

            //std::cout << "Ld=" << (int)row_light[w][0] << " iTotal=" <<(int) row_light[w][1] << std::endl;
        }
    }

    std::cout << " --- estimate_direct_light END ---\n";

    return direct_light;
}

cv::Mat sl::get_gray_image(const std::string & filename)
{
    //load image
    cv::Mat rgb_image = cv::imread(filename);
    if (rgb_image.rows>0 && rgb_image.cols>0)
    {
        //gray scale
        cv::Mat gray_image;
        cvtColor(rgb_image, gray_image, CV_BGR2GRAY);
        return gray_image;
    }
    return cv::Mat();
}

/*      From Wikipedia: http://en.wikipedia.org/wiki/Gray_code
        The purpose of this function is to convert an unsigned
        binary number to reflected binary Gray code.
*/
static unsigned util_binaryToGray(unsigned num)
{
        return (num>>1) ^ num;
}
 
/*      From Wikipedia: http://en.wikipedia.org/wiki/Gray_code
        The purpose of this function is to convert a reflected binary
        Gray code number to a binary number.
*/
static unsigned util_grayToBinary(unsigned num, unsigned numBits)
{
    for (unsigned shift = 1; shift < numBits; shift <<= 1)
    {
        num ^= num >> shift;
    }
    return num;
}

int sl::binaryToGray(int value) {return util_binaryToGray(value);}

inline int sl::binaryToGray(int value, unsigned offset) {return util_binaryToGray(value + offset);}
inline int sl::grayToBinary(int value, unsigned offset) {return (util_grayToBinary(value, 32) - offset);}

cv::Mat sl::colorize_pattern(const cv::Mat & pattern_image, unsigned set, float max_value)//将编码值着色,set=0:纵向；set=1:横向。
{
    if (pattern_image.rows==0)
    {   //empty image
        return cv::Mat();
    }
    if (pattern_image.type()!=CV_32FC2)
    {   //invalid image type
        return cv::Mat();
    }
    if (set!=0 && set!=1)
    {
        return cv::Mat();
    }

    cv::Mat image(pattern_image.size(), CV_8UC3);

    float max_t = max_value;//投影仪的宽和长（分别）
    float n = 4.f;
    float dt = 255.f/n;
    for (int h=0; h<pattern_image.rows; h++)
    {
        const cv::Vec2f * row1 = pattern_image.ptr<cv::Vec2f>(h);
        cv::Vec3b * row2 = image.ptr<cv::Vec3b>(h);
        for (int w=0; w<pattern_image.cols; w++)
        {
            if (row1[w][set]>max_value || INVALID(row1[w][set]))//不应该存在大于投影仪长和宽的解码值
            {   //invalid value: use grey
                row2[w] = cv::Vec3b(128, 128, 128);//rgb值为128.128.128是uncertain点和超出投影仪范围的解码值的点，【灰色】
                continue;
            }
            //display【黑】-》【红】-》【绿】-》【蓝】
            float t = row1[w][set]*255.f/max_t;
            float c1 = 0.f, c2 = 0.f, c3 = 0.f;
            if (t<=1.f*dt)
            {   //black -> red
                float c = n*(t-0.f*dt);
                c1 = c;     //0-255
                c2 = 0.f;   //0
                c3 = 0.f;   //0
            }
            else if (t<=2.f*dt)
            {   //red -> red,green
                float c = n*(t-1.f*dt);
                c1 = 255.f; //255
                c2 = c;     //0-255
                c3 = 0.f;   //0
            }
            else if (t<=3.f*dt)
            {   //red,green -> green
                float c = n*(t-2.f*dt);
                c1 = 255.f-c;   //255-0
                c2 = 255.f;     //255
                c3 = 0.f;       //0
            }
            else if (t<=4.f*dt)
            {   //green -> blue
                float c = n*(t-3.f*dt);
                c1 = 0.f;       //0
                c2 = 255.f-c;   //255-0
                c3 = c;         //0-255
            }
            row2[w] = cv::Vec3b(static_cast<uchar>(c3), static_cast<uchar>(c2), static_cast<uchar>(c1));
        }
    }
    return image;
}
