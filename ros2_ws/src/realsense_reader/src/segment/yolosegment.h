#ifndef TPT_YOLOSEGMENT_H
#define TPT_YOLOSEGMENT_H

#include <opencv2/opencv.hpp>
#include <iostream>
#include <vector>

#include "trtprocessor.h"

namespace tpt
{
namespace objectsegment {

struct OutputSeg {
    int id;
    float confidence;
    cv::Rect box;
    cv::Mat boxMask;
};

class YoloSegment
{
private:
    static const int VERSION_MAJOR = 1;
    static const int VERSION_MINOR = 0;

    int maxBatchSize = 1;
    std::string modelPath;

    std::string outputBlobNameDetect = "output0";
    std::string outputBlobNameMask = "output1";

    // model dims
    std::vector<std::string> outputNames;
    int numBox = 8400;
    int nClasses = 80;
    int segWidth = 160;
    int segHeight = 160;
    int segChannels = 32;

    size_t outSize; // detect
    size_t outSize1; // mask
    size_t indexOutputDetect;
    size_t indexOutputMask;

    // preprocess parameters
    int iHeight;
    int iWidth;
    std::vector<float> ratio_h;
    std::vector<float> ratio_w;
    std::vector<int> padh;
    std::vector<int> padw;
    std::vector<cv::Mat> srcImages;
    std::vector<cv::Mat> processImages;

    float mean = 0.0f;
    float std = 255.0f;
    IFormat inputImageFormat = IFormat::BGR;

    const float CONF_THRESHOLD = 0.8;
    const float NMS_THRESHOLD = 0.5;
    const float MASK_THRESHOLD = 0.45;

    /**
     * @brief mask face classification model
     */
    TRTProcessor segmentModel;

    /**
     * @brief holds binding outputs of model in cpu
     */
    float** outputModel;

    void preProcess(std::vector<cv::Mat> &batchImage);
    void postProcess(float** preds, size_t batchSize, std::vector<std::vector<OutputSeg>> &segmented);


public:
    YoloSegment();
    ~YoloSegment();
    YoloSegment(std::string enginePath, int batchSize);

    void init(std::string enginePath, int batchSize);
    /**
     * @brief inference yolo segment
     * @param batchImage list of input images
     * @param results
     */
    void infer(std::vector<cv::Mat> batchImage, std::vector<std::vector<OutputSeg>> &results);

    cv::Mat drawMaskPred(cv::Mat img, std::vector<OutputSeg> result);
    IFormat getInputImageFormat() const;
    void setInputImageFormat(const IFormat &value);
};

} // namespace objectsegment
} // namespace tpt

#endif // TPT_YOLOSEGMENT_H
