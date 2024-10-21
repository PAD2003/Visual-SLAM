#include "yolosegment.h"

namespace tpt
{
namespace objectsegment {

YoloSegment::YoloSegment()
{

}

YoloSegment::~YoloSegment()
{

}

YoloSegment::YoloSegment(std::string enginePath, int batchSize)
{
    this->init(enginePath, batchSize);
}

void YoloSegment::init(std::string enginePath, int batchSize)
{
    this->maxBatchSize = (batchSize != -1) ? batchSize : this->maxBatchSize;
    this->modelPath = enginePath;

    this->segmentModel.init(this->modelPath, this->maxBatchSize);

    this->outputModel = this->segmentModel.getOutputData();

    nvinfer1::Dims inputDims = this->segmentModel.getInputDims()[0];
    this->iWidth = inputDims.d[3];
    this->iHeight = inputDims.d[2];

    this->outputNames = this->segmentModel.getOutputNames();
    std::vector<nvinfer1::Dims> outputDims = this->segmentModel.getOutputDims();
    int outDetectDim1 = 116;
    for (size_t i = 0; i < this->outputNames.size(); i++) {
        nvinfer1::Dims dims = outputDims.at(i);
        if (this->outputNames.at(i) == this->outputBlobNameDetect) {
            indexOutputDetect = i;
            this->numBox = dims.d[2];
            outDetectDim1 = dims.d[1];
            this->outSize = getSizeByDim(dims, 1);
        } else if (this->outputNames.at(i) == this->outputBlobNameMask) {
            indexOutputMask = i;
            this->segChannels = dims.d[1];
            this->segHeight = dims.d[2];
            this->segWidth = dims.d[3];
            this->outSize1 = getSizeByDim(dims, 1);
        }
    }

    this->nClasses = outDetectDim1 - 4 - this->segChannels;
}

IFormat YoloSegment::getInputImageFormat() const
{
    return inputImageFormat;
}

void YoloSegment::setInputImageFormat(const IFormat &value)
{
    inputImageFormat = value;
}

void YoloSegment::preProcess(std::vector<cv::Mat> &batchImage)
{
    ratio_h.clear();
    ratio_w.clear();
    padh.clear();
    padw.clear();
    processImages.clear();
    srcImages.clear();
    srcImages.resize(batchImage.size());
    processImages.resize(batchImage.size());

    for (size_t ib = 0; ib < batchImage.size(); ib++) {
        srcImages.at(ib) = batchImage[ib];
        cv::Mat img = batchImage[ib];
        int w, h, x, y;
        float r_w = iWidth / (img.cols*1.0);
        float r_h = iHeight / (img.rows*1.0);
        if (r_h > r_w) {
            w = iWidth;
            h = static_cast<int>(r_w * img.rows);
            x = 0;
            y = (iHeight - h) / 2;
        }
        else {
            w = static_cast<int>(r_h * img.cols);
            h = iHeight;
            x = (iWidth - w) / 2;
            y = 0;
        }
        cv::Mat re(h, w, CV_8UC3);
        cv::resize(batchImage[ib], re, re.size(), 0, 0, cv::INTER_LINEAR);
        cv::Mat out(iHeight, iWidth, CV_8UC3, cv::Scalar(128, 128, 128));
        re.copyTo(out(cv::Rect(x, y, re.cols, re.rows)));

        padh.push_back(y);
        padw.push_back(x);
        ratio_h.push_back((float)img.rows / h);
        ratio_w.push_back((float)img.cols / w);

        float alpha = 1.0f / this->std;
        float beta = -1.0f * this->mean / this->std;
        out.convertTo(processImages.at(ib), CV_32FC3, static_cast<double>(alpha), static_cast<double>(beta));
    }
}

void YoloSegment::postProcess(float** preds, size_t batchSize, std::vector<std::vector<OutputSeg>> &segmented)
{
    segmented.resize(batchSize);
    for (size_t ib = 0; ib < batchSize; ib++) {
        std::vector<int> classIds;
        std::vector<float> confidences;
        std::vector<cv::Rect> boxes;
        std::vector<cv::Mat> picked_proposals;  //mask

        int net_length = nClasses + 4 + segChannels;
        cv::Mat out1 = cv::Mat(net_length, numBox, CV_32F, preds[indexOutputDetect]);
        for (int i = 0; i < numBox; i++) {
            // Output: 1*net_length*Num_box
            cv::Mat scores = out1(cv::Rect(i, 4, 1, nClasses)).clone();
            cv::Point classIdPoint;
            double max_class_score_;
            minMaxLoc(scores, 0, &max_class_score_, 0, &classIdPoint);
            float max_class_core = static_cast<float>(max_class_score_);
            if (max_class_core >= CONF_THRESHOLD) {
                cv::Mat temp_proto = out1(cv::Rect(i, 4 + nClasses, 1, segChannels)).clone();
                picked_proposals.push_back(temp_proto.t());
                float x = (out1.at<float>(0, i) - padw.at(ib)) * ratio_w.at(ib);  //cx
                float y = (out1.at<float>(1, i) - padh.at(ib)) * ratio_h.at(ib);  //cy
                float w = out1.at<float>(2, i) * ratio_w.at(ib);  //w
                float h = out1.at<float>(3, i) * ratio_h.at(ib);  //h
                int left = static_cast<int>(MAX((x - 0.5f * w), 0));
                int top = static_cast<int>(MAX((y - 0.5f * h), 0));
                int width = (int)w;
                int height = (int)h;
                if (width <= 0 || height <= 0) {
                    continue;
                }

                if (classIdPoint.y == 0) {
                    classIds.push_back(classIdPoint.y);
                }
                else {
                    continue;
                }
                confidences.push_back(max_class_core);
                boxes.push_back(cv::Rect(left, top, width, height));
            }
        }

        //NMS
        std::vector<int> nms_result;
        cv::dnn::NMSBoxes(boxes, confidences, CONF_THRESHOLD, NMS_THRESHOLD, nms_result);
        std::vector<cv::Mat> temp_mask_proposals;
        std::vector<OutputSeg> output;
        cv::Rect holeImgRect(0, 0, srcImages.at(ib).cols, srcImages.at(ib).rows);
        for (size_t i = 0; i < nms_result.size(); ++i) {
            int idx = nms_result[i];
            OutputSeg result;
            result.id = classIds[idx];
            result.confidence = confidences[idx];
            result.box = boxes[idx]& holeImgRect;
            output.push_back(result);
            temp_mask_proposals.push_back(picked_proposals[idx]);
        }

        if (temp_mask_proposals.size() == 0) {
            continue;
        }

        // Processing mask
        cv::Mat maskProposals;
        for (size_t i = 0; i < temp_mask_proposals.size(); ++i)
            maskProposals.push_back(temp_mask_proposals[i]);

        cv::Mat protos = cv::Mat(segChannels, segWidth * segHeight, CV_32F, preds[indexOutputMask]);
        cv::Mat matmulRes = (maskProposals * protos).t();   //n*32 32*25600 A*B
        cv::Mat masks = matmulRes.reshape(static_cast<int>(output.size()), { segWidth,segHeight }); //n*160*160

        std::vector<cv::Mat> maskChannels;
        cv::split(masks, maskChannels);



        cv::Rect roi(int((float)padw.at(ib) / iWidth * segWidth), int((float)padh.at(ib) / iHeight * segHeight), int(segWidth - padw.at(ib) / 2), int(segHeight - padh.at(ib) / 2));
        for (size_t i = 0; i < output.size(); ++i) {
            cv::Mat dest, mask;
            cv::exp(-maskChannels[i], dest);    //sigmoid
            dest = 1.0 / (1.0 + dest);      //160*160
            dest = dest(roi);
            resize(dest, mask, cv::Size(srcImages.at(ib).cols, srcImages.at(ib).rows), cv::INTER_NEAREST);
            //crop----
            cv::Rect temp_rect = output[i].box;
            mask = mask(temp_rect) > MASK_THRESHOLD;
            output[i].boxMask = mask;
        }

        segmented.at(ib) = output;
    }
}

cv::Mat YoloSegment::drawMaskPred(cv::Mat img, std::vector<OutputSeg> result)
{
    cv::Mat mask = cv::Mat::zeros(img.size(), CV_8UC1);
    for(size_t i = 0; i < result.size(); i++){
        cv::Rect box = result[i].box;
        cv::Mat objectMask = result[i].boxMask;

        //create a binary mask  for the object and place it in the appropriate location on the full mask
        cv::Mat resizeMask;
        cv::resize(objectMask, resizeMask, box.size());

        mask(box).setTo(255, resizeMask);

    }
    return mask;
}

void YoloSegment::infer(std::vector<cv::Mat> batchImage, std::vector<std::vector<OutputSeg>> &results)
{
    results.resize(batchImage.size());

    this->preProcess(batchImage);
    this->segmentModel.runInference(processImages, this->inputImageFormat);
    this->postProcess(this->outputModel, batchImage.size(), results);
}

} // namespace objectsegment
} // namespace tpt
