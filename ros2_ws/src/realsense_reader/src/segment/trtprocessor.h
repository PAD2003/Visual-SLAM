#ifndef TPT_TRTPROCESSOR_H
#define TPT_TRTPROCESSOR_H

#include <iostream>
#include <fstream>
#include <vector>

#include <opencv2/opencv.hpp>
#include <opencv2/core/cuda.hpp>

#include <cuda_runtime_api.h>
#include <NvInfer.h>
#include <NvInferRuntime.h>
#include "NvOnnxParser.h"

#include "utils.h"
#include "basehelper.h"
#include "trtlogger.h"

namespace tpt
{
namespace objectsegment {

// Precision used for GPU inference
enum class Precision {
    // Full precision floating point value
    FP32,
    // Half prevision floating point value
    FP16,
    // Int8 quantization.
    // Has reduced dynamic range, may result in slight loss in accuracy.
    // If INT8 is selected, must provide path to calibration dataset directory.
    INT8,
};

// Options for the network
struct Options {
    // Precision to use for GPU inference.
    Precision precision = Precision::FP16;
    // If INT8 precision is selected, must provide path to calibration dataset directory.
    std::string calibrationDataDirectoryPath;
    // The batch size to be used when computing calibration data for INT8 inference.
    // Should be set to as large a batch number as your GPU will support.
    int32_t calibrationBatchSize = 128;
    // The batch size which should be optimized for.
    int32_t optBatchSize = 16;
    // Maximum allowable batch size
    int32_t maxBatchSize = 16;
    // GPU device index
    int deviceIndex = 0;
};

class TRTProcessor
{
protected:
    // Holds pointers to the input and output GPU buffers
    std::vector<void*> mBuffers;
    std::vector<uint32_t> mOutputLengthsFloat{};

    // Holds binding name of inputs and outputs
    std::vector<std::string> inputNames;
    std::vector<std::string> outputNames;

    // Hold binding dims of inputs and outputs
    std::vector<nvinfer1::Dims> inputDims;
    std::vector<nvinfer1::Dims> outputDims;

    // number of binding input and number output
    size_t inputCnt;
    size_t outputCnt;
    int maxBatchSize = 1;

    std::unique_ptr<nvinfer1::ICudaEngine> mEngine = nullptr;
    std::unique_ptr<nvinfer1::IExecutionContext> mContext = nullptr;

    Options mOptions;
    Logger mLogger;
    std::string mEngineName;

    // Store ouput pointer in CPU
    float**  outputData;

    // get output data from GPU to CPU
    void postProcess();
    void preProcess(const cv::Mat img, float* gpu_out, const nvinfer1::Dims &dims, const IFormat format=IFormat::BGR);
    void preProcess(const std::vector<cv::Mat> imgs, float* gpu_out, const nvinfer1::Dims &dims, const IFormat format=IFormat::BGR);

    cv::Mat BGRhwc2RGBchw(cv::Mat bgr);
    cv::Mat RGBhwc2RGBchw(cv::Mat bgr);
    cv::Mat RGBchw2BGRhwc(cv::Mat bgr);

public:
    TRTProcessor();
    TRTProcessor(std::string enginePath, int batchSize);
//    TRTProcessor(std::string onnxPath, size_t batchSize);

    ~TRTProcessor();

    void init(std::string engineModelPath, int batchSize);
    /**
     * @brief create engine file from onnx file
     * @param onnxPath
     */
    bool buildEngine(std::string onnxModelPath);

    /**
     * @brief load engine and prepare network
     * @return
     */
    bool loadNetwork(std::string engineModelPath);

    /**
     * @brief inference trt model with input images
     * @return
     */
//    bool runInference(std::vector<cv::Mat> inputs);

    bool runInference(std::vector<cv::Mat> inputs, const IFormat format=IFormat::BGR, std::vector<float*> features = {});

    float** getOutputData();

    size_t getInputCnt();

    size_t getOutputCnt();

    std::vector<nvinfer1::Dims> getInputDims();

    std::vector<nvinfer1::Dims> getOutputDims();

    std::vector<std::string> getInputNames();

    std::vector<std::string> getOutputNames();
};

} // namespace objectsegment
} // namespace tpt


#endif // TPT_TRTPROCESSOR_H
