#include "trtprocessor.h"

namespace tpt
{
namespace objectsegment {

TRTProcessor::TRTProcessor()
{

}

TRTProcessor::TRTProcessor(std::string engine_path, int batchSize)
{
    this->init(engine_path, batchSize);
}

void TRTProcessor::init(std::string engineModelPath, int batchSize)
{
    this->maxBatchSize = batchSize;
    this->mOptions.maxBatchSize = batchSize;
    this->mOptions.optBatchSize = batchSize;

    this->mEngineName = engineModelPath;
    if (!BaseHelper::doesFileExist(engineModelPath))
    {
        std::cout << "Engine not found, generating. This could take a while..." << std::endl;
        std::string onnxFile = BaseHelper::replaceExtension(engineModelPath, ".onnx");
        if (!BaseHelper::doesFileExist(onnxFile))
            throw std::runtime_error("Could not find model at path: " + onnxFile);
        bool res = this->buildEngine(onnxFile);

        if (!res)
            throw std::runtime_error("Could not build engine model from onnx path: " + onnxFile);
    }
    this->loadNetwork(this->mEngineName);
}

bool TRTProcessor::buildEngine(std::string onnxModelPath)
{
    // Create our engine builder.
    auto builder = std::unique_ptr<nvinfer1::IBuilder>(nvinfer1::createInferBuilder(this->mLogger));
    if (!builder) {
        return false;
    }

    // Define an explicit batch size and then create the network (implicit batch size is deprecated).
    // More info here: https://docs.nvidia.com/deeplearning/tensorrt/developer-guide/index.html#explicit-implicit-batch
    auto explicitBatch = 1U << static_cast<uint32_t>(nvinfer1::NetworkDefinitionCreationFlag::kEXPLICIT_BATCH);
    auto network = std::unique_ptr<nvinfer1::INetworkDefinition>(builder->createNetworkV2(explicitBatch));
    if (!network) {
        return false;
    }

    // Create a parser for reading the onnx file.
    auto parser = std::unique_ptr<nvonnxparser::IParser>(nvonnxparser::createParser(*network, this->mLogger));
    if (!parser) {
        return false;
    }

    // We are going to first read the onnx file into memory, then pass that buffer to the parser.
    // Had our onnx model file been encrypted, this approach would allow us to first decrypt the buffer.
    std::ifstream file(onnxModelPath, std::ios::binary | std::ios::ate);
    std::streamsize size = file.tellg();
    file.seekg(0, std::ios::beg);

    std::vector<char> buffer(size);
    if (!file.read(buffer.data(), size)) {
        throw std::runtime_error("Unable to read engine file");
    }

    // Parse the buffer we read into memory.
    auto parsed = parser->parse(buffer.data(), buffer.size());
    if (!parsed) {
        return false;
    }

    // Ensure that all the inputs have the same batch size
    const auto numInputs = network->getNbInputs();
    if (numInputs < 1) {
        throw std::runtime_error("Error, model needs at least 1 input!");
    }
    const auto input0Batch = network->getInput(0)->getDimensions().d[0];
    for (int32_t i = 1; i < numInputs; ++i) {
        if (network->getInput(i)->getDimensions().d[0] != input0Batch) {
            throw std::runtime_error("Error, the model has multiple inputs, each with differing batch sizes!");
        }
    }

    // Check to see if the model supports dynamic batch size or not
    bool doesSupportDynamicBatch = false;
    if (input0Batch == -1) {
        doesSupportDynamicBatch = true;
        std::cout << "Model supports dynamic batch size" << std::endl;
    } else {
        std::cout << "Model only supports fixed batch size of " << input0Batch << std::endl;
        // If the model supports a fixed batch size, ensure that the maxBatchSize and optBatchSize were set correctly.
        if (this->mOptions.optBatchSize != input0Batch || this->mOptions.maxBatchSize != input0Batch) {
//            throw std::runtime_error("Error, model only supports a fixed batch size of " + std::to_string(input0Batch) +
//            ". Must set Options.optBatchSize and Options.maxBatchSize to 1");
            this->mOptions.optBatchSize = input0Batch;
            this->mOptions.maxBatchSize = input0Batch;
        }
    }

    auto config = std::unique_ptr<nvinfer1::IBuilderConfig>(builder->createBuilderConfig());
    if (!config) {
        return false;
    }

    // Register a single optimization profile
    nvinfer1::IOptimizationProfile *optProfile = builder->createOptimizationProfile();
    for (int32_t i = 0; i < numInputs; ++i) {
        // Must specify dimensions for all the inputs the model expects.
        const auto input = network->getInput(i);
        const auto inputName = input->getName();
        const auto inputDims = input->getDimensions();
        int32_t inputC = inputDims.d[1];
        int32_t inputH = inputDims.d[2];
        int32_t inputW = inputDims.d[3];

        // Specify the optimization profile`
        if (doesSupportDynamicBatch) {
            optProfile->setDimensions(inputName, nvinfer1::OptProfileSelector::kMIN, nvinfer1::Dims4(1, inputC, inputH, inputW));
        } else {
            optProfile->setDimensions(inputName, nvinfer1::OptProfileSelector::kMIN, nvinfer1::Dims4(this->mOptions.optBatchSize, inputC, inputH, inputW));
        }
        optProfile->setDimensions(inputName, nvinfer1::OptProfileSelector::kOPT, nvinfer1::Dims4(this->mOptions.optBatchSize, inputC, inputH, inputW));
        optProfile->setDimensions(inputName, nvinfer1::OptProfileSelector::kMAX, nvinfer1::Dims4(this->mOptions.maxBatchSize, inputC, inputH, inputW));
    }
    config->addOptimizationProfile(optProfile);

    // Set the precision level
    // currently not support INT8 calibration
    if (this->mOptions.precision == Precision::FP16) {
        // Ensure the GPU supports FP16 inference
        if (!builder->platformHasFastFp16()) {
            throw std::runtime_error("Error: GPU does not support FP16 precision");
        }
        config->setFlag(nvinfer1::BuilderFlag::kFP16);
    }

    // CUDA stream used for profiling by the builder.
    cudaStream_t profileStream;
    TPT_CUDA_CHECK(cudaStreamCreate(&profileStream));
    config->setProfileStream(profileStream);

    // Build the engine
    // If this call fails, it is suggested to increase the logger verbosity to kVERBOSE and try rebuilding the engine.
    // Doing so will provide you with more information on why exactly it is failing.
    std::unique_ptr<nvinfer1::IHostMemory> plan{builder->buildSerializedNetwork(*network, *config)};
    if (!plan) {
        return false;
    }

    // Write the engine to disk
    std::ofstream outfile(this->mEngineName, std::ofstream::binary);
    outfile.write(reinterpret_cast<const char*>(plan->data()), plan->size());

    std::cout << "Success, saved engine to " << this->mEngineName << std::endl;

    TPT_CUDA_CHECK(cudaStreamDestroy(profileStream));
    return true;
}

float** TRTProcessor::getOutputData()
{
    return this->outputData;
}

bool TRTProcessor::loadNetwork(std::string engineModelPath)
{
    std::ifstream file(engineModelPath, std::ios::binary | std::ios::ate);
    std::streamsize fsize = file.tellg();
    file.seekg(0, std::ios::beg);

    std::vector<char> buffer(static_cast<size_t>(fsize));
    if (!file.read(buffer.data(), fsize)) {
        throw std::runtime_error("Unable to read engine file");
    }

    file.close();

    std::unique_ptr<nvinfer1::IRuntime> runtime{nvinfer1::createInferRuntime(this->mLogger)};
    if (!runtime) {
        return false;
    }

    mEngine = std::unique_ptr<nvinfer1::ICudaEngine>(runtime->deserializeCudaEngine(buffer.data(), buffer.size()));
    if (!mEngine) {
        return false;
    }

//    if (!m_engine->bindingIsInput(0)) {
//        throw std::runtime_error("Error, the model does not have an input!");
//    }

    mContext = std::unique_ptr<nvinfer1::IExecutionContext>(mEngine->createExecutionContext());
    if (!mContext) {
        return false;
    }

//    TPT_CUDA_CHECK(cudaStreamCreate(&m_stream));

    // Allocate the input and output buffers
    int nIOTensors = mEngine->getNbIOTensors();
    mBuffers.resize(static_cast<size_t>(nIOTensors));

    for (int i = 0; i < nIOTensors; i++) {
        std::string bname = mEngine->getIOTensorName(i);
        nvinfer1::Dims bsize = mEngine->getTensorShape(bname.c_str());
        bsize.d[0] = this->maxBatchSize;
//        nvinfer1::DataType btype = m_engine->getTensorDataType(bname.c_str());

        if (mEngine->getTensorIOMode(bname.c_str()) == nvinfer1::TensorIOMode::kINPUT) {
            this->inputNames.push_back(bname);
            this->inputDims.push_back(bsize);
        } else {
            this->outputNames.push_back(bname);
            this->outputDims.push_back(bsize);
        }
        TPT_CUDA_CHECK(cudaMalloc(&mBuffers[static_cast<size_t>(i)], getSizeByDim(bsize) * sizeof(float)));
    }

    this->inputCnt = inputNames.size();
    this->outputCnt = outputNames.size();
    // Init output data size
    outputData = new float*[this->outputCnt];
    for (size_t index = 0; index < outputNames.size(); index++) {
        outputData[index] = new float[getSizeByDim(this->outputDims[index])];
    }

    return true;
}

cv::Mat TRTProcessor::BGRhwc2RGBchw(cv::Mat bgr)
{
    int c = bgr.channels();
    int h = bgr.rows;
    int w = bgr.cols;
    int sz[] = {c, h, w};
    cv::Mat chw(3, sz, bgr.depth()); // dst

    std::vector<cv::Mat> channels = {
        cv::Mat(h, w, chw.depth(), chw.ptr<uchar>(2)),  // r
        cv::Mat(h, w, chw.depth(), chw.ptr<uchar>(1)),  // g
        cv::Mat(h, w, chw.depth(), chw.ptr<uchar>(0))   // b
    };
    cv::split(bgr, channels);
    return chw;
}

cv::Mat TRTProcessor::RGBhwc2RGBchw(cv::Mat bgr)
{
    int c = bgr.channels();
    int h = bgr.rows;
    int w = bgr.cols;
    int sz[] = {c, h, w};
    cv::Mat chw(3, sz, bgr.depth()); // dst

    std::vector<cv::Mat> channels = {
        cv::Mat(h, w, chw.depth(), chw.ptr<uchar>(0)),  // r
        cv::Mat(h, w, chw.depth(), chw.ptr<uchar>(1)),  // g
        cv::Mat(h, w, chw.depth(), chw.ptr<uchar>(2))   // b
    };
    cv::split(bgr, channels);
    return chw;
}

cv::Mat TRTProcessor::RGBchw2BGRhwc(cv::Mat rgb)
{
    int imgh = rgb.size[1];
    int imgw = rgb.size[2];
    // std::cout << rgb.size[0] << " " << rgb.size[1] << " " << rgb.size[2] << "\n";
    cv::Mat bgr(imgh, imgw, CV_8UC3);
    for (int y = 0; y < imgh; y++) {
        for (int x = 0; x < imgw; x++) {
            bgr.at<cv::Vec3b>(y, x)[0] = *(rgb.ptr<uchar>(0, 0, 0) + 2 * imgh * imgw + y * imgw + x);
            bgr.at<cv::Vec3b>(y, x)[1] = *(rgb.ptr<uchar>(0, 0, 0) + imgh * imgw     + y * imgw + x);
            bgr.at<cv::Vec3b>(y, x)[2] = *(rgb.ptr<uchar>(0, 0, 0) +                 + y * imgw + x);
        }
    }
    return bgr;
}

void TRTProcessor::preProcess(const cv::Mat img, float* gpu_out, const nvinfer1::Dims &dims,const IFormat format)
{
    cv::Mat tensorImage;
    switch (format) {
        case IFormat::RGB:
            tensorImage = RGBhwc2RGBchw(img);
            break;
        case IFormat::BGR:
            tensorImage = BGRhwc2RGBchw(img);
            break;
        case IFormat::GRAY:
            tensorImage = img.clone();
            break;
    }

    int c = img.channels();
    int h = img.rows;
    int w = img.cols;
    TPT_CUDA_CHECK(cudaMemcpy(gpu_out, tensorImage.ptr(0), c*h*w*sizeof(float), cudaMemcpyHostToDevice));
}

void TRTProcessor::preProcess(const std::vector<cv::Mat> imgs, float *gpu_out, const nvinfer1::Dims &dims, const IFormat format)
{
    size_t imgSize = getSizeByDim(dims, 1);
    for (size_t i=0; i < imgs.size(); i++) {
        this->preProcess(imgs.at(i), gpu_out + i * imgSize, dims, format);
    }
}

//bool TRTProcessor::runInference(std::vector<cv::Mat> inputs)
//{
//    this->preProcess(inputs, static_cast<float*>(mBuffers[0]), inputDims[0]);
//    bool res = mContext->executeV2(mBuffers.data());

//    if (!res)
//        return false;
//    this->postProcess();

//    return true;
//}

bool TRTProcessor::runInference(std::vector<cv::Mat> inputs, const IFormat format, std::vector<float*> features)
{
    this->preProcess(inputs, static_cast<float*>(mBuffers[0]), inputDims[0], format);
    if (features.size() > 0) {
        for (size_t i = 0; i < this->inputCnt - 1; i++) {
            TPT_CUDA_CHECK(cudaMemcpy(static_cast<float*>(mBuffers[1 + i]), features[i],
                               getSizeByDim(this->inputDims[1 + i]) * sizeof(float), cudaMemcpyHostToDevice));
        }
    }
    nvinfer1::Dims indims = this->inputDims[0];
    indims.d[0] = static_cast<int>(inputs.size());
    bool success = this->mContext->setInputShape(this->inputNames[0].c_str(), indims);

    bool res = this->mContext->executeV2(mBuffers.data());
//    m_context->enqueueV2(m_buffers.data(), m_stream, nullptr);

    if (!res)
        return false;
    this->postProcess();

    return true;
}

void TRTProcessor::postProcess()
{
    // copy results from GPU to CPU
    for (size_t i = 0; i < this->outputCnt; i++) {
        size_t bindingSize = getSizeByDim(this->outputDims[i]);
        TPT_CUDA_CHECK(cudaMemcpy(outputData[i], static_cast<float*>(mBuffers[this->inputCnt + i]),
                           bindingSize * sizeof(float), cudaMemcpyDeviceToHost));
    }
}

TRTProcessor::~TRTProcessor()
{
    for (size_t i = 0; i < this->outputCnt; i++) {
        delete[] this->outputData[i];
    }

    delete[] outputData;

    // Free the GPU memory
    for (auto & buffer : mBuffers) {
        TPT_CUDA_CHECK(cudaFree(buffer));
    }

    mBuffers.clear();
}

size_t TRTProcessor::getInputCnt() {
    return this->inputCnt;
}

size_t TRTProcessor::getOutputCnt() {
    return this->outputCnt;
}

std::vector<nvinfer1::Dims> TRTProcessor::getInputDims() {
    return this->inputDims;
}

std::vector<nvinfer1::Dims> TRTProcessor::getOutputDims() {
    return this->outputDims;
}

std::vector<std::string> TRTProcessor::getInputNames()
{
    return this->inputNames;
}

std::vector<std::string> TRTProcessor::getOutputNames()
{
    return this->outputNames;
}

} // namespace objectsegment
} // namespace tpt
