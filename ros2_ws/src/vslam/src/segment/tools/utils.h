#ifndef TPT_MFC_UTILS_H
#define TPT_MFC_UTILS_H

#include <fstream>
#include <iostream>
#include <string>
#include <sys/stat.h>
#include <experimental/filesystem>

#include <cuda_runtime_api.h>
#include <NvInfer.h>

namespace tpt {

#ifndef TPT_CUDA_CHECK
#define TPT_CUDA_CHECK(code) tptCudaCheckError(code)
#endif // TPT_CUDA_CHECK

static inline void tptCudaCheckError(cudaError_t code) {
    if (code != cudaSuccess) {
        std::string errMsg = "CUDA operation failed with code: " + std::to_string(code) + "(" + cudaGetErrorName(code) + "), with message: " + cudaGetErrorString(code);
        throw std::runtime_error(errMsg);
    }
}

static inline size_t getSizeByDim(const nvinfer1::Dims& dims, size_t startDim=0)
{
    size_t size = 1;
    for (size_t i = startDim; i < static_cast<size_t>(dims.nbDims); ++i)
    {
        size *= static_cast<size_t>(dims.d[i]);
    }
    return size;
}

enum class IFormat : int32_t
{
    RGB  = 0,
    BGR  = 1,
    GRAY = 2,
};

} // namespace tpt

#endif // TPT_MFC_UTILS_H
