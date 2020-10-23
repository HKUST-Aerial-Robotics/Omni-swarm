#include "NvInfer.h"
#include <opencv2/opencv.hpp>
#include <trt_utils.h>


//Original code from https://github.com/enazoe/yolo-tensorrt

struct TensorInfo
{
    std::string blobName;
    float* hostBuffer{nullptr};
    uint64_t volume{0};
    int bindingIndex{-1};
};


class TensorRTInferenceGeneric {
protected:
    Logger m_Logger;
    nvinfer1::ICudaEngine* m_Engine = nullptr;
    int m_InputBindingIndex;
    uint64_t m_InputSize;
    nvinfer1::IExecutionContext* m_Context;
    std::vector<void*> m_DeviceBuffers;
    cudaStream_t m_CudaStream;
    std::vector<TensorInfo> m_OutputTensors;
    int m_BatchSize = 1;
    const std::string m_InputBlobName;
public:
    TensorRTInferenceGeneric(std::string input_blob_name);

    virtual void doInference(const unsigned char* input, const uint32_t batchSize);

    virtual void doInference(const cv::Mat & input);

    bool verifyEngine();

    void allocateBuffers();

    void init(const std::string & engine_path);
};


class SuperPointTensorRT: public TensorRTInferenceGeneric {
public:
    SuperPointTensorRT(std::string engine_path) : TensorRTInferenceGeneric("image") {
        TensorInfo outputTensorSemi, outputTensorDesc;
        outputTensorSemi.blobName = "semi";
        outputTensorDesc.blobName = "desc";
        outputTensorSemi.volume = 65*26*50;
        outputTensorDesc.volume = 1*256*26*50;
        m_InputSize = 208*400;
        m_OutputTensors.push_back(outputTensorSemi);
        m_OutputTensors.push_back(outputTensorDesc);

        init(engine_path);
    }

    void inference(const cv::Mat & input, std::vector<cv::Point2f> & keypoints, std::vector<float> & local_descriptors) {
        doInference(input);
    }
};