#pragma once

#ifdef USE_TENSORRT
#include "tensorrt_generic.h"

class MobileNetVLADTensorRT: public TensorRTInferenceGeneric {
public:
    int width = 400;
    int height = 208;
    bool enable_perf;
    const int descriptor_size = 4096;
    MobileNetVLADTensorRT(std::string engine_path, bool _enable_perf = false) : TensorRTInferenceGeneric("image:0"), enable_perf(_enable_perf) {
        TensorInfo outputTensorDesc;
        outputTensorDesc.blobName = "descriptor:0";
        outputTensorDesc.volume = descriptor_size;
        m_InputSize = height*width;
        m_OutputTensors.push_back(outputTensorDesc);
        std::cout << "Trying to init TRT engine of MobileNetVLADTensorRT" << std::endl;

        init(engine_path);
    }

    std::vector<float> inference(const cv::Mat & input);
};
#endif