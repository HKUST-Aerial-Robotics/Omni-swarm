#include "mobilenetvlad_tensorrt.h"

std::vector<float> MobileNetVLADTensorRT::inference(const cv::Mat & input) {
    cv::Mat _input;
    if (input.rows != height || input.cols != width) {
        cv::resize(input, _input, cv::Size(400, 208));
        _input.convertTo(_input, CV_32F, 1/255.0);
    } else {
        input.convertTo(_input, CV_32F, 1/255.0);
    }
    doInference(_input);

    return std::vector<float>(m_OutputTensors[0].hostBuffer, m_OutputTensors[0].hostBuffer+descriptor_size);
}