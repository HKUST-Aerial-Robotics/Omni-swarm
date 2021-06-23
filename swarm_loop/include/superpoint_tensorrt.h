#pragma once

#ifdef USE_TENSORRT
#include "tensorrt_generic.h"
#include <torch/csrc/autograd/variable.h>
#include <ATen/ATen.h>
#include <torch/csrc/api/include/torch/types.h>

class SuperPointTensorRT: public TensorRTInferenceGeneric {
public:
    double thres = 0.015;
    bool enable_perf;
    int max_num = 200;
    SuperPointTensorRT(std::string engine_path, int _width, int _height, float _thres = 0.015, int _max_num = 200, bool _enable_perf = false);

    void getKeyPoints(const cv::Mat & prob, float threshold, std::vector<cv::Point2f> &keypoints);
    void computeDescriptors(const torch::Tensor & mProb, const torch::Tensor & desc, const std::vector<cv::Point2f> &keypoints, std::vector<float> & local_descriptors);

    void inference(const cv::Mat & input, std::vector<cv::Point2f> & keypoints, std::vector<float> & local_descriptors);
};
#endif