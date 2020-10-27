#pragma once

#ifdef USE_TENSORRT
#include "tensorrt_generic.h"
#include <torch/csrc/autograd/variable.h>
#include <ATen/ATen.h>
#include <torch/csrc/api/include/torch/types.h>

class SuperPointTensorRT: public TensorRTInferenceGeneric {
public:
    int width = 400;
    int height = 208;
    double thres = 0.015;
    bool enable_perf;
    SuperPointTensorRT(std::string engine_path, float _thres = 0.015, bool _enable_perf = false);

    void getKeyPoints(const cv::Mat & prob, float threshold, std::vector<cv::Point2f> &keypoints);
    void computeDescriptors(const torch::Tensor & mProb, const torch::Tensor & desc, const std::vector<cv::Point2f> &keypoints, std::vector<float> & local_descriptors);

    void inference(const cv::Mat & input, std::vector<cv::Point2f> & keypoints, std::vector<float> & local_descriptors);
};
#endif