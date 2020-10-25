#include "superpoint_tensorrt.h"

int main(int argc, char* argv[]) {
    if (argc < 2) {
        return -1;
    }

    std::cout << "Load Model from " << argv[1] << std::endl;
    std::string engine_path(argv[1]);
    SuperPointTensorRT sp_trt(engine_path, true);
    std::cout << "Load Model success" << std::endl;

    cv::Mat img = cv::imread(argv[2]);
    cv::resize(img, img, cv::Size(400, 208));
    std::vector<float> local_desc;
    std::vector<cv::Point2f> kps;

    cv::Mat img_gray;
    cv::cvtColor(img, img_gray, cv::COLOR_BGR2GRAY);
    sp_trt.inference(img_gray, kps, local_desc);
    for(auto pt : kps) {
        cv::circle(img, pt, 1, cv::Scalar(255, 0, 0), -1);
    }

    cv::resize(img, img, cv::Size(), 4, 4);
    cv::imshow("Image", img);
    cv::waitKey(-1);
}