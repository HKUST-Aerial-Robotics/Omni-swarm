#include "superpoint_tensorrt.h"
#include "loop_defines.h"

int main(int argc, char* argv[]) {
    if (argc < 2) {
        return -1;
    }

    std::cout << "Load Model from " << argv[1] << std::endl;
    std::string engine_path(argv[1]);
    SuperPointTensorRT sp_trt(engine_path, 0.012, true);
    std::cout << "Load Model success" << std::endl;

    cv::Mat img = cv::imread(argv[2]);
    cv::resize(img, img, cv::Size(400, 208));
    std::vector<float> local_desc;
    std::vector<cv::Point2f> kps;

    cv::Mat img_gray;
    cv::cvtColor(img, img_gray, cv::COLOR_BGR2GRAY);
    TicToc tic;
    for (unsigned int i = 0; i < 1000; i ++) {
        sp_trt.inference(img_gray, kps, local_desc);
    }
    std::cout << "1000 takes" << tic.toc() << std::endl;
    for(auto pt : kps) {
        cv::circle(img, pt, 1, cv::Scalar(255, 0, 0), -1);
    }

    cv::resize(img, img, cv::Size(), 4, 4);
    cv::imshow("Image", img);
    cv::waitKey(-1);
}