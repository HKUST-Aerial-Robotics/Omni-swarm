#include "superpoint_tensorrt.h"
#include "loop_defines.h"
#include "ATen/Parallel.h"

//NMS code is from https://github.com/KinglittleQ/SuperPoint_SLAM
void NMS2(std::vector<cv::Point2f> det, cv::Mat conf, std::vector<cv::Point2f>& pts,
            int border, int dist_thresh, int img_width, int img_height);


SuperPointTensorRT::SuperPointTensorRT(std::string engine_path, int _width, int _height, float _thres, bool _enable_perf):
    TensorRTInferenceGeneric("image", _width, _height), thres(_thres), enable_perf(_enable_perf) {
    at::set_num_threads(1);
    TensorInfo outputTensorSemi, outputTensorDesc;
    outputTensorSemi.blobName = "semi";
    outputTensorDesc.blobName = "desc";
    outputTensorSemi.volume = height*width;
    outputTensorDesc.volume = 1*256*height/8*width/8;
    m_InputSize = height*width;
    m_OutputTensors.push_back(outputTensorSemi);
    m_OutputTensors.push_back(outputTensorDesc);
    std::cout << "Trying to init TRT engine of SuperPointTensorRT" << std::endl;
    init(engine_path);
}

void SuperPointTensorRT::inference(const cv::Mat & input, std::vector<cv::Point2f> & keypoints, std::vector<float> & local_descriptors) {
    TicToc tic;
    cv::Mat _input;
    keypoints.clear();
    local_descriptors.clear();
    assert(input.rows == height && input.cols == width && "Input image must have same size with network");
    if (input.rows != height || input.cols != width) {
        cv::resize(input, _input, cv::Size(width, height));
        _input.convertTo(_input, CV_32F, 1/255.0);
    } else {
        input.convertTo(_input, CV_32F, 1/255.0);
    }
    doInference(_input);
    if (enable_perf) {
        std::cout << "Inference Time " << tic.toc();
    }

    TicToc tic1;
    auto options = torch::TensorOptions().dtype(torch::kFloat32);
    
    
    
    auto mProb = at::from_blob(m_OutputTensors[0].hostBuffer, {1, 1, height, width}, options);
    auto mDesc = at::from_blob(m_OutputTensors[1].hostBuffer, {1, 256, height/8, width/8}, options);
    cv::Mat Prob = cv::Mat(height, width, CV_32F, m_OutputTensors[0].hostBuffer);
    if (enable_perf) {
        std::cout << " from_blob " << tic1.toc();
    }

    TicToc tic2;
    getKeyPoints(Prob, thres, keypoints);
    if (enable_perf) {
        std::cout << " getKeyPoints " << tic2.toc();
    }

    computeDescriptors(mProb, mDesc, keypoints, local_descriptors);
    
    if (enable_perf) {
        std::cout << " getKeyPoints+computeDescriptors " << tic2.toc() << "inference all" << tic.toc() << "features" << keypoints.size() << "desc size" << local_descriptors.size() << std::endl;
        // cv::Mat heat(height, width, CV_32F, 1);
        // memcpy(heat.data, m_OutputTensors[0].hostBuffer, width*height * sizeof(float));
        // heat.convertTo(heat, CV_8U, 10000);
        // cv::resize(heat, heat, cv::Size(), 2, 2);
        // cv::imshow("Heat", heat);
    }
}

void SuperPointTensorRT::getKeyPoints(const cv::Mat & prob, float threshold, std::vector<cv::Point2f> &keypoints)
{
    TicToc getkps;
    auto mask = (prob > threshold);
    std::vector<cv::Point> kps;
    cv::findNonZero(mask, kps);
    std::vector<cv::Point2f> keypoints_no_nms;
    for (int i = 0; i < kps.size(); i++) {
        keypoints_no_nms.push_back(cv::Point2f(kps[i].x, kps[i].y));
    }

    cv::Mat conf(keypoints_no_nms.size(), 1, CV_32F);
    for (size_t i = 0; i < keypoints_no_nms.size(); i++) {
        int x = keypoints_no_nms[i].x;
        int y = keypoints_no_nms[i].y;
        conf.at<float>(i, 0) = prob.at<float>(y, x);
    }

    int border = 0;
    int dist_thresh = 4;
    TicToc ticnms;
    NMS2(keypoints_no_nms, conf, keypoints, border, dist_thresh, width, height);
    if (enable_perf) {
        printf(" NMS %f keypoints_no_nms %ld keypoints %ld\n", ticnms.toc(), keypoints_no_nms.size(), keypoints.size());
    }
}


void SuperPointTensorRT::computeDescriptors(const torch::Tensor & mProb, const torch::Tensor & mDesc, const std::vector<cv::Point2f> &keypoints, std::vector<float> & local_descriptors) {
    TicToc tic;
    cv::Mat kpt_mat(keypoints.size(), 2, CV_32F);  // [n_keypoints, 2]  (y, x)
    for (size_t i = 0; i < keypoints.size(); i++) {
        kpt_mat.at<float>(i, 0) = (float)keypoints[i].y;
        kpt_mat.at<float>(i, 1) = (float)keypoints[i].x;
    }


    auto fkpts = torch::from_blob(kpt_mat.data, {keypoints.size(), 2}, torch::kFloat);

    auto grid = torch::zeros({1, 1, fkpts.size(0), 2});  // [1, 1, n_keypoints, 2]
    grid[0][0].slice(1, 0, 1) = 2.0 * fkpts.slice(1, 1, 2) / width - 1;  // x
    grid[0][0].slice(1, 1, 2) = 2.0 * fkpts.slice(1, 0, 1) / height - 1;  // y

    // mDesc.to(torch::kCUDA);
    // grid.to(torch::kCUDA);
    auto desc = torch::grid_sampler(mDesc, grid, 0, 0, 0);

    desc = desc.squeeze(0).squeeze(1);

    // normalize to 1
    auto dn = torch::norm(desc, 2, 1);
    desc = desc.div(torch::unsqueeze(dn, 1));

    desc = desc.transpose(0, 1).contiguous();
    desc = desc.to(torch::kCPU);

    local_descriptors = std::vector<float>(desc.data<float>(), desc.data<float>()+desc.size(1)*desc.size(0));
    if (enable_perf) {
        std::cout << " computeDescriptors full " << tic.toc() << std::endl;
    }
}

void NMS2(std::vector<cv::Point2f> det, cv::Mat conf, std::vector<cv::Point2f>& pts,
            int border, int dist_thresh, int img_width, int img_height)
{

    std::vector<cv::Point2f> pts_raw = det;

    cv::Mat grid = cv::Mat(cv::Size(img_width, img_height), CV_8UC1);
    cv::Mat inds = cv::Mat(cv::Size(img_width, img_height), CV_16UC1);

    cv::Mat confidence = cv::Mat(cv::Size(img_width, img_height), CV_32FC1);

    grid.setTo(0);
    inds.setTo(0);
    confidence.setTo(0);

    for (unsigned int i = 0; i < pts_raw.size(); i++)
    {   
        int uu = (int) pts_raw[i].x;
        int vv = (int) pts_raw[i].y;

        grid.at<char>(vv, uu) = 1;
        inds.at<unsigned short>(vv, uu) = i;

        confidence.at<float>(vv, uu) = conf.at<float>(i, 0);
    }

    for (int i = 0; i < pts_raw.size(); i++)
    {   
        int uu = (int) pts_raw[i].x;
        int vv = (int) pts_raw[i].y;

        if (grid.at<char>(vv, uu) != 1)
            continue;

        for(int k = -dist_thresh; k < (dist_thresh+1); k++)
            for(int j = -dist_thresh; j < (dist_thresh+1); j++)
            {
                if(j==0 && k==0) continue;

                if ( confidence.at<float>(vv + k, uu + j) < confidence.at<float>(vv, uu) ) {
                    grid.at<char>(vv + k, uu + j) = 0;
                }
            }
        grid.at<char>(vv, uu) = 2;
    }

    size_t valid_cnt = 0;

    for (int v = 0; v < (img_height); v++){
        for (int u = 0; u < (img_width); u++)
        {
            if (u>= (img_width - border) || u < border || v >= (img_height - border) || v < border)
            continue;

            if (grid.at<char>(v,u) == 2)
            {
                int select_ind = (int) inds.at<unsigned short>(v, u);
                cv::Point2f p = pts_raw[select_ind];
                pts.push_back(p);
                valid_cnt++;
            }
        }
    }
}