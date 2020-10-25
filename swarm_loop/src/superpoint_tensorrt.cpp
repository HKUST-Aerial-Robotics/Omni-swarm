#include "superpoint_tensorrt.h"
#include "loop_defines.h"


//NMS code is from https://github.com/KinglittleQ/SuperPoint_SLAM
void NMS2(std::vector<cv::Point2f> det, cv::Mat conf, std::vector<cv::Point2f>& pts,
            int border, int dist_thresh, int img_width, int img_height);

using namespace nvinfer1;
uint64_t get3DTensorVolume4(nvinfer1::Dims inputDims);
TensorRTInferenceGeneric::TensorRTInferenceGeneric(std::string input_blob_name):
    m_InputBlobName(input_blob_name) {

}

void TensorRTInferenceGeneric::init(const std::string & engine_path) {
    printf("Trying to load TRT engine of superpoint");
    m_Engine = loadTRTEngine(engine_path, nullptr, m_Logger);
    assert(m_Engine != nullptr);
    
    m_Context = m_Engine->createExecutionContext();
	assert(m_Context != nullptr);
	m_InputBindingIndex = m_Engine->getBindingIndex(m_InputBlobName.c_str());
	assert(m_InputBindingIndex != -1);
    std::cout << "MaxBatchSize" << m_Engine->getMaxBatchSize();
	assert(m_BatchSize <= static_cast<uint32_t>(m_Engine->getMaxBatchSize()));
	allocateBuffers();
	NV_CUDA_CHECK(cudaStreamCreate(&m_CudaStream));
	assert(verifyEngine());
}

void TensorRTInferenceGeneric::doInference(const cv::Mat & input) {
    assert(input.channels() == 1 && "Only support 1 channel now");
    TicToc inference;
    //This function is very slow event on i7, we need to optimize it
    //But not now.
    cv::Mat trtInput = cv::dnn::blobFromImages(input, 1.0, cv::Size(input.cols, input.rows),
                                   cv::Scalar(0.0, 0.0, 0.0), false);
    double dt_blob = inference.toc(); 
    doInference(input.data, 1);
    printf("Inference blob %fms full %fms", dt_blob, inference.toc());
}


void TensorRTInferenceGeneric::doInference(const unsigned char* input, const uint32_t batchSize)
{
	//Timer timer;
    assert(batchSize <= m_BatchSize && "Image batch size exceeds TRT engines batch size");
    NV_CUDA_CHECK(cudaMemcpyAsync(m_DeviceBuffers.at(m_InputBindingIndex), input,
                                  batchSize * m_InputSize * sizeof(float), cudaMemcpyHostToDevice,
                                  m_CudaStream));
	
    m_Context->enqueue(batchSize, m_DeviceBuffers.data(), m_CudaStream, nullptr);
    for (auto& tensor : m_OutputTensors)
    {
        NV_CUDA_CHECK(cudaMemcpyAsync(tensor.hostBuffer, m_DeviceBuffers.at(tensor.bindingIndex),
                                      batchSize * tensor.volume * sizeof(float),
                                      cudaMemcpyDeviceToHost, m_CudaStream));
    }
    cudaStreamSynchronize(m_CudaStream);
//	timer.out("inference");
}

bool TensorRTInferenceGeneric::verifyEngine()
{
    std::cout << "NB bindings" <<  m_Engine->getNbBindings() <<  " output tensor " << m_OutputTensors.size() << std::endl;
    assert((m_Engine->getNbBindings() == (1 + m_OutputTensors.size())
            && "Binding info doesn't match between cfg and engine file \n"));

    for (auto tensor : m_OutputTensors)
    {
        assert(!strcmp(m_Engine->getBindingName(tensor.bindingIndex), tensor.blobName.c_str())
               && "Blobs names dont match between cfg and engine file \n");
        assert(get3DTensorVolume4(m_Engine->getBindingDimensions(tensor.bindingIndex))
                   == tensor.volume
               && "Tensor volumes dont match between cfg and engine file \n");
    }

    assert(m_Engine->bindingIsInput(m_InputBindingIndex) && "Incorrect input binding index \n");
    assert(m_Engine->getBindingName(m_InputBindingIndex) == m_InputBlobName
           && "Input blob name doesn't match between config and engine file");
    assert(get3DTensorVolume4(m_Engine->getBindingDimensions(m_InputBindingIndex)) == m_InputSize);
    return true;
}

void TensorRTInferenceGeneric::allocateBuffers()
{
    m_DeviceBuffers.resize(m_Engine->getNbBindings(), nullptr);
    assert(m_InputBindingIndex != -1 && "Invalid input binding index");
    NV_CUDA_CHECK(cudaMalloc(&m_DeviceBuffers.at(m_InputBindingIndex),
                             m_BatchSize * m_InputSize * sizeof(float)));

    for (auto& tensor : m_OutputTensors)
    {
        tensor.bindingIndex = m_Engine->getBindingIndex(tensor.blobName.c_str());
        assert((tensor.bindingIndex != -1) && "Invalid output binding index");
        NV_CUDA_CHECK(cudaMalloc(&m_DeviceBuffers.at(tensor.bindingIndex),
                                 m_BatchSize * tensor.volume * sizeof(float)));
        NV_CUDA_CHECK(
            cudaMallocHost(&tensor.hostBuffer, tensor.volume * m_BatchSize * sizeof(float)));
    }
}


uint64_t get3DTensorVolume4(nvinfer1::Dims inputDims)
{
    return inputDims.d[0] * inputDims.d[1] * inputDims.d[2] *  inputDims.d[3];
}


void SuperPointTensorRT::inference(const cv::Mat & input, std::vector<cv::Point2f> & keypoints, std::vector<float> & local_descriptors) {
    doInference(input);
    auto options = torch::TensorOptions().dtype(torch::kFloat32);
    auto mProb = at::from_blob(m_OutputTensors[0].hostBuffer, {1, 1, height, width}, options);
    auto mDesc = at::from_blob(m_OutputTensors[1].hostBuffer, {1, 256, height/8, width/8}, options);
    cv::Mat descriptors;
    getKeyPoints(mProb, thres, keypoints);
    computeDescriptors(mProb, mDesc, keypoints, descriptors);
}

void SuperPointTensorRT::getKeyPoints(const torch::Tensor & mProb, float threshold, std::vector<cv::Point2f> &keypoints)
{
    auto kpts = (mProb > threshold);
    kpts = torch::nonzero(kpts);  // [n_keypoints, 2]  (y, x)

    std::vector<cv::Point2f> keypoints_no_nms;
    for (int i = 0; i < kpts.size(0); i++) {
        keypoints_no_nms.push_back(cv::Point2f(kpts[i][1].item<float>(), kpts[i][0].item<float>()));
    }

    cv::Mat conf(keypoints_no_nms.size(), 1, CV_32F);
    for (size_t i = 0; i < keypoints_no_nms.size(); i++) {
        int x = keypoints_no_nms[i].x;
        int y = keypoints_no_nms[i].y;
        conf.at<float>(i, 0) = mProb[y][x].item<float>();
    }

    // cv::Mat descriptors;

    int border = 0;
    int dist_thresh = 4;
    NMS2(keypoints_no_nms, conf, keypoints, border, dist_thresh, width, height);
}


void SuperPointTensorRT::computeDescriptors(const torch::Tensor & mProb, const torch::Tensor & mDesc, const std::vector<cv::Point2f> &keypoints, cv::Mat &descriptors)
{
    cv::Mat kpt_mat(keypoints.size(), 2, CV_32F);  // [n_keypoints, 2]  (y, x)

    for (size_t i = 0; i < keypoints.size(); i++) {
        kpt_mat.at<float>(i, 0) = (float)keypoints[i].y;
        kpt_mat.at<float>(i, 1) = (float)keypoints[i].x;
    }

    auto fkpts = torch::from_blob(kpt_mat.data, {keypoints.size(), 2}, torch::kFloat);

    auto grid = torch::zeros({1, 1, fkpts.size(0), 2});  // [1, 1, n_keypoints, 2]
    grid[0][0].slice(1, 0, 1) = 2.0 * fkpts.slice(1, 1, 2) / mProb.size(1) - 1;  // x
    grid[0][0].slice(1, 1, 2) = 2.0 * fkpts.slice(1, 0, 1) / mProb.size(0) - 1;  // y

    auto desc = torch::grid_sampler(mDesc, grid, 0, 0, 0);
    desc = desc.squeeze(0).squeeze(1);

    // normalize to 1
    auto dn = torch::norm(desc, 2, 1);
    desc = desc.div(torch::unsqueeze(dn, 1));

    desc = desc.transpose(0, 1).contiguous();
    desc = desc.to(torch::kCPU);

    cv::Mat desc_mat(cv::Size(desc.size(1), desc.size(0)), CV_32FC1, desc.data<float>());

    descriptors = desc_mat.clone();
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

    for (int i = 0; i < pts_raw.size(); i++)
    {   
        int uu = (int) pts_raw[i].x;
        int vv = (int) pts_raw[i].y;

        grid.at<char>(vv, uu) = 1;
        inds.at<unsigned short>(vv, uu) = i;

        confidence.at<float>(vv, uu) = conf.at<float>(i, 0);
    }
    
    cv::copyMakeBorder(grid, grid, dist_thresh, dist_thresh, dist_thresh, dist_thresh, cv::BORDER_CONSTANT, 0);

    for (int i = 0; i < pts_raw.size(); i++)
    {   
        int uu = (int) pts_raw[i].x + dist_thresh;
        int vv = (int) pts_raw[i].y + dist_thresh;

        if (grid.at<char>(vv, uu) != 1)
            continue;

        for(int k = -dist_thresh; k < (dist_thresh+1); k++)
            for(int j = -dist_thresh; j < (dist_thresh+1); j++)
            {
                if(j==0 && k==0) continue;

                if ( confidence.at<float>(vv + k, uu + j) < confidence.at<float>(vv, uu) )
                    grid.at<char>(vv + k, uu + j) = 0;
                
            }
        grid.at<char>(vv, uu) = 2;
    }

    size_t valid_cnt = 0;

    for (int v = 0; v < (img_height + dist_thresh); v++){
        for (int u = 0; u < (img_width + dist_thresh); u++)
        {
            if (u -dist_thresh>= (img_width - border) || u-dist_thresh < border || v-dist_thresh >= (img_height - border) || v-dist_thresh < border)
            continue;

            if (grid.at<char>(v,u) == 2)
            {
                int select_ind = (int) inds.at<unsigned short>(v-dist_thresh, u-dist_thresh);
                cv::Point2f p = pts_raw[select_ind];
                pts.push_back(p);
                valid_cnt++;
            }
        }
    }
}