#include "superpoint_tensorrt.h"
#include "loop_defines.h"

//NMS code is from https://github.com/KinglittleQ/SuperPoint_SLAM

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
