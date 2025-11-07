#pragma once
#include <future>
#include <thread>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>
#include <chrono>
#include <cstdint>
#include <cstring>
#include <cstdio>
#include <sys/socket.h>
#include <sys/un.h>
#include <fcntl.h>
#include <unistd.h>

#include <opencv2/opencv.hpp>

#include <post_process/yolo_post_processing.hpp>

#include <dxrt/dxrt_api.h>
#include <common/objects.hpp>
#include <utils/videostream.hpp>
#include <app_parser.hpp>

// ===== ByteTrack 추가 =====
#include <ByteTrack/BYTETracker.h>

// ---------- UDS header & utils ----------
#pragma pack(push,1)
struct DxHdr {
  uint32_t magic;      // 0xABCD1204
  uint16_t version;    // 2 (V2: track_id 포함)
  uint16_t flags;      // 0
  uint64_t seq_id;
  int64_t  stamp_nsec; // epoch ns (system_clock)
  uint32_t img_w, img_h;
  uint32_t count;      // objects
  uint32_t reserved;   // 0
};

// V2: 추적 ID 포함
struct DxObjV2 {
  float    x, y, w, h;   // 좌상단 + w,h (dstSize 픽셀 기준)
  float    score;        // track score (또는 det score)
  uint32_t label;        // class id (없으면 0xFFFFFFFF)
  uint32_t track_id;     // 추적 ID (없으면 0xFFFFFFFF)
};
#pragma pack(pop)

static inline int64_t now_epoch_ns() {
  using namespace std::chrono;
  return duration_cast<nanoseconds>(system_clock::now().time_since_epoch()).count();
}

// 비연결 UDS 송신 헬퍼
struct DxUds {
  int sock{-1};
  sockaddr_un addr{};

  void open_sender(const char* path) {
    if (sock >= 0) return;
    sock = ::socket(AF_UNIX, SOCK_DGRAM | SOCK_NONBLOCK, 0);
    addr.sun_family = AF_UNIX;
    std::snprintf(addr.sun_path, sizeof(addr.sun_path), "%s", path);
  }

  inline void send(const void* buf, size_t len) {
    if (sock < 0) return;
    (void)::sendto(sock, buf, len, 0,
                   reinterpret_cast<sockaddr*>(&addr),
                   sizeof(addr));
  }
};


class DetectorApp
{
public :
    DetectorApp(std::shared_ptr<dxrt::InferenceEngine> inferenceEngine, AppSourceInfo sourceInfo, AppInputFormat input_format, AppType app_type, dxapp::yolo::Params& params, int channel, dxapp::common::Point position, dxapp::common::Size dst_size)
            : _inferenceEngine(inferenceEngine), _profiler(dxrt::Profiler::GetInstance()), _channel(channel + 1), _inputFormat(input_format), _appType(app_type), _params(params), _position(position), _dstSize(dst_size)
    {
        _videoPath = sourceInfo.inputPath;
        _inputType = sourceInfo.inputType;
        _name = "app" + std::to_string(_channel);
        _outName = _name + "_" + dxapp::common::getFileName(_videoPath);
        _processName = "proc_"+_name;
        _inferName = "infer_"+_name;

        _inputSize = _params._input_size;

        _vStream = VideoStream(_inputType, _videoPath, sourceInfo.numOfFrames, _inputSize, _inputFormat, _dstSize, _inferenceEngine);
        _srcSize = _vStream._srcSize;
        _postProcessing = dxapp::yolo::PostProcessing(_params, _inputSize, _srcSize, _dstSize);
        _resultFrame = cv::Mat(_dstSize._height, _dstSize._width, CV_8UC3, cv::Scalar(0, 0, 0));
        _frame_count = 0;
        _processed_count = 0;
        _fps_time_s = std::chrono::high_resolution_clock::now();
        _fps_time_e = std::chrono::high_resolution_clock::now();

        _outputBufferSize = _inferenceEngine->GetOutputSize();
        _bufferPoolSize = 5;
        _currentBufferIndex = 0;
        for (int i = 0; i < _bufferPoolSize; ++i) {
            _bufferPool.push_back(std::vector<uint8_t>(_outputBufferSize));
        }
        detSock.open_sender("/tmp/dx_det.sock");            // 단일 V2 전송
        prevSock.open_sender("/tmp/dx_det_preview.sock");   // 프리뷰 전송
        lastPreviewSend = std::chrono::steady_clock::now();

    };

    void runPostProcess(dxrt::TensorPtrs outputs)
    {
        std::unique_lock<std::mutex> _uniqueLock(_lock);
        _postProcessing.run(outputs);
        _processed_count += 1;
        _fps_time_e = std::chrono::high_resolution_clock::now();
        _processTime = std::chrono::duration_cast<std::chrono::microseconds>(_fps_time_e - _fps_time_s).count();

        _currentOutputBuffer = nullptr;
    };

    cv::Mat getResultFrame()
    {
        std::unique_lock<std::mutex> _uniqueLock(_getFrameLock);
        return _resultFrame;
    };

    dxapp::common::Point Position()
    {
        return _position;
    };

    dxapp::common::Size Resolution()
    {
        return _dstSize;
    };

    void makeThread()
    {
        _thread = std::thread(&DetectorApp::ppThreadFunction, this);
    };

    void notify_all()
    {
        _wait.store(false);
        std::cout << "[" << _name << "] : notify to this thread function. "<<std::endl;
        _cv.notify_all();
    };

    void joinThread()
    {
        while(!_thread.joinable())
        {
            std::this_thread::sleep_for(std::chrono::microseconds(10));
        }
        _thread.join();
    };

    void quitThread()
    {
        _quit.store(true);
    };

    bool quit()
    {
        return _quit.load();
    };

    void ppThreadFunction()
    {
        void* inf_data = nullptr;
        std::condition_variable _internal_cv;
        std::cout << "[" << _name << "] : entered post process thread function. "<<std::endl;
        {
            std::unique_lock<std::mutex> _uniqueLock(_lock);
            _cv.wait(_uniqueLock, [&](){return !_wait.load();});
        }
        cv::Mat outputImg = cv::Mat(_dstSize._height, _dstSize._width, CV_8UC3, cv::Scalar(0, 0, 0));

        while(!_quit.load())
        {
            std::vector<uint8_t>* outputBuffer = nullptr;
            {
                std::unique_lock<std::mutex> poolLock(_bufferPoolMutex);
                outputBuffer = &_bufferPool[_currentBufferIndex];
                _currentBufferIndex = (_currentBufferIndex + 1) % _bufferPoolSize; // Circular access
            }

            if(!_quit.load())
            {
                inf_data = _vStream.GetInputStream();
                {
                    std::unique_lock<std::mutex> _uniqueLock(_lock);
                    _get_frame_result = _internal_cv.wait_for(_uniqueLock, std::chrono::seconds(3), [&](){ return inf_data!=nullptr; });
                    if(!_get_frame_result)
                    {
                        std::cout << "[DX-APP ERROR] The camera capture frame did not arrive properly." << std::endl;
                        std::cout << "[DX-APP ERROR] Shutting down the yolo_multi application." << std::endl;
                        _quit.store(true);
                        break;
                    }
                    else if(inf_data == nullptr)
                        continue;
                }
                _fps_time_s = std::chrono::high_resolution_clock::now();
                _profiler.Start(_inferName);

                _currentOutputBuffer = outputBuffer;
                std::ignore = _inferenceEngine->RunAsync(inf_data, (void*)this, (void*)outputBuffer->data());
                _frame_count += 1;
            }
            if(_processed_count > 0)
            {
                std::unique_lock<std::mutex> _uniqueLock(_lock);
                dxapp::common::DetectObject results = _postProcessing.getResult();

                // ---- (1) Detection → ByteTrack 입력 변환
                std::vector<byte_track::Object> bt_in;
                bt_in.reserve(results._detections.size());
                for (const auto& d : results._detections) {
                    byte_track::Rect<float> rr(d._bbox._xmin, d._bbox._ymin,
                                               d._bbox._width, d._bbox._height);
                    int   cls = static_cast<int>(d._classId);
                    float sc  = d._conf;
                    bt_in.emplace_back(rr, cls, sc);
                }

                // ---- (2) ByteTrack 업데이트
                auto tracks = _tracker.update(bt_in);

                // ---- (3) 단일 UDS(V2) 전송: track_id 포함
                sendObjectsUDS_V2(tracks, results, _frame_count, _dstSize._width, _dstSize._height);

                // ---- (4) 프리뷰 이미지: 디텍션 박스 + Track ID 텍스트 오버레이
                outputImg = _vStream.GetOutputStream(results);

                // Track ID 그리기 (tlwh 기준)
                for (const auto& t : tracks) {
                    if (!t->isActivated()) continue;
                    const auto& r = t->getRect();
                    int x = static_cast<int>(r.x());
                    int y = static_cast<int>(r.y());
                    int w = static_cast<int>(r.width());
                    int h = static_cast<int>(r.height());
                    int tid = static_cast<int>(t->getTrackId());

                    // 박스 라인(초록)
                    cv::rectangle(outputImg, cv::Rect(x,y,w,h), cv::Scalar(0,255,0), 2);

                    // 라벨 텍스트: "#<track_id>"
                    std::string tag = "#" + std::to_string(tid);
                    int base = 0;
                    const int font = cv::FONT_HERSHEY_SIMPLEX;
                    const double scale = 0.6;
                    const int thickness = 2;
                    cv::Size ts = cv::getTextSize(tag, font, scale, thickness, &base);

                    // 텍스트 배경(반투명 검정 박스)
                    int tx = std::max(0, x);
                    int ty = std::max(0, y - ts.height - 4);
                    cv::rectangle(outputImg,
                                  cv::Rect(tx, std::max(0, ty), ts.width + 8, ts.height + 6),
                                  cv::Scalar(0,0,0), cv::FILLED);
                    // 텍스트(초록)
                    cv::putText(outputImg, tag, cv::Point(tx + 4, ty + ts.height + 1),
                                font, scale, cv::Scalar(0,255,0), thickness, cv::LINE_AA);
                }

                // ---- (5) 프리뷰 JPEG 전송 (Throttle)
                sendPreviewJPEG_Throttled(outputImg, _frame_count, _dstSize._width, _dstSize._height);

                int64_t new_average = ((_fps_previous_average_time * _processed_count) + _processTime) / (_processed_count + 1);
                int64_t fps = 1000000 / new_average;
                _fps_previous_average_time = new_average;
                std::string fpsCaption = "FPS : " + std::to_string((int)fps);
                (void)fpsCaption; // 필요시 영상에 찍어 사용

                if(_appType == NONE)
                    std::cout << results <<std::endl;
                {
                    std::unique_lock<std::mutex> _result_frameLock(_getFrameLock);
                    _resultFrame = outputImg;
                    _result_frame_count++;
                }
            }
            _inferTime = _inferenceEngine->GetLatency();
            std::this_thread::sleep_for(std::chrono::microseconds(1));
        };
        while(_frame_count != _processed_count)
        {
            std::this_thread::sleep_for(std::chrono::microseconds(1));
        }
        _vStream.Destructor();

    };

    int save()
    {
        while(_result_frame_count < 1)
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        std::unique_lock<std::mutex> _uniqueLock(_lock);
        std::string file_name = dxapp::common::getFileName(_videoPath);
        if(cv::imwrite("./result-" + _name + ".jpg", _resultFrame))
        {
            std::cout << "save file : result-" << _name << ".jpg" <<std::endl;
            return 0;
        }
        else
            return -1;
    };

    uint8_t* get_outputMem(){
        return nullptr;
    };

    ~DetectorApp(){
        std::unique_lock<std::mutex> poolLock(_bufferPoolMutex);
        _bufferPool.clear();
    };

private:
    // ---- Objects(V2: track 포함) UDS 전송 ----
    void sendObjectsUDS_V2(const std::vector<byte_track::BYTETracker::STrackPtr>& tracks,
                           const dxapp::common::DetectObject& dets_for_label_fallback,
                           uint64_t seq_id, int img_w, int img_h)
    {
        if (detSock.sock < 0) return;

        DxHdr H{};
        H.magic = 0xABCD1204;
        H.version = 2;
        H.flags = 0;
        H.seq_id = seq_id;
        H.stamp_nsec = now_epoch_ns();
        H.img_w = static_cast<uint32_t>(img_w);
        H.img_h = static_cast<uint32_t>(img_h);
        H.count = static_cast<uint32_t>(tracks.size());
        H.reserved = 0;

        std::vector<uint8_t> buf(sizeof(DxHdr) + sizeof(DxObjV2) * tracks.size());
        std::memcpy(buf.data(), &H, sizeof(DxHdr));
        auto* O = reinterpret_cast<DxObjV2*>(buf.data() + sizeof(DxHdr));

        // (옵션) det기반 라벨/점수 보강 — 필요 없으면 이 블록 통째로 제거 가능
        const auto& dets = dets_for_label_fallback._detections;
        auto iou = [](float ax, float ay, float aw, float ah,
                      float bx, float by, float bw, float bh)->float {
            float ax2=ax+aw, ay2=ay+ah, bx2=bx+bw, by2=by+bh;
            float ix = std::max(ax, bx);
            float iy = std::max(ay, by);
            float ix2= std::min(ax2, bx2);
            float iy2= std::min(ay2, by2);
            float iw = std::max(0.0f, ix2-ix);
            float ih = std::max(0.0f, iy2-iy);
            float inter = iw*ih;
            float areaA = aw*ah, areaB = bw*bh;
            float uni = areaA + areaB - inter + 1e-6f;
            return inter/uni;
        };

        for (size_t i = 0; i < tracks.size(); ++i)
        {
            const auto& t = tracks[i];
            const auto& r = t->getRect(); // tlwh

            DxObjV2 v{};
            v.x = r.x();
            v.y = r.y();
            v.w = r.width();
            v.h = r.height();

            v.score    = t->getScore(); // 기본 track score
            v.label    = 0xFFFFFFFFu;   // 기본 비움
            v.track_id = static_cast<uint32_t>(t->getTrackId());

            float best_iou = 0.0f; int best_cls = -1; float best_conf = 0.0f;
            for (const auto& d : dets)
            {
                float iouv = iou(v.x, v.y, v.w, v.h,
                                 d._bbox._xmin, d._bbox._ymin, d._bbox._width, d._bbox._height);
                if (iouv > best_iou) { best_iou = iouv; best_cls = d._classId; best_conf = d._conf; }
            }
            if (best_iou >= 0.3f && best_cls >= 0)
            {
                v.label = static_cast<uint32_t>(best_cls);
                // v.score = best_conf; // 라벨 신뢰도를 쓰고 싶으면 활성화
            }

            O[i] = v;
        }

        detSock.send(buf.data(), buf.size());
    }

    // ---- Preview JPEG 전송 (Throttle) ----
    void sendPreviewJPEG_Throttled(const cv::Mat& bgr, uint64_t seq_id, int img_w, int img_h)
    {
        if (prevSock.sock < 0) return;

        const int kPreviewHz = 10;
        const int kJpegQuality = 60;

        auto now = std::chrono::steady_clock::now();
        auto diff = std::chrono::duration_cast<std::chrono::milliseconds>(now - lastPreviewSend).count();
        if (diff < (1000 / kPreviewHz)) return;
        lastPreviewSend = now;

        std::vector<uchar> jpg;
        std::vector<int> params = {cv::IMWRITE_JPEG_QUALITY, kJpegQuality};
        if (!cv::imencode(".jpg", bgr, jpg, params)) return;

        DxHdr H{};
        H.magic = 0xABCD1204;
        H.version = 2; // 프리뷰도 V2 헤더 재사용 (count=0)
        H.flags = 0;
        H.seq_id = seq_id;
        H.stamp_nsec = now_epoch_ns();
        H.img_w = static_cast<uint32_t>(img_w);
        H.img_h = static_cast<uint32_t>(img_h);
        H.count = 0;
        H.reserved = 0;

        uint32_t jpeg_size = static_cast<uint32_t>(jpg.size());
        std::vector<uint8_t> buf(sizeof(DxHdr) + sizeof(uint32_t) + jpeg_size);

        std::memcpy(buf.data(), &H, sizeof(DxHdr));
        std::memcpy(buf.data() + sizeof(DxHdr), &jpeg_size, sizeof(uint32_t));
        std::memcpy(buf.data() + sizeof(DxHdr) + sizeof(uint32_t), jpg.data(), jpeg_size);

        prevSock.send(buf.data(), buf.size());
    }

    std::shared_ptr<dxrt::InferenceEngine> _inferenceEngine;
    dxrt::Profiler &_profiler;

    std::string _videoPath;
    std::string _name;
    std::string _outName;
    int _channel;
    int _numOfFrames;
    AppInputType _inputType;
    AppInputFormat _inputFormat;
    AppType _appType;
    dxapp::yolo::Params &_params;

    VideoStream _vStream;

    dxapp::common::Size _srcSize;
    dxapp::common::Size _inputSize;
    dxapp::common::Point _position;
    dxapp::common::Size _dstSize;

    cv::Mat _resultFrame;

    dxapp::yolo::PostProcessing _postProcessing;

    std::string _processName;
    std::string _inferName;
    uint64_t _inferTime = 0;
    int64_t _processTime = 0;
    int64_t _fps_previous_average_time = 0;

    std::thread _thread;
    std::thread _ppThread;
    std::mutex _lock;
    std::mutex _getFrameLock;
    std::condition_variable _cv;
    std::atomic<bool> _wait = {true};
    std::atomic<bool> _quit = {false};

    unsigned long long _processed_count = 0;
    unsigned long long _frame_count = 0;
    unsigned long long _result_frame_count = 0;

    std::chrono::high_resolution_clock::time_point _fps_time_s;
    std::chrono::high_resolution_clock::time_point _fps_time_e;

    bool _get_frame_result = true;

    // Circular buffer pool for dynamic memory management
    std::vector<std::vector<uint8_t>> _bufferPool;
    std::mutex _bufferPoolMutex;
    size_t _outputBufferSize;
    size_t _bufferPoolSize;
    size_t _currentBufferIndex; // Index for circular buffer access
    std::vector<uint8_t>* _currentOutputBuffer; // Pointer to current buffer

    // 단일 전송 소켓
    DxUds detSock;     // /tmp/dx_det.sock : V2(Object+track_id)
    DxUds prevSock;    // /tmp/dx_det_preview.sock : JPEG 프리뷰
    std::chrono::steady_clock::time_point lastPreviewSend;

    // ===== ByteTrack 멤버 =====
    // fps=30, track_buffer=30, track_thresh=0.10, high_thresh=0.25, match=0.80
    byte_track::BYTETracker _tracker{30, 30, 0.10f, 0.25f, 0.80f};
};

class Detector
{
public:
    Detector(const dxapp::AppConfig &_config):config(_config)
    {
        bool is_valid = dxapp::validationJsonSchema(config.modelInfo.c_str(), modelInfoSchema);
        if(!is_valid)
        {
            std::cout << config.modelInfo << std::endl;
            throw std::invalid_argument("model params is invalid parsing");
        }

        readModelInfo(config.modelInfo.c_str());
        auto inferenceOption = dxrt::InferenceOption();
        inferenceEngine = std::make_shared<dxrt::InferenceEngine>(modelPath, inferenceOption);
        if(!dxapp::common::minversionforRTandCompiler(inferenceEngine.get()))
        {
            throw std::runtime_error("[DXAPP] [ER] The version of the compiled model is not compatible with the version of the runtime. Please compile the model again.");
        }
        if(params._layers.empty())
        {
            if(!ORT_OPTION_DEFAULT)
            {
                throw std::invalid_argument("[DXAPP] [ER] Layer information is missing. This is only supported when USE_ORT=ON. Please modify and rebuild.");
            }
        }
        auto outputDataInfo = inferenceEngine->GetOutputs();
        for(auto &info:outputDataInfo)
        {
            outputShape.emplace_back(info.shape());
        }

        params._classes = config.classes;
        params._numOfClasses = config.numOfClasses;
        params._outputShape = outputShape;
        for(int i = 0; i < (int)params._final_outputs.size(); i++)
        {
            for(int j = 0; j < (int)inferenceEngine->GetOutputTensorNames().size(); j++)
            {
                if(inferenceEngine->GetOutputTensorNames()[j] == params._final_outputs[i])
                {
                    params._outputTensorIndexMap.emplace_back(std::make_pair(i, j));
                    break;
                }
            }
        }
        if(params._outputTensorIndexMap.size() > 0)
            params._is_onnx_output = true;

        if(!params._is_onnx_output)
        {
            for(int i = 0; i < (int)params._layers.size(); i++)
            {
                for(int j = 0; j < (int)inferenceEngine->GetOutputTensorNames().size(); j++)
                {
                    if(inferenceEngine->GetOutputTensorNames()[j] == params._layers[i].name)
                    {
                        params._outputTensorIndexMap.emplace_back(std::make_pair(i, j));
                        break;
                    }
                }
            }
        }
        if(!params._is_onnx_output && params._outputTensorIndexMap.size() < inferenceEngine->GetOutputTensorNames().size())
        {
            throw std::invalid_argument("[DXAPP] [ER] output tensor index list is not enough. Please check the model output configuration and the output tensor names.");
        }

        size_t all_image_count = 0;
        for(auto const& source_info : config.sourcesInfo)
        {
            if(source_info.inputType == AppInputType::IMAGE)
                all_image_count++;
        }
        if(all_image_count == config.sourcesInfo.size())
            is_all_image = true;

        int div = dxapp::common::divideBoard(config.sourcesInfo.size());

        for(int i=0;i<(int)config.sourcesInfo.size();i++)
        {
            dxapp::common::Size dstSize = dxapp::common::Size(config.videoOutResolution._width/div, config.videoOutResolution._height/div);
            dxapp::common::Point dstPosition((config.videoOutResolution._width/div)*(i%div), (config.videoOutResolution._height/div)*(i/div));
            apps.emplace_back(std::make_shared<DetectorApp>(inferenceEngine, config.sourcesInfo[i], config.inputFormat, config.appType,
                                    params, i, dstPosition, dstSize));
        }
        std::function<int(dxrt::TensorPtrs, void*)> postProcCallBack = \
        [&](dxrt::TensorPtrs outputs, void* arg)
        {
            DetectorApp* app = (DetectorApp*)arg;
            app->runPostProcess(outputs);
            return 0;
        };
        inferenceEngine->RegisterCallback(postProcCallBack);

        resultView = cv::Mat(config.videoOutResolution._height, config.videoOutResolution._width, CV_8UC3, cv::Scalar(0, 0, 0));
    };

    ~Detector(void)
    {
        if(config.appType == OFFLINE)
        {
            if(saveThread.joinable())
                saveThread.join();
        }
    };

    void makeThread()
    {
        for(int i = 0;i<(int)apps.size();i++){
            apps[i]->makeThread();
        }
        if(config.appType == OFFLINE)
        {
            if(is_all_image)
            {
                std::cout << "[result save mode] ./xxx.jpg \n"
                        << "Create Thread to save jpg " << std::endl;
                saveThread = std::thread(&Detector::saveImage, this);
            }
            else
            {
                std::cout << "[result save mode] ./result.avi \n"
                        << "Create Thread to save avi " << std::endl;
                saveThread = std::thread(&Detector::saveResult, this);
            }
        }
    };

    void startThread()
    {
        for(int i=0;i<(int)apps.size();i++){
            apps[i]->notify_all();
        }
        if(config.appType == OFFLINE)
        {
            _wait.store(false);
            cv.notify_all();
        }
    };

    void joinThread()
    {
        for(int i=0;i<(int)apps.size();i++){
            apps[i]->joinThread();
        }
        if(config.appType == OFFLINE)
        {
            saveThread.join();
        }
    };

    void quitThread()
    {
        for(int i=0;i<(int)apps.size();i++){
            apps[i]->quitThread();
        }
    };

    bool status()
    {
        for(int i=0;i<(int)apps.size();i++){
            if(!apps[i]->quit())
            {
                return true;
            }
        }
        return false;
    };

    cv::Mat totalView()
    {
        for(int i=0; i<(int)apps.size();i++)
        {
            apps[i]->getResultFrame().copyTo(resultView(cv::Rect(apps[i]->Position()._x, apps[i]->Position()._y, apps[i]->Resolution()._width, apps[i]->Resolution()._height)));
        }
        return resultView;
    };

    void saveResult()
    {
        {
            std::unique_lock<std::mutex> _uniqueLock(lock);
            cv.wait(_uniqueLock, [&](){return !_wait.load();});
        }
        std::cout << "start Save Thread " << std::endl;
        std::cout << "Press 'q' key to quit " << std::endl;
        std::string saveFileName = "result.avi";
        cv::VideoWriter writer;
        writer.open(saveFileName, cv::VideoWriter::fourcc('M', 'J', 'P','G'), 30,
                    cv::Size(config.videoOutResolution._width, config.videoOutResolution._height), true);
        if(!writer.isOpened())
        {
            std::cout << "Error : video writer for result.mp4 could not be opened. " << std::endl;
            return;
        }

        while(true)
        {
            writer << totalView();
            std::this_thread::sleep_for(std::chrono::microseconds(30000));
            if (!status()){
                std::cout << "quit save Thread "<< std::endl;
                break;
            }
        }
    };

    void saveImage()
    {
        {
            std::unique_lock<std::mutex> _uniqueLock(lock);
            cv.wait(_uniqueLock, [&](){return !_wait.load();});
        }
        std::this_thread::sleep_for(std::chrono::microseconds(100000));
        for(int idx = 0; idx < static_cast<int>(apps.size()); idx++)
        {
            int processed_complete = 0;
            for(int idx = 0; idx < static_cast<int>(apps.size()); idx++)
            {
                auto ret = apps[idx]->save();
                if(ret < 0){
                    idx--;
                    continue;
                }
                processed_complete++;
            }
            if(processed_complete == static_cast<int>(apps.size()))
            {
                quitThread();
                break;
            }
        }
        quitThread();
    };

    void readModelInfo(const char* modelInfo)
    {
        rapidjson::Document doc;
        doc.Parse(modelInfo);
        modelPath = doc["path"].GetString();
        auto modelParam = doc["param"].GetObject();
        params._objectness_threshold = modelParam.HasMember("objectness_threshold")?modelParam["objectness_threshold"].GetFloat():0.25f;
        params._score_threshold = modelParam["score_threshold"].GetFloat();
        params._iou_threshold = modelParam["iou_threshold"].GetFloat();
        params._kpt_count = 0;
        params._input_size = dxapp::common::Size(modelParam["input_width"].GetInt(), modelParam["input_height"].GetInt());
        for(auto &output : modelParam["final_outputs"].GetArray())
        {
            params._final_outputs.push_back(output.GetString());
        }

        std::string read = "";
        read = modelParam.HasMember("last_activation")?modelParam["last_activation"].GetString():"";
        if(read=="sigmoid")
            params._last_activation = dxapp::yolo::sigmoid;
        else if(read=="exp")
            params._last_activation = dxapp::yolo::exp;
        else
            params._last_activation = [](float x){return x;};

        read = modelParam["decoding_method"].GetString();
        if(read=="yolox")
            params._decode_method = dxapp::yolo::Decode::YOLOX;
        else if(read=="yolo_scale")
            params._decode_method = dxapp::yolo::Decode::YOLOSCALE;
        else if(read=="yolo_pose")
        {
            params._decode_method = dxapp::yolo::Decode::YOLO_POSE;
            params._kpt_count = modelParam["kpt_count"].GetInt();
        }
        else if(read=="yolo_face")
        {
            params._decode_method = dxapp::yolo::Decode::YOLO_FACE;
            params._kpt_count = modelParam["kpt_count"].GetInt();
        }
        else if(read=="scrfd")
        {
            params._decode_method = dxapp::yolo::Decode::SCRFD;
            params._kpt_count = modelParam["kpt_count"].GetInt();
        }
        else if(read=="yolov8")
            params._decode_method = dxapp::yolo::Decode::YOLOV8;
        else if(read=="yolov9")
            params._decode_method = dxapp::yolo::Decode::YOLOV9;
        else if(read=="custom_decode")
            params._decode_method = dxapp::yolo::Decode::CUSTOM_DECODE;
        else
            params._decode_method = dxapp::yolo::Decode::YOLO_BASIC;

        read = modelParam["box_format"].GetString();
        if(read=="corner")
            params._box_format = dxapp::yolo::BBoxFormat::XYX2Y2;
        else
            params._box_format = dxapp::yolo::BBoxFormat::CXCYWH;

        for(auto &d:modelParam["layer"].GetArray()){
            auto o = d.GetObject();
            dxapp::yolo::Layers l = {};
            l.name = o["name"].GetString();
            l.stride = o.HasMember("stride")? o["stride"].GetInt() : 0;
            l.scale = o.HasMember("scale")? o["scale"].GetFloat() : 0;
            if(o.HasMember("shape"))
            {
                for(auto &s:o["shape"].GetArray()){
                    l.shape.emplace_back(s.GetInt());
                }
            }
            if(o.HasMember("anchor_width"))
            {
                for(auto &w:o["anchor_width"].GetArray()){
                    l.anchor_width.emplace_back(w.GetFloat());
                }
            }
            if(o.HasMember("anchor_height"))
            {
                for(auto &h:o["anchor_height"].GetArray()){
                    l.anchor_height.emplace_back(h.GetFloat());
                }
            }
            params._layers.emplace_back(l);
        }
    };

    dxapp::AppConfig config;
    std::vector<std::vector<int64_t>> outputShape;
    bool is_all_image = false;

    std::shared_ptr<dxrt::InferenceEngine> inferenceEngine;
    dxapp::yolo::Params params;
    std::string modelPath;

    std::vector<std::shared_ptr<DetectorApp>> apps;

    cv::Mat resultView;
    std::thread saveThread;
    std::mutex lock;
    std::condition_variable cv;
    std::atomic<bool> _wait = {true};
private:
    const char* modelInfoSchema = R"""(
            {
                "type": "object",
                "properties":{
                    "path": {
                        "type": "string"
                    },
                    "param": {
                        "type": "object",
                        "properties": {
                            "input_width": {
                                "type": "number"
                            },
                            "input_height": {
                                "type": "number"
                            },
                            "score_threshold": {
                                "type": "number"
                            },
                            "iou_threshold": {
                                "type": "number"
                            },
                            "last_activation": {
                                "type": "string"
                            },
                            "decoding_method": {
                                "type": "string"
                            },
                            "box_format": {
                                "type": "string",
                                "enum": ["corner", "center"]
                            },
                            "kpt_order": {
                                "type": "string"
                            },
                            "kpt_count": {
                                "type": "number"
                            },
                            "final_outputs":{
                                "type": "array",
                                "items": "string"
                            },
                            "layer": {
                                "type": "array",
                                "items":{
                                    "type": "object",
                                    "properties":{
                                        "name":{
                                            "type": "string"
                                        },
                                        "stride":{
                                            "type": "number"
                                        },
                                        "anchor_width":{
                                            "type": "array",
                                            "items": "number"
                                        },
                                        "anchor_height":{
                                            "type": "array",
                                            "items": "number"
                                        },
                                        "scale":{
                                            "type": "number"
                                        },
                                        "shape":{
                                            "type": "array",
                                            "items": "number"
                                        }
                                    },
                                    "required": [
                                        "name"
                                    ]
                                }
                            }
                        },
                        "required": [
                            "input_width", "input_height", "score_threshold", "iou_threshold", "decoding_method", "box_format"
                        ]
                    }
                },
                "required":[
                    "path", "param"
                ]
            }
        )""";

};
