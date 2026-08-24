#include <iostream>
#include <string>
#include <thread>
#include <atomic>
#include <csignal>
#include <unistd.h>
#include <sys/types.h>
#include <sys/wait.h>
#include <chrono>
#include <mqtt/async_client.h>

// -----------------------------------------------------------------------------
// Global flags & process IDs
// -----------------------------------------------------------------------------

std::atomic<bool> running(true);

pid_t recognizer       = -1;
pid_t fall_detector    = -1;
pid_t alert_sender     = -1;
pid_t report_generator = -1;
pid_t backup_process   = -1;   // backup.py 용 프로세스 PID

// -----------------------------------------------------------------------------
// Process management helpers
// -----------------------------------------------------------------------------

pid_t start_process(const std::string &cmd)
{
    pid_t pid = fork();
    if (pid == 0)
    {
        execl("/bin/bash", "bash", "-c", cmd.c_str(), (char *)nullptr);
        _exit(1); // execl 실패 시 자식 프로세스 종료
    }
    return pid;
}

void stop_process(pid_t &pid)
{
    if (pid > 0)
    {
        kill(pid, SIGTERM);
        waitpid(pid, nullptr, 0);
        pid = -1;
    }
}

void stop_all()
{
    stop_process(recognizer);
    stop_process(fall_detector);
    stop_process(alert_sender);
    stop_process(report_generator);
    stop_process(backup_process);   // backup 도 정리
}

// -----------------------------------------------------------------------------
// Start functions
// -----------------------------------------------------------------------------

void start_recognizer()
{
    stop_all();
    recognizer = start_process(
        "cd ~/project/mediapipe/raspberry_pi_gesture_recognizer && "
        "source ~/project/myenv/bin/activate && "
        "python3 recognize.py"
    );
}

void start_fall_detection()
{
    stop_all();

    fall_detector = start_process(
        "cd ~/project/hailo-rpi5-examples && source setup_env.sh && "
        "python3 smart-fall-detection/fall_detector_all_in_one.py --input usb"
    );

    alert_sender = start_process(
        "cd ~/project/hailo-rpi5-examples && source setup_env.sh && "
        "python3 smart-fall-detection/alert-sender.py --input usb"
    );
}

void start_leavework()
{
    // 최종 퇴근 확정:
    //  - 제스처 기반 모드 선택( recognizer ) 포함 전체 프로세스를 모두 중단
    //  - Ollama 기반 리포트 생성기(report_generator.py)만 실행
    stop_all();

    report_generator = start_process(
        "cd ~/project/hailo-rpi5-examples && "
        "source setup_env.sh && "
        "python3 smart-fall-detection/report_generator.py"
    );

    std::cout << "[LEAVEWORK] All processes stopped. Report generator started (Ollama report)." << std::endl;
}

void start_backup()
{
    // report 끝나고 돌리는 백업용. 이미 돌아가면 한 번 정리 후 다시 시작.
    stop_process(backup_process);

    backup_process = start_process(
        "cd ~/project/hailo-rpi5-examples && "
        "source setup_env.sh && "
        "python3 smart-fall-detection/backup.py"
    );

    std::cout << "[BACKUP] backup.py started." << std::endl;
}

void handle_button()
{
    // 초기 화면 상태로 리셋: 전부 멈추고 제스처 인식기만 실행
    stop_all();
    start_recognizer();
}

// -----------------------------------------------------------------------------
// MQTT callback: local subscribe + remote forward
// -----------------------------------------------------------------------------

// 브로커 주소 상수
const std::string LOCAL_BROKER  = "tcp://localhost:1883";
const std::string REMOTE_BROKER = "tcp://192.168.0.59:1883"; // 우분투 브로커

// remote 브로커로도 쏘기 위해 포인터를 들고 있는 콜백
class system_callback : public virtual mqtt::callback
{
public:
    explicit system_callback(mqtt::async_client* remote_client)
        : remote_client_(remote_client) {}

    void message_arrived(mqtt::const_message_ptr msg) override
    {
        std::string topic   = msg->get_topic();
        std::string payload = msg->to_string();

        std::cout << "[MQTT] message arrived: topic=" << topic
                  << ", payload=" << payload << std::endl;

        // 1) 로컬 시스템 제어
        if (topic == "/system/mode/gotowork")
        {
            // 근무 시작: 낙상 감지 + 알림 시작
            start_fall_detection();
        }
        else if (topic == "/system/mode/leavework")
        {
            // 근무 종료 확정:
            //  - mode select(제스처 인식) 포함 전체 종료
            //  - Ollama 리포트 생성 시작
            start_leavework();
        }
        else if (topic == "/system/button")
        {
            // 물리 버튼: 초기 화면(제스처 인식)으로
            handle_button();
        }
        else if (topic == "/system/report/job")
        {
            // report_generator.py 가 리포트 생성 & 아카이브 끝난 후
            // "done" 이라는 payload 를 보낸다고 가정
            if (payload == "done")
            {
                std::cout << "[MQTT] report job done → starting backup.py" << std::endl;
                start_backup();
            }
            else
            {
                std::cout << "[MQTT] /system/report/job payload ignored: " << payload << std::endl;
            }
        }
        else if (topic == "/system/backup")
        {
            // backup.py 가 모든 백업/정리 완료 후 "done" 발행
            if (payload == "done")
            {
                std::cout << "[MQTT] backup done → returning to idle UI" << std::endl;
                std::cout << "Have a good day!" << std::endl;

                // /system/button 을 받았을 때와 같은 환경으로 복귀
                handle_button();
            }
            else
            {
                std::cout << "[MQTT] /system/backup payload ignored: " << payload << std::endl;
            }
        }

        // 2) 같은 메시지를 원격 브로커에도 포워딩
        forward_to_remote(topic, payload);
    }

private:
    mqtt::async_client* remote_client_ {nullptr};

    void forward_to_remote(const std::string& topic, const std::string& payload)
    {
        if (!remote_client_)
            return;

        if (!remote_client_->is_connected())
        {
            std::cerr << "[WARN] Remote MQTT not connected, cannot forward: "
                      << topic << std::endl;
            return;
        }

        try
        {
            auto msg = mqtt::make_message(topic, payload);
            msg->set_qos(1);
            remote_client_->publish(msg);
        }
        catch (const mqtt::exception& e)
        {
            std::cerr << "[ERROR] Failed to forward MQTT to remote: "
                      << e.what() << std::endl;
        }
    }
};

// -----------------------------------------------------------------------------
// Key listener thread (q to quit)
// -----------------------------------------------------------------------------

void key_listener()
{
    while (running)
    {
        int c = getchar();
        if (c == 'q')
        {
            std::cout << "[EXIT] q pressed → stopping all processes…" << std::endl;
            running = false;
            stop_all();
            std::exit(0);
        }
    }
}

// -----------------------------------------------------------------------------
// Main
// -----------------------------------------------------------------------------

int main()
{
    std::cout << "[INIT] Starting Gesture Recognizer…" << std::endl;
    start_recognizer();

    std::cout << "[READY] Press q to exit." << std::endl;

    std::thread keyThread(key_listener);

    // 1) MQTT 클라이언트 두 개: 로컬용 + 원격용
    mqtt::async_client local_client(LOCAL_BROKER,  "controller_cpp_local");
    mqtt::async_client remote_client(REMOTE_BROKER, "controller_cpp_remote");

    // 콜백은 로컬 클라이언트에만 붙이고, 원격 클라이언트 포인터를 넘겨줌
    system_callback cb(&remote_client);
    local_client.set_callback(cb);

    try
    {
        // 2) 원격 브로커 먼저 연결 시도 (실패해도 로컬은 동작하게)
        try
        {
            std::cout << "[MQTT] Connecting to remote broker "
                      << REMOTE_BROKER << "…" << std::endl;
            remote_client.connect()->wait();
            std::cout << "[MQTT] Connected to remote broker." << std::endl;
        }
        catch (const mqtt::exception& e)
        {
            std::cerr << "[WARN] Could not connect to remote broker ("
                      << REMOTE_BROKER << "): " << e.what() << std::endl;
        }

        // 3) 로컬 브로커 연결 + 구독
        std::cout << "[MQTT] Connecting to local broker "
                  << LOCAL_BROKER << "…" << std::endl;
        local_client.connect()->wait();
        std::cout << "[MQTT] Connected to local broker." << std::endl;

        local_client.subscribe("/system/mode/gotowork", 1)->wait();
        local_client.subscribe("/system/mode/leavework", 1)->wait();
        local_client.subscribe("/system/button", 1)->wait();
        local_client.subscribe("/system/report/job", 1)->wait();
        local_client.subscribe("/system/backup", 1)->wait();

        std::cout << "[MQTT] Subscribed to system topics." << std::endl;
    }
    catch (const mqtt::exception& e)
    {
        std::cerr << "[FATAL] MQTT connection failed: " << e.what() << std::endl;
    }

    // 메인 루프
    while (running)
    {
        std::this_thread::sleep_for(std::chrono::milliseconds(200));
    }

    // 종료 처리
    try
    {
        if (local_client.is_connected())
            local_client.disconnect()->wait();
        if (remote_client.is_connected())
            remote_client.disconnect()->wait();
    }
    catch (const mqtt::exception& e)
    {
        std::cerr << "[WARN] MQTT disconnect error: " << e.what() << std::endl;
    }

    keyThread.join();
    stop_all();
    return 0;
}
