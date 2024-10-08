#include <lely/ev/loop.hpp>
#include <lely/io2/linux/can.hpp>
#include <lely/io2/posix/poll.hpp>
#include <lely/io2/sys/io.hpp>
#include <lely/io2/sys/sigset.hpp>
#include <lely/io2/sys/timer.hpp>
#include <lely/coapp/fiber_driver.hpp>
#include <lely/coapp/loop_driver.hpp>
#include <lely/coapp/master.hpp>
#include <iostream>
#include <cstdlib>
#include <thread>
#include <chrono>
#include <atomic>
#include <sstream>
#include <time.h>
#include "rclcpp/rclcpp.hpp"


using namespace lely;
using namespace std::chrono_literals;


class PD4Motor : public canopen::LoopDriver
{
public:
    using LoopDriver::LoopDriver;

    //PD4Motor(std::shared_ptr<ev::Executor> exec, std::shared_ptr<canopen::AsyncMaster> master, uint8_t id) :
    //    FiberDriver(exec, master, id){}

    ~PD4Motor(){}

    enum OperatingMode
    {
        AutoSetup = -2,
        ClockDirection = -1,
        NoMode = 0,
        ProfilePosition = 1,
        Velocity = 2,
        ProfileVelocity = 3,
        ProfileTorque = 4,
        Homing = 6,
        InterpolatedPosition = 7,
        CyclicSynchronousPosition = 8,
        CyclicSynchronousVelocity = 9,
        CyclicSynchronousTorque = 10
    };
    enum TransitionCommand
    {
        Shutdown = 0x06,
        SwitchOn = 0x07,
        DisableVoltage = 0x00,
        QuickStop = 0x02,
        DisableOperation = 0x07,
        EnableOperation = 0x0F,
        StartHoming = 0x1F
    };

    std::string name = "";
    double cmd = 0;
    double pos = 0;
    double vel = 0;

    void set_name(const std::string &wheel_name)
    {
      this->name = wheel_name;
    }

    

    template <typename T>
    T wait_(lely::canopen::SdoFuture<T>& fu, const char* type)
    {
        while(!fu.is_ready()){
            std::this_thread::sleep_for(20ms);
            //puts("not ready");
        }
        try{
            auto res = fu.get(); // throw an exception if the operation failed
            //std::cout <<"read successful" << std::endl;
            //RCLCPP_INFO(rclcpp::get_logger("DiffDriveCanOpenHardware"), "read value is %s %d", type, res.value());
            return res.value();
        } catch (const canopen::SdoError& e){
            std::cerr << "read failed" << e.what() << std::endl;
        } catch(...){
            puts("err");
        }
        return 1;
    }


    void set_transition(TransitionCommand cmd)
    {
        // Wait(AsyncWrite<uint16_t>(0x6040, 0, cmd));
        auto fu = AsyncWrite<uint16_t>(0x6040, 0, cmd);
        std::this_thread::sleep_for(20ms);
        //wait_(fu);
    }

    void set_mode(OperatingMode mode)
    {
        AsyncWrite<int8_t>(0x6060, 0, mode);
        std::this_thread::sleep_for(20ms);
    }



    void set_TargetVelocity(int16_t value)
    {
        //Wait(AsyncWrite<int16_t>(0x6042, 0, (int16_t)value));
        auto fu = AsyncWrite<int16_t>(0x6042, 0, (int16_t)value);
        std::this_thread::sleep_for(20ms);
    }

    int16_t get_VelocityActualValue()
    {
        lely::canopen::SdoFuture<int16_t> fu = AsyncRead<int16_t>(0x6044, 0, 20ms);
        int16_t vel_ = wait_<int16_t>(fu, "actual velocity");
        
        return vel_;
    }

    int32_t get_PositionActualValue()
    {
        lely::canopen::SdoFuture<int32_t> fu = AsyncRead<int32_t>(0x6064, 0, 20ms);
        int32_t pos_ = wait_<int32_t>(fu, "actual position");
        
        return pos_;
    }

private:

    void OnConfig(std::function<void(std::error_code ec)> res) noexcept override
    {
        RCLCPP_INFO(rclcpp::get_logger("DiffDriveCanOpenHardware"), "PD4E config!");

             //set the homing method
            set_mode(PD4Motor::OperatingMode::Homing);
            Wait(AsyncWrite<uint8_t>(0x6098, 0, 0x23)); // 35 decimal 0x23 Hexadecimal  // 37 decimal 0x25 Hexadecimal
            Wait(AsyncWrite<uint32_t>(0x607C, 0, 0x0));  // set position offest to 0
            
            set_transition(PD4Motor::TransitionCommand::Shutdown);
            set_transition(PD4Motor::TransitionCommand::SwitchOn);
            //set_transition(PD4Motor::TransitionCommand::EnableOperation);

            //start homing
            set_transition(PD4Motor::TransitionCommand::StartHoming);
 
    }

    void OnDeconfig(::std::function<void(::std::error_code ec)> /*res*/) noexcept override
    {
        puts("OnDeconfig");
    }

    void OnRpdoWrite(uint16_t idx, uint8_t /*subidx*/) noexcept override
    {
        // printf("idx: %d\n", idx);
        if (idx == 0x6044)
        {
        RCLCPP_INFO(
        rclcpp::get_logger("DiffDriveCanOpenHardware"),"TPDO VelocityActualValue: %d", get_VelocityActualValue());
        } 
        else if (idx == 0x6064)
        {
        RCLCPP_INFO(
        rclcpp::get_logger("DiffDriveCanOpenHardware"),"TPDO PositionActualValue: %d", get_PositionActualValue());
        }
    }
};


class CanOpenDriver
{
public:

    CanOpenDriver() = default;

    ~CanOpenDriver(){
        stop_thread();

        delete ctx;
        delete poll;
        delete loop;
        delete timer;
        delete ctrl;
        delete chan;
    }

    std::shared_ptr<canopen::AsyncMaster> master_;
    std::shared_ptr<ev::Executor> exec;
    std::shared_ptr<PD4Motor> wheel_l_;
    std::shared_ptr<PD4Motor> wheel_r_;

    //TODO: finish this function according to the latest Panther link
    // void set_config(const std::string &can_interface_name, const std::string &file_path, const int &motor_1_id, const int &motor_2_id)
    // {

    // }

    void initialize() {
        ctx = new io::Context();
        poll = new io::Poll(*ctx);
        loop = new ev::Loop(poll->get_poll());
        exec = std::make_shared<ev::Executor>(loop->get_executor());
        timer = new io::Timer(*poll, *exec, CLOCK_MONOTONIC);
        ctrl = new io::CanController("can0");
        chan = new io::CanChannel(*poll, *exec);
        chan->open(*ctrl);
        master_ = std::make_shared<canopen::AsyncMaster>(*timer, *chan, "/ros2_ws/install/diffdrive_canopen/include/diffdrive_canopen/diffdrive_canopen/master.dcf", "", 1);
        master_->Reset();
        wheel_l_ = std::make_shared<PD4Motor>(*master_, 2);
        wheel_r_ = std::make_shared<PD4Motor>(*master_, 3);
        //pass_ = true;
        puts("Starting loop");
        spinner = std::thread(std::bind(&CanOpenDriver::start_thread, this));
    }

    void start_thread() {
        this->loop->run();
        puts("Stopping loop");
    }

    void stop_thread() {
        puts("Stop thread");
        puts("Shutting motors down...");
        loop->stop();
        if (spinner.joinable()) {
            spinner.join();
        }
        ctx->shutdown();
        puts("Stopped the thread");

    }

    // void set_this_op() {
    //       //this->motor1_->AsyncWrite<int16_t>(0x6042, 0, 300);
    //       this->master_->AsyncWrite(this->master_->GetExecutor(),2, 0x6042, 0, 300);
    // }



private:
    io::Context *ctx;
    io::Poll *poll;
    ev::Loop *loop;
    //ev::Executor *exec;
    io::Timer *timer;
    io::CanController *ctrl;
    io::CanChannel *chan;
    //canopen::AsyncMaster *master_;
    std::thread spinner;
    std::string canopen_interface_name = "can0";
    std::string file_path = "/ros2_ws/install/diffdrive_canopen/include/diffdrive_canopen/diffdrive_canopen/master.dcf";
    
};

// int main() // .h of diffbot
// {
//     CanOpenDriver comms_;

//     comms_.motor1_->AsyncWrite<int16_t>(0x6042, 0, 300);
//     puts("sleeping main");
//     std::this_thread::sleep_for(5000ms);
//     std::this_thread::sleep_for(std::chrono::seconds(4));
//     return EXIT_SUCCESS;
// }