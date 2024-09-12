#include <lely/ev/loop.hpp>
#include <lely/io2/linux/can.hpp>
#include <lely/io2/posix/poll.hpp>
#include <lely/io2/sys/io.hpp>
#include <lely/io2/sys/sigset.hpp>
#include <lely/io2/sys/timer.hpp>
#include <lely/coapp/fiber_driver.hpp>
#include <lely/coapp/master.hpp>
#include <iostream>
#include <cstdlib>
#include <thread>
#include <chrono>
#include <atomic>
#include <sstream>
#include <time.h>


using namespace lely;
using namespace std::chrono_literals;


class PD4Motor : public canopen::FiberDriver
{
public:
    //using FiberDriver::FiberDriver;
    PD4Motor(canopen::AsyncMaster& master, uint8_t id) :
        FiberDriver(master, id)
    {}

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
        EnableOperation = 0x0F
    };

    ~PD4Motor(){
        
    }


    void set_mode(OperatingMode mode)
    {
        Wait(AsyncWrite<int8_t>(0x6060, 0, mode));
    }

    void set_transition(TransitionCommand cmd)
    {
        Wait(AsyncWrite<uint16_t>(0x6040, 0, cmd));
    }

    void set_TargetVelocity(int16_t value)
    {
        //Wait(AsyncWrite<int16_t>(0x6042, 0, (int16_t)value));
        AsyncWrite<int16_t>(0x6042, 0, (int16_t)value, 20ms);
        std::this_thread::sleep_for(500ms);
    }

    int16_t get_RPDO_VelocityActualValue()
    {
        return rpdo_mapped[0x6044][0];
    }



private:

    void OnConfig(std::function<void(std::error_code ec)> res) noexcept override
    {
        puts("OnConfig");
        try
        {
            set_mode(PD4Motor::OperatingMode::Velocity);
            set_transition(PD4Motor::TransitionCommand::Shutdown);
            set_transition(PD4Motor::TransitionCommand::SwitchOn);
            set_transition(PD4Motor::TransitionCommand::EnableOperation);
            // Report success (empty error code).
            res({});
        }
        catch (canopen::SdoError &e)
        {
            // If one of the SDO requests resulted in an error, abort the
            // configuration and report the error code.
            res(e.code());
        }
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
            printf("TPDO VelocityActualValue: %d\n", get_RPDO_VelocityActualValue());
        }
    }
};


class CanOpenComms
{
public:
    CanOpenComms()
        {
            pass_ = false;
            start_thread();
            while(!pass_) {}
            //std::this_thread::sleep_for(5000ms);
            //stop_thread();
        }

    ~CanOpenComms(){
        stop_thread();

    }

    void start_thread() {
        //stop_flag = false;
        spinner = std::thread(&CanOpenComms::init, this);
    }

    void init() {
        ctx = new io::Context();
        poll = new io::Poll(*ctx);
        loop = new ev::Loop(poll->get_poll());
        exec = new ev::Executor(loop->get_executor());
        timer = new io::Timer(*poll, *exec, CLOCK_MONOTONIC);
        ctrl = new io::CanController("can0");
        chan = new io::CanChannel(*poll, *exec);
        chan->open(*ctrl);
        master_ = new canopen::AsyncMaster(*timer, *chan, "master.dcf", "", 1);
        
        master_->Reset();
        motor1_ = new PD4Motor(*master_, 2);
        motor2_ = new PD4Motor(*master_, 3);
        pass_ = true;
        puts("Start");
        loop->run();
        puts("Stop");
    }

    void stop_thread() {
        puts("Stop thread");
        //stop_flag = true;
        puts("Shutting motors down...");
        motor1_->AsyncWrite<uint16_t>(0x6040, 0, 0x06);
        motor2_->AsyncWrite<uint16_t>(0x6040, 0, 0x06);
        std::this_thread::sleep_for(100ms);
        loop->stop();
        if (spinner.joinable()) {
            spinner.join();
        }
        ctx->shutdown();
        puts("Stopped the thread");

    }

    PD4Motor *motor1_;
    PD4Motor *motor2_;



private:
    io::Context *ctx;
    io::Poll *poll;
    ev::Loop *loop;
    ev::Executor *exec;
    io::Timer *timer;
    io::CanController *ctrl;
    io::CanChannel *chan;
    canopen::AsyncMaster *master_;
    std::thread spinner;
    //std::atomic<bool> stop_flag;
    std::atomic<bool> pass_;
    
    //std::this_thread::sleep_for(std::chrono::seconds(4));
};

// int main() // .h of diffbot
// {
//     CanOpenComms comms_;

//     // for(int i=0 ; i<400; i++)
//     // {
//     //     comms_.motor1_->set_TargetVelocity(2*i);
//     //     if(i%10 == 0){ std::cout << "i is: " << i; };

//     // }


   
//     comms_.motor1_->AsyncWrite<int16_t>(0x6042, 0, 300);
//     puts("sleeping main");
//     std::this_thread::sleep_for(5000ms);
//     return EXIT_SUCCESS;
// }

// on_set velocity
// 
//
//