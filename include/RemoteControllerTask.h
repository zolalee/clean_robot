#ifndef REMOTE_CONTROLLER_TASK_H
#define REMOTE_CONTROLLER_TASK_H

#include <memory>
#include <mutex>
#include <condition_variable>
#include <core/queue.h>
#include <core/skeleton_thread.h>
#include <common_function/MotionControl.h>

class RemoteControllerTask : public core::SkeletonThread
{
public:
    struct ControlCommand
    {
        ControlCommand()
        {
            ir = 0;
            gamepad = 0;
            collision = 0;
        }

        int ir;
        int gamepad;
        int collision;
    };

    enum IrActionState
    {
        kIrActionIdle = 0,
        kIrActionGo,
        kIrActionTurnLeft,
        kIrActionTurnRight,
        kIrActionTurnBack,
    };

    enum GamepadActionState
    {
        kGamepadActionIdle = 0,
        kGamepadActionGo,
        kGamepadActionTurnLeft,
        kGamepadActionTurnRight,
        kGamepadActionTurnBack,
    };

public:
    RemoteControllerTask();

    static std::shared_ptr<RemoteControllerTask> getInstance();

    bool addCommand(std::shared_ptr<ControlCommand> command);

    void setMotionControl(useerobot::MotionControl *motion_control);

protected:
    virtual bool threadLoop() override;

    void handleCommand(std::shared_ptr<ControlCommand> command);

    void processAction();

    void processCollisionAction();

    uint32_t getStepByDurationMs(int ms);

private:
    static std::shared_ptr<RemoteControllerTask> instance_;

    core::Queue<std::shared_ptr<ControlCommand>> command_queue_;

    std::condition_variable event_condition_;
    std::mutex event_mutex_;

    bool collision_action;
    uint32_t collision_step;
    uint32_t collision_end;

    bool ir_action;
    int ir_state;
    bool gamepad_action;
    int gamepad_state;
    uint32_t current_step;
    uint32_t end_step;
    const int step_duration = 10; // 10ms

    useerobot::MotionControl *motion_control;
};

#endif
