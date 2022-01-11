#include <RemoteControllerTask.h>
#include <core/log.h>
#include <PathPlanningInterface.h>
#include <message/navigation_msg.h>
#include <um_chassis/chassisBase.h>

using namespace core;

static const char *TAG = "remote_controller";

#define GO_ACTION_DURATION      (500)
#define TURN_ACTION_DURATION    (500)
#define BACK_ACTION_DURACTION   (3430)
#define GAMEPAD_MAX_DURATION    (100*1000)


std::shared_ptr<RemoteControllerTask> RemoteControllerTask::instance_ = nullptr;


RemoteControllerTask::RemoteControllerTask()
{
    collision_action = false;
    ir_action = false;
    gamepad_action = false;
}

std::shared_ptr<RemoteControllerTask> RemoteControllerTask::getInstance()
{
    if (instance_ == nullptr) {
        instance_ = std::make_shared<RemoteControllerTask>();
    }

    return instance_;
}

bool RemoteControllerTask::addCommand(std::shared_ptr<ControlCommand> command)
{
    bool success;
    std::unique_lock<std::mutex> lck(event_mutex_);
    
    success = command_queue_.enqueue(command);
    event_condition_.notify_all();

    return success;
}

void RemoteControllerTask::setMotionControl(useerobot::MotionControl *motion_control)
{
    this->motion_control = motion_control;
}

bool RemoteControllerTask::threadLoop()
{
    bool success;
    bool got_task;
    std::shared_ptr<ControlCommand> command;

    processAction();

    {
        std::unique_lock<std::mutex> lock(event_mutex_);
        got_task = event_condition_.wait_for(lock, std::chrono::milliseconds(step_duration),  [&] {
            return command_queue_.size() != 0;
        });

        if (got_task == false) {
            return true;
        }
    }

    success = command_queue_.dequeue(command);
    if (success == false) {
        log_error(TAG, "got command from command queue failed");
        return true;
    }
    log_info(TAG, "got remote controller command ir %d gamepad %d", command->ir, command->gamepad);

    handleCommand(command);

    return true;
}

void RemoteControllerTask::handleCommand(std::shared_ptr<ControlCommand> command)
{
    if (command->collision) {
        collision_action = true;
        collision_step = 0;
        collision_end = getStepByDurationMs(200);
        return;
    }

    // gamepad优先级更高，如有gamepad正在进行，忽略此命令
    if (command->ir != REMOTE_CONTROL_IDLE) {
        if (gamepad_action) {
            return;
        }

        // 除后转，会覆盖原来的任务
        if (ir_action && ir_state == kIrActionTurnBack) {
            return;
        }
        ir_action = true;
        current_step = 0;
        switch (command->ir)
        {
        case REMOTE_CONTROL_GO:
            ir_state = kIrActionGo;
            end_step = getStepByDurationMs(GO_ACTION_DURATION);
            log_info(TAG, "ir go");
            break;
        case REMOTE_CONTROL_TURN_LEFT:
            ir_state = kIrActionTurnLeft;
            end_step = getStepByDurationMs(TURN_ACTION_DURATION);
            log_info(TAG, "ir turn left");
            break;
        case REMOTE_CONTROL_TURN_RIGHT:
            ir_state = kIrActionTurnRight;
            end_step = getStepByDurationMs(TURN_ACTION_DURATION);
            log_info(TAG, "ir turn right");
            break;
        case REMOTE_CONTROL_TURN_BACK:
            ir_state = kIrActionTurnBack;
            end_step = getStepByDurationMs(BACK_ACTION_DURACTION);
            log_info(TAG, "ir turn back");
            break;
        default:
            break;
        }
    }

    if (command->gamepad != GAMEPAD_IDLE) {
        // 除后转，会覆盖原来的任务
        if (gamepad_action && gamepad_state == kGamepadActionTurnBack) {
            return;
        }

        gamepad_action = true;
        current_step = 0;
        end_step = getStepByDurationMs(GAMEPAD_MAX_DURATION); // 最大100秒，待定
        switch (command->gamepad)
        {
        case GAMEPAD_GO:
            gamepad_state = kGamepadActionGo;
            log_info(TAG, "gamepad go");
            break;
        case GAMEPAD_TURN_LEFT:
            gamepad_state = kGamepadActionTurnLeft;
            log_info(TAG, "gamepad turn left");
            break;
        case GAMEPAD_TURN_RIGHT:
            gamepad_state = kGamepadActionTurnRight;
            log_info(TAG, "gamepad turn right");
            break;
        case GAMEPAD_TURN_BACK:
            gamepad_state = kGamepadActionTurnBack;
            log_info(TAG, "gamepad turn back");
            end_step = getStepByDurationMs(BACK_ACTION_DURACTION);
            break;
        default:
            break;
        }
    } else {
        // 后退让其自然走完
        if (gamepad_action && gamepad_state != kGamepadActionTurnBack) {
            gamepad_action = false;
            gamepad_state = kGamepadActionIdle;
            current_step = 0;
            end_step = 0;
            useerobot::GLOBAL_CONTROL = useerobot::WHEEL_STOP;
            motion_control->robotchassiscontrol.chassisSpeed(0,0,1);
            log_info(TAG, "gamepad stop action by recv event");
        }
    }
}

void RemoteControllerTask::processAction()
{
    if (collision_action) {
        processCollisionAction();
        return;
    }

    if (ir_action == false && gamepad_action == false) {
        return;
    }

    // gamepad优先级更高
    current_step++;
    if (current_step >= end_step) {
        if (ir_action) {
            ir_action = false;
            ir_state = kIrActionIdle;
            end_step = 0;
            current_step = 0;
            useerobot::GLOBAL_CONTROL = useerobot::WHEEL_STOP;
            motion_control->robotchassiscontrol.chassisSpeed(0,0,1);
            log_info(TAG, "ir stop action by end");
        }

        if (gamepad_action) {
            gamepad_action = false;
            gamepad_state = kGamepadActionIdle;
            end_step = 0;
            current_step = 0;
            useerobot::GLOBAL_CONTROL = useerobot::WHEEL_STOP;
            motion_control->robotchassiscontrol.chassisSpeed(0,0,1);
            log_info(TAG, "gamepad stop action by end");
        }
    }

    if (ir_action) {
        switch (ir_state)
        {
        case kIrActionGo:
            useerobot::GLOBAL_CONTROL = useerobot::WHEEL_RUN;
            motion_control->robotchassiscontrol.chassisSpeed(200,200,1);
            break;
        case kIrActionTurnLeft:
            useerobot::GLOBAL_CONTROL = useerobot::WHEEL_RUN;
            motion_control->robotchassiscontrol.chassisSpeed(-50,50,1);
            break;
        case kIrActionTurnRight:
            useerobot::GLOBAL_CONTROL = useerobot::WHEEL_RUN;
            motion_control->robotchassiscontrol.chassisSpeed(50,-50,1);
            break;
        case kIrActionTurnBack:
            useerobot::GLOBAL_CONTROL = useerobot::WHEEL_RUN;
            motion_control->robotchassiscontrol.chassisSpeed(-100,100,1);
            break;
        default:
            // motion_control->robotchassiscontrol.chassisSpeed(0,0,1);
            break;
        }
    }

    if (gamepad_action) {
        switch (gamepad_state)
        {
        case kGamepadActionGo:
            useerobot::GLOBAL_CONTROL = useerobot::WHEEL_RUN;
            motion_control->robotchassiscontrol.chassisSpeed(200,200,1);
            break;
        case kGamepadActionTurnLeft:
            useerobot::GLOBAL_CONTROL = useerobot::WHEEL_RUN;
            motion_control->robotchassiscontrol.chassisSpeed(-50,50,1);
            break;
        case kGamepadActionTurnRight:
            useerobot::GLOBAL_CONTROL = useerobot::WHEEL_RUN;
            motion_control->robotchassiscontrol.chassisSpeed(50,-50,1);
            break;
        case kGamepadActionTurnBack:
            useerobot::GLOBAL_CONTROL = useerobot::WHEEL_RUN;
            motion_control->robotchassiscontrol.chassisSpeed(-100,100,1);
            break;
        default:
            // motion_control->robotchassiscontrol.chassisSpeed(0,0,1);
            break;
        }
    }
}

void RemoteControllerTask::processCollisionAction()
{
    collision_step++;
    if (collision_step >= collision_end) {
        collision_action = false;
        useerobot::GLOBAL_CONTROL = useerobot::WHEEL_STOP;
        motion_control->robotchassiscontrol.chassisSpeed(0,0,1);
    } else {
        useerobot::GLOBAL_CONTROL = useerobot::WHEEL_RUN;
        motion_control->robotchassiscontrol.chassisSpeed(-200,-200,1);
    }
}

uint32_t RemoteControllerTask::getStepByDurationMs(int ms)
{
    uint32_t step = ms / step_duration;

    return step;
}
