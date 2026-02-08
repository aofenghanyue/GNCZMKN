/**
 * @file state_machine.hpp
 * @brief 通用状态机库
 */
#pragma once

#include <functional>
#include <unordered_map>
#include <stdexcept>

namespace gnc::libraries {

/**
 * @brief 通用状态机
 * @tparam StateEnum 状态枚举类型
 */
template<typename StateEnum>
class StateMachine {
public:
    using TransitionCallback = std::function<void()>;
    using UpdateCallback = std::function<void(double dt)>;
    
    explicit StateMachine(StateEnum initial_state) 
        : current_state_(initial_state) {}
    
    /// 获取当前状态
    StateEnum getCurrentState() const { return current_state_; }
    
    /// 设置状态进入时的回调
    void onEnter(StateEnum state, TransitionCallback callback) {
        enter_callbacks_[state] = std::move(callback);
    }
    
    /// 设置状态退出时的回调
    void onExit(StateEnum state, TransitionCallback callback) {
        exit_callbacks_[state] = std::move(callback);
    }
    
    /// 设置状态更新时的回调
    void onUpdate(StateEnum state, UpdateCallback callback) {
        update_callbacks_[state] = std::move(callback);
    }
    
    /// 切换状态
    void transitionTo(StateEnum new_state) {
        if (new_state == current_state_) return;
        
        // 调用退出回调
        auto exit_it = exit_callbacks_.find(current_state_);
        if (exit_it != exit_callbacks_.end()) {
            exit_it->second();
        }
        
        current_state_ = new_state;
        
        // 调用进入回调
        auto enter_it = enter_callbacks_.find(current_state_);
        if (enter_it != enter_callbacks_.end()) {
            enter_it->second();
        }
    }
    
    /// 更新当前状态
    void update(double dt) {
        auto it = update_callbacks_.find(current_state_);
        if (it != update_callbacks_.end()) {
            it->second(dt);
        }
    }
    
private:
    StateEnum current_state_;
    std::unordered_map<StateEnum, TransitionCallback> enter_callbacks_;
    std::unordered_map<StateEnum, TransitionCallback> exit_callbacks_;
    std::unordered_map<StateEnum, UpdateCallback> update_callbacks_;
};

} // namespace gnc::libraries
