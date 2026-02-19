#pragma once

#include <atomic>
#include <chrono>
#include <functional>
#include <mutex>
#include <thread>
#include <iostream>

class Watchdog
{
public:
    explicit Watchdog(
        std::chrono::milliseconds timeout = std::chrono::milliseconds(1000),
        std::chrono::milliseconds check_interval = std::chrono::milliseconds(500),
        std::function<void()> callback = []() {} )
        : timeout_(timeout),
          check_interval_(check_interval),
          callback_(std::move(callback)),
          last_reset_(std::chrono::steady_clock::now())
    {
        is_running_.store(true, std::memory_order_release);

        watchdog_thread_ = std::thread([this]() {
            //pthread_setname_np(pthread_self(), "watchdog_thread");
            while (is_running_.load(std::memory_order_acquire)) {
                const auto to_sleep = get_check_interval();
                const auto time_out = get_timeout();

                std::this_thread::sleep_for(to_sleep);

                if (!is_enabled_.load(std::memory_order_acquire)) continue;

                const auto now = std::chrono::steady_clock::now();
                std::chrono::steady_clock::time_point last_reset_cp;
                {
                    std::lock_guard<std::mutex> lock(reset_mutex_);
                    last_reset_cp = last_reset_;
                }

                std::function<void()> current_callback;
                {
                    std::lock_guard<std::mutex> lock(callback_mutex_);
                    if (is_enabled_.load(std::memory_order_relaxed)
                        && (now - last_reset_cp) >= time_out) {
                        current_callback = callback_;
                    }
                }

                if (current_callback) current_callback();
            }
        });
    }

    ~Watchdog(){
        is_enabled_.store(false, std::memory_order_release);
        is_running_.store(false, std::memory_order_release);
        if (watchdog_thread_.joinable()) {
            watchdog_thread_.join();
        }
    }

    void reset(){
        std::lock_guard<std::mutex> lock(reset_mutex_);
        last_reset_ = std::chrono::steady_clock::now();
    }

    void stop() noexcept {
        is_enabled_.store(false, std::memory_order_release);
    }

    void start() {
        reset();
        is_enabled_.store(true, std::memory_order_release);
    }

    void set_timeout(std::chrono::milliseconds timeout){
        std::lock_guard<std::mutex> lock(reset_mutex_);
        timeout_ = timeout;
    }

    std::chrono::milliseconds get_timeout() noexcept {
        std::lock_guard<std::mutex> lock(reset_mutex_);
        return timeout_;
    }

    std::function<void()> get_callback() const noexcept{
        return callback_;
    }

    void set_callback(std::function<void()> callback) {
        std::lock_guard<std::mutex> lock(callback_mutex_);
        callback_ = std::move(callback);
    }
    void set_check_interval(std::chrono::milliseconds interval) {
        std::lock_guard<std::mutex> lock(reset_mutex_);
        check_interval_ = interval;
    }

    std::chrono::milliseconds get_check_interval() noexcept {
        std::lock_guard<std::mutex> lock(reset_mutex_);
        return check_interval_;
    }
    
private:
    std::chrono::milliseconds timeout_;
    std::chrono::milliseconds check_interval_;

    std::function<void()> callback_;
    std::atomic<bool> is_running_{false};
    std::atomic<bool> is_enabled_{false};
    std::chrono::steady_clock::time_point last_reset_;

    std::mutex callback_mutex_;
    std::mutex reset_mutex_;
    std::thread watchdog_thread_;
};
