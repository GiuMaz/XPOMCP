#pragma once

#include "beliefstate.h"
#include "json.hpp"
#include "node.h"
#include <iterator>
#include <memory>
#include <vector>

template<typename I, typename S>
class RUN {
public:
    using step_t = S;
    using run_info_t = I;
    using step_info_t = typename step_t::step_info_t;
    using size_type = typename std::vector<step_t>::size_type;

    RUN(run_info_t i, std::vector<step_t> s) : info_(i), steps_(s) {}

    const run_info_t &info() const { return info_; }
    run_info_t &info() { return info_; }

    const std::vector<step_t> &steps() const {
        return steps_;
    }

    std::vector<step_t> &steps() {
        return steps_;
    }

    const step_t &step(size_type i) const {
        return steps_[i];
    }
    step_t &step(size_type i) {
        return steps_[i];
    }

private:
    run_info_t info_;
    std::vector<step_t> steps_;
};

template<typename R>
class StepIterator {
public:
    using run_t = R;
    using step_t = typename R::step_t;

    using iterator_category = std::forward_iterator_tag;
    using difference_type   = std::ptrdiff_t;
    using value_type        = step_t;
    using pointer           = value_type*;
    using reference         = value_type&;
    using size_type = typename std::vector<run_t>::size_type;

    StepIterator(std::vector<run_t> &d, size_type r, size_type s)
        : data_(d), run_(r), step_(s) {}

    reference operator*() const { return data_[run_].step(step_); }
    pointer operator->() { return &data_[run_].step(step_); }

    // Prefix increment
    StepIterator& operator++() {
        if (run_ >= data_.size())
            return *this;

        ++step_;
        if (step_ >= data_[run_].steps().size()) {
            step_ = 0;
            ++run_;
        }

        return *this;
    }  

    // Postfix increment
    StepIterator operator++(int) {
        StepIterator tmp = *this;
        ++(*this);
        return tmp;
    }

    friend bool operator==(const StepIterator<R> &a, const StepIterator<R> &b) {
        return std::addressof(a) == std::addressof(b) 
               && a.run_ == b.run_
               && a.step_ == b.step_;
    };

    friend bool operator!=(const StepIterator<R> &a, const StepIterator<R> &b) {
        return std::addressof(a) != std::addressof(b) 
               || a.run_ != b.run_ 
               || a.step_ != b.step_;
    };

private:
    std::vector<run_t> &data_;
    size_type run_, step_;
};

template<typename R>
class ConstantStepIterator {
public:
    using run_t = R;
    using step_t = typename R::step_t;

    using iterator_category = std::forward_iterator_tag;
    using difference_type   = std::ptrdiff_t;
    using value_type        = step_t;
    using pointer           = value_type*;
    using reference         = value_type&;
    using size_type = typename std::vector<run_t>::size_type;

    ConstantStepIterator(const std::vector<run_t> &d, size_type r, size_type s)
        : data_(d), run_(r), step_(s) {}

    reference operator*() const { return data_[run_].step(step_); }
    pointer operator->() const { return &data_[run_].step(step_); }

    // Prefix increment
    ConstantStepIterator& operator++() {
        if (run_ >= data_.size())
            return *this;

        ++step_;
        if (step_ >= data_[run_].steps().size()) {
            step_ = 0;
            ++run_;
        }

        return *this;
    }  

    // Postfix increment
    ConstantStepIterator operator++(int) {
        ConstantStepIterator tmp = *this;
        ++(*this);
        return tmp;
    }

    friend bool operator==(const ConstantStepIterator<R> &a,
                           const ConstantStepIterator<R> &b) {
        return std::addressof(a) == std::addressof(b) && a.run_ == b.run_ &&
               a.step_ == b.step_;
    };

    friend bool operator!=(const ConstantStepIterator<R> &a,
                           const ConstantStepIterator<R> &b) {
        return std::addressof(a) != std::addressof(b) || a.run_ != b.run_ ||
               a.step_ != b.step_;
    };

private:
    const std::vector<run_t> &data_;
    size_type run_, step_;
};

template<typename R>
struct AllSteps {
    using run_t = R;

    AllSteps(std::vector<run_t> &d) : data_(d) {}

    StepIterator<run_t> begin() {
        return {data_, 0, 0};
    }

    StepIterator<run_t> end() {
        return {data_,data_.size(), 0};
    }

    ConstantStepIterator<run_t> cbegin() const {
        return {data_, 0, 0};
    }

    ConstantStepIterator<run_t> cend() const {
        return {data_,data_.size(), 0};
    }

private:
    std::vector<run_t> &data_;
};

template<typename R>
struct ConstantAllSteps {
    using run_t = R;

    ConstantAllSteps(const std::vector<run_t> &d) : data_(d) {}

    ConstantStepIterator<run_t> begin() const {
        return {data_, 0, 0};
    }

    ConstantStepIterator<run_t> end() const {
        return {data_,data_.size(), 0};
    }

    ConstantStepIterator<run_t> cbegin() const {
        return begin();
    }

    ConstantStepIterator<run_t> cend() const {
        return end();
    }

private:
    const std::vector<run_t> &data_;
};

template<typename I, typename R>
class TRACE {
public:

    using run_t = R;
    using step_t = typename run_t::step_t;

    using trace_info_t = I;
    using run_info_t = typename run_t::run_info_t;
    using step_info_t = typename run_t::step_info_t;
    using size_type = typename std::vector<run_t>::size_type;


    // run generation
    void start_run() {
        current_run_.clear();
    }

    void end_run() {
        runs_.emplace_back(current_run_info_, current_run_);
        current_run_.clear();
        current_run_info_ = {};
    }

    // step generation
    void start_step() {
        current_run_.push_back({});
    }

    void end_step() {
    }

    void save_belief(const BELIEF_STATE &b) { 
        current_run_.rbegin()->save_belief(b);
    }

    void save_action(int a) {
        current_run_.rbegin()->save_action(a);
    }

    void save_action_rank(const VNODE * const a) {
        current_run_.rbegin()->save_action_rank(a);
    }

    // trace-, run-, and step-specific information
    const trace_info_t &info() const {
        return info_;
    }

    trace_info_t &info() {
        return info_;
    }

    void save_step_info(step_info_t i) {
        current_run_.rbegin()->save_info(i);
    }

    void save_run_info(run_info_t i) {
        current_run_.save_info(i);
    }

    void save_trace_info(trace_info_t i) {
        info_ = i;
    }

    AllSteps<run_t> all_steps() {
        return AllSteps(runs_);
    }

    ConstantAllSteps<run_t> all_steps() const {
        return ConstantAllSteps(runs_);
    }

    std::vector<run_t> &runs() {
        return runs_;
    }

    const std::vector<run_t> &runs() const {
        return runs_;
    }

    run_t &run(size_type i) {
        return runs_[i];
    }

    const run_t &run(size_type i) const {
        return runs_[i];
    }

    // data serialization
    nlohmann::json serialize() const {
        // TODO: save metainfo
        nlohmann::json j;
        std::vector<nlohmann::json> rns;
        for (const auto &run : runs_) {
            std::vector<nlohmann::json> stp;
            for (const auto &s : run.steps())
                stp.push_back(s);
            rns.push_back(stp);
        }
        j = rns;
        return j;
    }

    void deserialize(const nlohmann::json &j) {
        runs_.clear();
        for (const auto &r : j) {
            start_run();
            for (const auto &s : r) {
                current_run_.push_back(s.get<step_t>());
            }
            end_run();
        }
    }

private:
    std::vector<run_t> runs_;
    trace_info_t info_;

    std::vector<step_t> current_run_;
    run_info_t current_run_info_;
};

