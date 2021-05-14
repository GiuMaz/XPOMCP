#pragma once

#include <string>
#include <fstream>
#include <vector>
#include <chrono>

class xes_attribute {
public:
    xes_attribute(const std::string &k, const std::string &v):
        type("string"), key(k), value(v) {}
    xes_attribute(const std::string &k, const char *v):
        type("string"), key(k), value(v) {}
    xes_attribute(const std::string &k, unsigned short v ):
        type("int"), key(k), value(std::to_string(v)) {}
    xes_attribute(const std::string &k, unsigned int v ):
        type("int"), key(k), value(std::to_string(v)) {}
    xes_attribute(const std::string &k, unsigned long v ):
        type("int"), key(k), value(std::to_string(v)) {}
    xes_attribute(const std::string &k, short v ):
        type("int"), key(k), value(std::to_string(v)) {}
    xes_attribute(const std::string &k, int v ):
        type("int"), key(k), value(std::to_string(v)) {}
    xes_attribute(const std::string &k, long v ):
        type("int"), key(k), value(std::to_string(v)) {}
    xes_attribute(const std::string &k, double v ):
        type("float"), key(k), value(std::to_string(v)) {}
    xes_attribute(const std::string &k, bool v ):
        type("boolean"), key(k), value(std::to_string(v)) {}
    /*
    template<typename Clock>
    xes_attribute(const std::string &k, std::chrono::time_point<Clock> v):
        type("date"), key(k), value(std::to_string(v)) {}
    */

    const std::string &get_type() const {
        return type;
    }

    const std::string &get_key() const {
        return key;
    }

    const std::string &get_value() const {
        return value;
    }

private:
    std::string type;
    std::string key;
    std::string value;
};

class xes_logger {
public:
    xes_logger(std::string f)
        : file_name(f), file(f.c_str()), state() {
            file << "<log "
                << "xes.version=\"2.0\" "
                << "xes.features=\"\" "
                << "openxes.version=\"2.26\" "
                << "xmlns=\"rttp://www.w3.org/2001/XMLSchema\">" << std::endl;
            state.set_fsm(fsm_in_log);
        }

    // non-copiable
    xes_logger(const xes_logger &l) = delete;
    xes_logger &operator=(const xes_logger &l) = delete;

    // movable
    xes_logger(xes_logger &&l) = default;
    xes_logger &operator=(xes_logger &&l) = default;

    ~xes_logger() {
        close();
    }

    void add_attribute(const xes_attribute &a) {
        file << indent()
             << "<" << a.get_type()
             << " key=\"" << a.get_key() << "\""
             << " value=\"" << a.get_value() << "\""
             << "/>\n";
    }

    void add_attributes(const std::vector<xes_attribute> &attrs) {
        for (const auto &at : attrs)
            add_attribute(at);
    }

    void start_trace() {
        if (!state.is_in_log())
            throw std::runtime_error("misplaced trace");
        file << indent() << "<trace>\n";
        state.set_fsm(fsm_in_trace);
    }

    void end_trace() {
        if (!state.is_in_trace())
            throw std::runtime_error("not in trace");
        state.set_fsm(fsm_in_log);
        file << indent() << "</trace>" << std::endl;
    }

    void start_event() {
        if (!state.is_in_trace())
            throw std::runtime_error("event outside a trace");
        file << indent() << "<event>\n";
        state.set_fsm(fsm_in_event);
    }

    void end_event() {
        if (!state.is_in_event())
            throw std::runtime_error("no event to close");
        state.set_fsm(fsm_in_trace);
        file << indent() << "</event>\n";
    }

    void start_list(const std::string &key) {
        file << indent() << "<list key=\"" << key << "\">\n";
        state.push_list_stack();
    }

    void end_list() {
        state.pop_list_stack();
        file << indent() << "</list>\n";
    }

    /* not supported in the python lib
    void start_container(const std::string &key) {
        file << indent() << "<container key=\"" << key << "\">\n";
        state.push_container_stack();
    }

    void end_container() {
        state.pop_container_stack();
        file << indent() << "</container>\n";
    }
    */

private:
    enum fsm_state {
        fsm_close = 0,
        fsm_in_log = 1,
        fsm_in_trace = 2,
        fsm_in_event = 3
    };

    class xes_state {
    public:
        xes_state() : fsm(fsm_close), list_stack(0), container_stack(0) {}
        bool is_close() { return fsm == fsm_close; }
        bool is_in_log() { return fsm == fsm_in_log; }
        bool is_in_trace() { return fsm == fsm_in_trace; }
        bool is_in_event() { return fsm == fsm_in_event; }

        void set_fsm(fsm_state s) {
            fsm = s;
        }

        void push_list_stack() {
            ++list_stack;
        }

        void pop_list_stack() {
            if (list_stack == 0)
                throw std::runtime_error("no list to close");
            --list_stack;
        }

        void push_container_stack() {
            ++container_stack;
        }

        void pop_container_stack() {
            if (container_stack == 0)
                throw std::runtime_error("no container to close");
            --container_stack;
        }

        uint32_t indent_depth() {
            return fsm + list_stack + container_stack;
        }

    private:
        fsm_state fsm : 2;
        uint32_t list_stack : 15;
        uint32_t container_stack : 15;
    };


    std::string indent() {
        return std::string(4 * state.indent_depth(), ' ');
    }

    void close() {
        if (state.is_close())
            return;
        file << "</log>" << std::endl;
        file.close();
        state.set_fsm(fsm_close);
    }

    xes_state state;
    std::string file_name;
    std::ofstream file;
};

