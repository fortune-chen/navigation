#include <vector>
#include <iostream>
#include <cstring>
#include <csignal>
#include <fcntl.h>
#include <unistd.h>
#include <execinfo.h>
#include "debug_helper.h"

#define FUNC_NAME_BUF_SIZE    64

using namespace utils;

DebugHelper* DebugHelper::mInstance = nullptr;
std::mutex DebugHelper::mInstanceMutex;

static std::string fileName;
static void signalHandleFunction(int signal);

std::vector<int> signals {SIGABRT, SIGBUS, SIGFPE, SIGILL, SIGSEGV, SIGKILL};

DebugHelper::DebugHelper() : mInited(false) {

};

DebugHelper::~DebugHelper() {
    delete mInstance;
    mInstance = nullptr;
}

DebugHelper* DebugHelper::getInstance() {
    if (mInstance == nullptr) {
        std::lock_guard<std::mutex> lock(mInstanceMutex);
        if (mInstance == nullptr) {
            mInstance = new DebugHelper();
        }
    }
    return mInstance;  
}

void DebugHelper::init() {
    init("");
}

void DebugHelper::init(const std::string& name) {
    if (mInited) {
        std::cout << "[DebugHelper][warning] DebugHelper init already!\n";
        return;
    }
    mInited = true;
    fileName = name;

    struct sigaction sa;
    for (auto &signal : signals) {
        memset(&sa, 0, sizeof(struct sigaction));
        sa.sa_handler = signalHandleFunction;
        sigemptyset(&sa.sa_mask);
        if (sigaction(signal, &sa, nullptr) == -1) {
            std::cout << "[DebugHelper][error] sigaction failed, signal : " << signal << std::endl;
        }
    }
}

void signalHandleFunction(int signal) {
    std::string signalType;
    bool isValid = false;
    for (auto &sig : signals) {
        if (sig == signal) {
            isValid = true;
        }
    }

    if (!isValid) {
        std::cout << "[DebugHelper][error] unexpected signal : " << signal << std::endl;
        return;
    }

    switch (signal) {
        case SIGABRT:signalType = "SIGABRT";
            break;
        case SIGBUS:signalType = "SIGBUS";;
            break;
        case SIGFPE:signalType = "SIGFPE";;
            break;
        case SIGILL:signalType = "SIGILL";;
            break;
        case SIGSEGV:signalType = "SIGSEGV";;
            break;
        case SIGKILL:signalType = "SIGKILL";;
            break;
        default:break;
    }

    std::cout << "[DebugHelper][info] catch signal : " << signalType << std::endl;

    // struct sigaction sa;
    // memset(&sa, 0, sizeof(struct sigaction));
    // sa.sa_handler = SIG_DFL;
    // sigemptyset(&sa.sa_mask);
    // if (sigaction(signal, &sa, nullptr) == -1) {
    //     std::cout << "[DebugHelper][error] sigaction failed, signal : " << signal << std::endl;
    // }

    void *stacks[FUNC_NAME_BUF_SIZE];
    if (fileName.empty()) {
        auto depth = backtrace(stacks, sizeof(stacks) / sizeof(void *));
        char** stackStr = backtrace_symbols(stacks, depth);
        std::cout << "------------------------------ backtrace ---------------------------------\n";
        for (int i = 0; i < depth; i++) {
            printf("[%02d] %s\n", i, stackStr[i]);
        }
        std::cout << "--------------------------------------------------------------------------\n";
    } else {
        std::string file = fileName + "-" + signalType;
        int fd = open(file.c_str(), O_CREAT | O_WRONLY, S_IRUSR | S_IWUSR);
        auto depth = backtrace(stacks, sizeof(stacks) / sizeof(void *));
        backtrace_symbols_fd(stacks, depth, fd);
        close(fd);
        std::cout << "[DebugHelper][info] generate backtrace file : " << file << std::endl;
    }
}
