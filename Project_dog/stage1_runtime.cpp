#include "stage1_runtime.h"
#include <atomic>

namespace stage1 {
namespace {
std::atomic<bool> active{false};
std::atomic<bool> model_holding{false};
std::atomic<bool> stop_requested{false};
}

void request_stop()
{
    if (active || model_holding) stop_requested = true;
}
}
