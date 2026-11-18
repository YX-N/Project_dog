extern "C" {
#include "ethercat.h"
#include "motor_control.h"
#include "transmit.h"
}

#include "queue.h"
#include "motor_bus.h"
#include "periodic_clock.h"
#include <sys/time.h>
#include <cinttypes>
#include <cstdio>
#include <cstring>
#include <mutex>
#include <condition_variable>
#include <chrono>
#include "time.h"

spsc_queue<Queue_Msg_ptr, capacity<10>> messages[SLAVE_NUMBER];
std::atomic<bool> running{false};           // 控制周期线程的运行标志；false 时 runImpl() 退出。
std::thread runThread;                      // 执行周期性 EtherCAT PDO 收发的后台线程。

namespace {
std::mutex direct_mutex;                    //创建一个名为 direct_mutex 的互斥锁对象，保护直接控制帧、阶段标识及其提交/完成状态
EtherCAT_Msg direct_frame{}, cycle_frame{}; //定义两个 EtherCAT_Msg 类型的报文变量,direct_frame 保存上层提交的整帧；cycle_frame 保存本周期实际准备发送的整帧
const char* direct_phase = "idle";          //phase 用于标识当前报文的处理阶段，初始值为 "idle"
const char* cycle_phase = "idle";           // direct_phase 随直接控制帧传递；cycle_phase 标记当前周期实际发送的控制阶段
MotorCycleObserver cycle_observer = nullptr; //定义一个通用指针，初始为空，周期收发完成后的观察回调及其用户上下文
void* cycle_observer_context = nullptr;
bool direct_active = false, direct_pending = false; //直接通信是否正在执行,是否有待处理的直接通信请求
std::condition_variable direct_changed;             //协调线程等待和唤醒

std::uint64_t direct_serial = 0, direct_completed = 0, cycle_direct_serial = 0;//direct_serial 是最新提交编号；direct_completed 是已完成编号；cycle_direct_serial 是本周期取用的编号
std::mutex query_mutex, query_serial;               //std::mutex：C++ 标准库的互斥锁类型,进入作用域自动加锁，离开作用域自动解锁
std::condition_variable query_changed;
struct {
    bool active = false, sent = false, done = false;
    int slave = 0, passage = 1;
    Motor_Msg request{}, reply{};
} query;                                           //query 相当于一个查询任务控制器,统一保存：查询状态 + 目标从站 + 通道参数 + 请求报文 + 响应报文
}


/**
 * @brief 查询指定从站电机的数据
 * @param slave 目标 EtherCAT 从站编号(从0开始)
 * @param passage 电机查询通道（1~6）
 * @param request 输入的电机查询请求报文
 * @param reply 输出的电机响应报文，查询成功时写入
 * @return true 查询成功；false 查询失败或超时（1秒）
 */
bool queryMotor(int slave, int passage, const Motor_Msg& request, Motor_Msg& reply) {
    if (!running || slave < 0 || slave >= SLAVE_NUMBER || slave >= ec_slavecount ||
        passage < 1 || passage > 6) return false;
    std::lock_guard<std::mutex> serial(query_serial);
    std::unique_lock<std::mutex> lock(query_mutex);
    query = {true, false, false, slave, passage, request, {}};
    const bool ok = query_changed.wait_for(lock, std::chrono::seconds(1),
                                           [] { return query.done || !running; }) && query.done;
    if (ok) reply = query.reply;
    query.active = false;
    return ok;
}

/**
 * @brief 注册电机周期通信回调函数， 设置周期观察回调；按接口约定，应在 startBus() 前注册、stopBus() 后注销
 * @param observer 外部传入的回调函数
 * @param context 回调函数的上下文数据指针
 */
void setMotorCycleObserver(MotorCycleObserver observer, void* context) {
    cycle_observer = observer;
    cycle_observer_context = context;
}

/**
 * @brief 主动提交一次 PDO 通信请求
 * Direct（直接）是为了区别于周期性 PDO 通信（Cycle），Direct：需要时主动提交一条电机控制报文， Cycle：通信线程按照既定周期持续收发电机数据
 * @param frame 待发送的 EtherCAT 报文
 * @param phase 本次通信的阶段标识
 * @return 本次请求的递增序号
 */
std::uint64_t writeMotorDirect(const EtherCAT_Msg& frame, const char* phase) {
    std::lock_guard<std::mutex> lock(direct_mutex);
    direct_frame = frame;
    direct_phase = phase;
    direct_active = direct_pending = true;
    return ++direct_serial;
}

//检查是否存在待处理的主动 PDO 请求
bool motorDirectPending() {
    std::lock_guard<std::mutex> lock(direct_mutex);
    return direct_pending;
}
//结束当前主动 PDO 通信
void finishMotorDirect() {
    std::lock_guard<std::mutex> lock(direct_mutex);
    direct_active = false; // 尚未发送的最后一帧仍由下一次 PDO 刷新接走。
}

/**
 * @brief 提交直接控制帧，并阻塞等待后台线程完成对应周期的 PDO 交换
 * writeMotorDirect() 只提交请求，而 exchangeMotorFrame() 会进一步等待该请求完成
 * @param frame 待发送的 EtherCAT 报文
 * @param phase 本次通信的阶段标识
 * @note 等待请求完成或通信停止
 */
void exchangeMotorFrame(const EtherCAT_Msg& frame, const char* phase) {
    const auto serial = writeMotorDirect(frame, phase);
    std::unique_lock<std::mutex> lock(direct_mutex);
    direct_changed.wait(lock, [serial] { return direct_completed >= serial || !running; });
}


char IOmap[4096];                    // 存放所有从站的输入输出数据，大小 4KB
std::thread checkThread;             // 用于周期性检查 EtherCAT 通信状态的线程
std::atomic<bool> checking{false};   // 标记是否正在进行状态检查（线程安全）
int expectedWKC;                     // 期望的 Working Counter（工作计数器），用于判断通信是否完整
boolean needlf;                      // 是否需要换行（通常用于打印日志时的格式控制）
std::atomic<int> wkc{0};             // 当前实际收到的 Working Counter（原子变量，线程安全）
std::atomic<bool> inOP{false};       // 标记主站是否已进入 Operational 状态
uint8 currentgroup = 0;              // 当前使用的从站组号
bool isConfig[SLAVE_NUMBER]{false};  // 记录每个从站是否已完成配置（数组）

#define EC_TIMEOUTMON 500            // 状态监控超时时间，单位通常是毫秒

void EtherCAT_Data_Get();

void EtherCAT_Command_Set();

// EtherCAT 通信异常时停止电机总线服务并输出错误日志
static void degraded_handler()
{
    motor_bus_ready = false;
    printf("[EtherCAT Error] Logging error...\n");
    time_t current_time = time(NULL);
    char* time_str = ctime(&current_time);
    printf("ESTOP. EtherCAT became degraded at %s.\n", time_str);
    printf("[EtherCAT Error] Stopping RT process.\n");
}
/**
 * @brief 输出 EtherCAT 初始化失败时的从站状态和错误信息
 * @param step 当前失败的初始化步骤
 */
void EtherCAT_Init_Failure(const char* step)
{
    if (!inOP && ec_slavecount > 0) ec_readstate();
    flockfile(stdout);
    printf("[EtherCAT Init] step=%s slaves=%d states=[", step, ec_slavecount);
    for (int slave = 1; slave <= ec_slavecount; ++slave)
        printf("%s%d:0x%02x/AL=0x%04x", slave == 1 ? "" : ", ", slave,
               ec_slave[slave].state, ec_slave[slave].ALstatuscode);
    printf("] WKC=%d/%d\n", wkc.load(), expectedWKC);
    funlockfile(stdout);
}
/**
 * @brief 执行 SOEM 主站初始化，并将所有从站切换到 OP 状态。
 * @param ifname 用于 EtherCAT 通信的网卡名称。
 * @return 所有初始化步骤完成且从站进入 OP 时返回 1，否则返回 0。
 * @details 依次打开网卡、扫描从站、配置 PDO/DC、等待 SAFE_OP、交换初始 PDO，
 *          启动周期收发线程，再请求所有从站进入 OP；OP 请求失败时持续重试。
 */
static int run_ethercat(const char* ifname)
{
    int chk;
    needlf = FALSE;
    inOP = FALSE;
    ec_slavecount = expectedWKC = 0;
    wkc = 0;

    /* initialise SOEM, bind socket to ifname */
    if (ec_init(ifname))
    {
        printf("[EtherCAT Init] Initialization on device %s succeeded.\n", ifname);
        /* find and auto-config slaves */

        if (ec_config_init(FALSE) > 0)
        {
            printf("[EtherCAT Init] %d slaves found and configured.\n", ec_slavecount);
            if (ec_slavecount < SLAVE_NUMBER)
            {
                printf("[RT EtherCAT] Warning: Expected %d slaves, found %d.\n", SLAVE_NUMBER, ec_slavecount);
            }

            for (int slave_idx = 0; slave_idx < ec_slavecount; slave_idx++)
                ec_slave[slave_idx + 1].CoEdetails &= ~ECT_COEDET_SDOCA;

            if (ec_config_map(&IOmap) <= 0)
            {
                EtherCAT_Init_Failure("map PDO");
                return 0;
            }
            if (!ec_configdc())
                EtherCAT_Init_Failure("configure DC: no DC slaves; continuing");

            printf("[EtherCAT Init] Mapped slaves.\n");
            /* wait for all slaves to reach SAFE_OP state */
            if (ec_statecheck(0, EC_STATE_SAFE_OP, EC_TIMEOUTSTATE * SLAVE_NUMBER) != EC_STATE_SAFE_OP)
            {
                EtherCAT_Init_Failure("wait SAFE_OP timeout; retry");
                return 0;
            }

            for (int slave_idx = 0; slave_idx < ec_slavecount; slave_idx++)
            {
                printf("[SLAVE %d]\n", slave_idx);
                printf("  IN  %d bytes, %d bits\n", ec_slave[slave_idx].Ibytes, ec_slave[slave_idx].Ibits);
                printf("  OUT %d bytes, %d bits\n", ec_slave[slave_idx].Obytes, ec_slave[slave_idx].Obits);
                printf("\n");
            }

            printf("[EtherCAT Init] segments : %d : %d %d %d %d\n", ec_group[0].nsegments, ec_group[0].IOsegment[0],
                   ec_group[0].IOsegment[1], ec_group[0].IOsegment[2], ec_group[0].IOsegment[3]);

            printf("[EtherCAT Init] Requesting operational state for all slaves...\n");
            expectedWKC = (ec_group[0].outputsWKC * 2) + ec_group[0].inputsWKC;
            printf("[EtherCAT Init] Calculated workcounter %d\n", expectedWKC);
            if (ec_send_processdata() <= 0)
                EtherCAT_Init_Failure("send initial PDO");
            wkc = ec_receive_processdata(EC_TIMEOUTRET);
            if (wkc < expectedWKC)
                EtherCAT_Init_Failure("receive initial PDO; waiting for OP");
            startRun(); // 此后 PDO 只由收发线程处理，等待 OP 时也不中断。
            for (;;)
            {
                ec_slave[0].state = EC_STATE_OPERATIONAL;
                if (ec_writestate(0) <= 0)
                    EtherCAT_Init_Failure("request OP");
                chk = 40;
                do
                {
                    ec_statecheck(0, EC_STATE_OPERATIONAL, 50000);
                }
                while (chk-- && (ec_slave[0].state != EC_STATE_OPERATIONAL));

                if (ec_slave[0].state == EC_STATE_OPERATIONAL)
                {
                    ec_readstate(); // 同步各从站状态缓存。
                    printf("[EtherCAT Init] Operational state reached for all slaves.\n");
                    inOP = TRUE;
                    return 1;
                }
                EtherCAT_Init_Failure("wait OP timeout; retry");
            }
        }
        else
        {
            EtherCAT_Init_Failure("discover/configure slaves");
        }
    }
    else
    {
        EtherCAT_Init_Failure("open socket");
    }
    return 0;
}

static int err_count = 0;
static int err_iteration_count = 0;
/**@brief EtherCAT errors are measured over this period of loop iterations */
#define K_ETHERCAT_ERR_PERIOD 100

/**@brief Maximum number of etherCAT errors before a fault per period of loop iterations */
#define K_ETHERCAT_ERR_MAX 20

/**
 * @brief 后台监测 EtherCAT WKC 和从站状态，并尝试恢复异常从站。
 * @details 检查从站是否仍处于 OP；对 SAFE_OP、SAFE_OP+ERROR、丢失或异常状态
 *          分别执行状态确认、重新请求 OP、重配置或恢复操作。错误次数超过阈值时
 *          调用 degraded_handler() 并退出监测循环。
 */
static void ecatcheck()
{
    int slave = 0;
    while (checking)
    {
        // count errors
        if (err_iteration_count > K_ETHERCAT_ERR_PERIOD)
        {
            err_iteration_count = 0;
            err_count = 0;
        }

        if (err_count > K_ETHERCAT_ERR_MAX)
        {
            // possibly shut down
            printf("[EtherCAT Error] EtherCAT connection degraded.\n");
            printf("[Simulink-Linux] Shutting down....\n");
            degraded_handler();
            break;
        }
        err_iteration_count++;

        if (inOP && ((wkc < expectedWKC) || ec_group[currentgroup].docheckstate))
        {
            if (needlf)
            {
                needlf = FALSE;
                printf("\n");
            }
            /* one ore more slaves are not responding */
            ec_group[currentgroup].docheckstate = FALSE;
            ec_readstate();
            for (slave = 1; slave <= ec_slavecount; slave++)
            {
                if ((ec_slave[slave].group == currentgroup) && (ec_slave[slave].state != EC_STATE_OPERATIONAL))
                {
                    ec_group[currentgroup].docheckstate = TRUE;
                    if (ec_slave[slave].state == (EC_STATE_SAFE_OP + EC_STATE_ERROR))
                    {
                        printf("[EtherCAT Error] Slave %d is in SAFE_OP + ERROR, attempting ack.\n", slave);
                        ec_slave[slave].state = (EC_STATE_SAFE_OP + EC_STATE_ACK);
                        ec_writestate(slave);
                        err_count++;
                    }
                    else if (ec_slave[slave].state == EC_STATE_SAFE_OP)
                    {
                        printf("[EtherCAT Error] Slave %d is in SAFE_OP, change to OPERATIONAL.\n", slave);
                        ec_slave[slave].state = EC_STATE_OPERATIONAL;
                        ec_writestate(slave);
                        err_count++;
                    }
                    else if (ec_slave[slave].state > 0)
                    {
                        if (ec_reconfig_slave(slave, EC_TIMEOUTMON))
                        {
                            ec_slave[slave].islost = FALSE;
                            printf("[EtherCAT Status] Slave %d reconfigured\n", slave);
                        }
                    }
                    else if (!ec_slave[slave].islost)
                    {
                        /* re-check state */
                        ec_statecheck(slave, EC_STATE_OPERATIONAL, EC_TIMEOUTRET);
                        if (!ec_slave[slave].state)
                        {
                            ec_slave[slave].islost = TRUE;
                            printf("[EtherCAT Error] Slave %d lost\n", slave);
                            err_count++;
                        }
                    }
                }
                if (ec_slave[slave].islost)
                {
                    if (!ec_slave[slave].state)
                    {
                        if (ec_recover_slave(slave, EC_TIMEOUTMON))
                        {
                            ec_slave[slave].islost = FALSE;
                            printf("[EtherCAT Status] Slave %d recovered\n", slave);
                        }
                    }
                    else
                    {
                        ec_slave[slave].islost = FALSE;
                        printf("[EtherCAT Status] Slave %d found\n", slave);
                    }
                }
            }
            if (!ec_group[currentgroup].docheckstate)
                printf("[EtherCAT Status] All slaves resumed OPERATIONAL.\n");
        }
        osal_usleep(50000);
    }
}
// 初始化 EtherCAT 总线，失败时重试
void EtherCAT_Init(char* ifname)
{
    int i;
    int rc;
    printf("[EtherCAT] Initializing EtherCAT\n");
    for (i = 1; i < 10; i++)
    {
        printf("[EtherCAT] Attempting to start EtherCAT, try %d of 10.\n", i);
        rc = run_ethercat(ifname);
        if (rc)
            break;
        osal_usleep(1000000);
    }
    if (rc)
        printf("[EtherCAT] EtherCAT successfully initialized on attempt %d \n", i);
    else
    {
        EtherCAT_Init_Failure("initialize: retries exhausted");
    }
}
/**
 * @brief 将主站控制报文写入从站 PDO 输出区并发送
 * @param MasterCommand 各从站的控制报文数组
 */
void EtherCAT_Transmit(EtherCAT_Msg* MasterCommand)
{
    for (int i = 0; i < ec_slavecount; i++)
    {
        memcpy((void*)(ec_slave[0].outputs + i * sizeof(EtherCAT_Msg)), (void*)&(MasterCommand[i]),
               sizeof(EtherCAT_Msg));
    }
    ec_send_processdata();
}
// EtherCAT_Run() 当前统计窗口内的 PDO WKC 异常次数及循环次数
static int wkc_err_count = 0;
static int wkc_err_iteration_count = 0;

//数组大小根据从站数量确定
EtherCAT_Msg Rx_Message[SLAVE_NUMBER];
EtherCAT_Msg Tx_Message[SLAVE_NUMBER];

OD_Motor_Msg Rx_Motor_Msg[SLAVE_NUMBER][6];
/**
 * @brief 执行一个运行期 EtherCAT 通信周期。
 * @details 先组织并发送控制 PDO，再接收 PDO 并更新 WKC；随后判断总线是否满足
 *          可用条件、解包电机反馈、触发周期观察回调，并通知等待中的直接控制调用。
 *          若 WKC 低于预期值，则累计丢包错误次数。
 */
void EtherCAT_Run()
{
    if (wkc_err_iteration_count > K_ETHERCAT_ERR_PERIOD)
    {
        wkc_err_count = 0;
        wkc_err_iteration_count = 0;
    }
    if (wkc_err_count > K_ETHERCAT_ERR_MAX)
    {
        printf("[EtherCAT Error] Error count too high!\n");
        degraded_handler();
    }
    // send
    EtherCAT_Command_Set();
    ec_send_processdata();
    // receive
    wkc = ec_receive_processdata(EC_TIMEOUTRET);
    motor_bus_ready = inOP && ec_slavecount > 0 && ec_slavecount <= SLAVE_NUMBER &&
                     wkc >= expectedWKC && ec_slave[1].state == EC_STATE_OPERATIONAL &&
                     ec_slave[1].Obytes == sizeof(EtherCAT_Msg) &&
                     ec_slave[1].Ibytes >= sizeof(EtherCAT_Msg) &&
                     ec_slave[1].outputs && ec_slave[1].inputs;
    EtherCAT_Data_Get();
    if (cycle_observer) cycle_observer(cycle_frame, cycle_phase, cycle_observer_context);
    {
        std::lock_guard<std::mutex> lock(direct_mutex);
        if (cycle_direct_serial) direct_completed = cycle_direct_serial;
    }
    direct_changed.notify_all();
    //  check for dropped packet
    if (wkc < expectedWKC)
    {
        printf("\x1b[31m[EtherCAT Error] Dropped packet (Bad WKC!)\x1b[0m\n");
        wkc_err_count++;
    }
    else
    {
        needlf = TRUE;
    }
    wkc_err_iteration_count++;
}

/**
 * @brief 读取各从站 TxPDO 输入数据，匹配查询回复并更新电机反馈缓存。
 * @details WKC 不满足预期时直接返回，避免将旧 PDO 数据当作新反馈；通信正常时
 *          逐个读取从站输入帧，处理等待中的查询响应，解包 6 路电机数据，并在总线
 *          已就绪时调用 recordMotorFeedback() 更新统一反馈缓存。
 */
void EtherCAT_Data_Get()
{
    if (wkc < expectedWKC) return; // 不把丢包时缓存的 PDO 当作新反馈。
    for (int slave = 0; slave < ec_slavecount && slave < SLAVE_NUMBER; ++slave)
    {
        EtherCAT_Msg* slave_src = (EtherCAT_Msg*)(ec_slave[slave + 1].inputs);
        if (!slave_src || ec_slave[slave + 1].Ibytes < sizeof(EtherCAT_Msg)) continue;
        Rx_Message[slave] = *slave_src;
        {
            std::lock_guard<std::mutex> lock(query_mutex);
            if (query.active && query.sent && query.slave == slave) {
                for (const auto& frame : Rx_Message[slave].motor) {
                    if (frame.id != query.request.id) continue;
                    const bool match = frame.id == 0x7ff
                        ? frame.dlc >= 5 && frame.data[2] == 1 &&
                          ((frame.data[0] == 0xff && frame.data[1] == 0xff) ||
                           (frame.data[0] == 0x80 && frame.data[1] == 0x80))
                        : frame.dlc == 6 && (frame.data[0] >> 5) == 5 &&
                          frame.data[1] == query.request.data[1];
                    if (match) {
                        query.reply = frame;
                        query.done = true;
                        query.active = false;
                        query_changed.notify_all();
                        break;
                    }
                }
            }
        }

        RV_can_data_repack(&Rx_Message[slave], comm_ack, Rx_Motor_Msg[slave], slave, isConfig[slave]);
        if (motor_bus_ready.load())
            recordMotorFeedback(slave, Rx_Message[slave], Rx_Motor_Msg[slave]);

        if (isConfig[slave])
        {
            isConfig[slave] = false;
        }
    }
}

/**
 * @brief 组织本周期各从站的 TxPDO 输出控制帧。
 * @details 直接控制帧优先于普通队列消息；没有直接控制帧时，从对应从站队列取消息
 *          并按 passage 填入电机通道。若存在查询请求，则用单独的查询帧覆盖该从站
 *          本周期输出，避免查询报文与混合控制报文冲突，最后写入 SOEM 输出映射区。
 */
// 使用命令行控制电机的例程
void EtherCAT_Command_Set()
{
    static int state[SLAVE_NUMBER];
    cycle_direct_serial = 0;
    cycle_frame = {};
    cycle_phase = "idle";
    for (int slave = 0; slave < ec_slavecount && slave < SLAVE_NUMBER; ++slave)
    {
        bool direct = false;
        std::uint64_t serial = 0;
        if (slave == 0) {
            std::lock_guard<std::mutex> lock(direct_mutex);
            direct = direct_active || direct_pending;
            if (direct) {
                // 整帧替换，控制模式变化时不保留其他槽的旧指令。
                Tx_Message[slave] = direct_frame;
                cycle_phase = direct_phase;
                serial = direct_serial;
            }
        }
        Queue_Msg_ptr msg;
        if (direct) state[slave] = 0;
        else if (state[slave] == 0)
        {
            if (messages[slave].pop(msg))
            {
                if (Tx_Message[slave].can_ide != msg->can_ide) Tx_Message[slave] = {};
                Tx_Message[slave].can_ide = msg->can_ide;
                Tx_Message[slave].motor[msg->passage - 1] = msg->motor;
                for (int k = 1; k < msg->count; ++k)
                    Tx_Message[slave].motor[msg->passage + k - 1] = msg->following[k - 1];
                state[slave] = msg->stream ? 0 : 1;
            }
        }
        else if (state[slave]++ == 10)
        {
            isConfig[slave] = true;
            state[slave] = 0;
        }

        EtherCAT_Msg output = Tx_Message[slave];
        {
            std::lock_guard<std::mutex> lock(query_mutex);
            if (query.active && query.slave == slave) {
                output = {}; // 查询帧不能混入 can_ide=1 的混合控制报文。
                output.motor[query.passage - 1] = query.request;
                query.sent = true;
                if (slave == 0) cycle_phase = "sample";
                serial = 0;
            }
        }
        EtherCAT_Msg* slave_dest = (EtherCAT_Msg*)(ec_slave[slave + 1].outputs);
        if (slave_dest && ec_slave[slave + 1].Obytes == sizeof(EtherCAT_Msg)) {
            *slave_dest = output;
            if (slave == 0) cycle_frame = output;
            if (serial) {
                std::lock_guard<std::mutex> lock(direct_mutex);
                cycle_direct_serial = serial;
                if (serial == direct_serial) direct_pending = false;
            }
        }
    }
}
/**
 * @brief 后台周期线程入口，按 PeriodicClock 节拍执行 PDO 收发。
 * @details 从站处于 OP 时调用完整 EtherCAT_Run()；进入 OP 前只交换 PDO 并更新 WKC，
 *          同时以 initializing 阶段调用观察回调，不累计运行期 WKC 故障。
 */
void runImpl()
{
    PeriodicClock cycle(1000);
    while (running)
    {
        cycle.wait();
        if (inOP) EtherCAT_Run();
        else { // 启动阶段仅交换 PDO，不累计运行期 WKC 故障。
            ec_send_processdata();
            wkc = ec_receive_processdata(EC_TIMEOUTRET);
            if (cycle_observer) cycle_observer(EtherCAT_Msg{}, "initializing", cycle_observer_context);
        }
        cycle.next();
    }
}

// 启动 EtherCAT 收发线程和状态检查线程
void startRun()
{
    running = true;
    checking = true;
    runThread = std::thread(runImpl);
    checkThread = std::thread(ecatcheck);
}
 /**
 * @brief 清理反馈/控制缓存并启动 EtherCAT 总线。
 * @return 总线已经处于运行状态或本次初始化成功进入 OP 时返回 1，否则返回 0。
 * @note 当前实现将网卡名称固定为 "enp4s0"。
 */
int startBus()
{
    if (running) return inOP ? 1 : 0;
    clearMotorFeedback();
    for (int slave = 0; slave < SLAVE_NUMBER; ++slave) {
        Tx_Message[slave] = {};
        Rx_Message[slave] = {};
    }
    {
        std::lock_guard<std::mutex> lock(direct_mutex);
        direct_frame = {};
        direct_phase = "idle";
        direct_active = direct_pending = false;
        direct_serial = direct_completed = cycle_direct_serial = 0;
    }
    EtherCAT_Init((char*)"eno1");
    return inOP ? 1 : 0;
}
/**
 * @brief 停止 EtherCAT 后台线程，唤醒等待者并关闭 SOEM 主站。
 * @details 先清除运行标志，再通知直接控制和查询等待线程，等待两个后台线程退出，
 *          最后清除总线状态并调用 ec_close() 释放 EtherCAT 通信资源。
 */
void stopBus()
{
    {
        std::scoped_lock lock(direct_mutex, query_mutex);
        running = false;
        checking = false;
    }
    direct_changed.notify_all();
    query_changed.notify_all();
    if (runThread.joinable()) runThread.join();
    if (checkThread.joinable()) checkThread.join();
    inOP = FALSE;
    motor_bus_ready = false;
    ec_close();
}
