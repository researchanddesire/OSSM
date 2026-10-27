#include "ossmHatEmergencyInternal.h"
#include "ossmHatEmergencyLogic.hpp"

#include <Arduino.h>
#include <cmath>
#include <driver/periph_ctrl.h>
#include <driver/uart.h>
#include <esp_intr_alloc.h>
#include <esp_spi_flash.h>
#include <hal/uart_ll.h>
#include <soc/gpio_struct.h>
#include <soc/gpio_sig_map.h>
#include <soc/timer_group_struct.h>
#include <soc/uart_struct.h>
#include <soc/soc.h>
#include <soc/dport_access.h>
#include <soc/dport_reg.h>
#include <esp32/rom/gpio.h>
#include <rom/ets_sys.h>
#include "AimFrame.h"
#include "constants/Pins.h"

// Pinned ESP-IDF4.4.7 cache_utils.h (component-private header). These existing
// IRAM entry points also back the flash driver; no erase/write is performed.
extern "C" void spi_flash_disable_interrupts_caches_and_other_cpu();
extern "C" void spi_flash_enable_interrupts_caches_and_other_cpu();

namespace ossm_hat_emergency {
namespace {
constexpr uint32_t gateConfiguration=1, gatePossible=2, gateIsolated=4,
                   gateProcedure=8, gateRestoring=16;
constexpr uint32_t rxMask=UART_INTR_RXFIFO_FULL|UART_INTR_RXFIFO_TOUT;
constexpr uint32_t errorMask=UART_INTR_RXFIFO_OVF|UART_INTR_FRAM_ERR|UART_INTR_PARITY_ERR;
constexpr uint32_t ringCapacity=512;

// Every byte/data object reached by the raw IRQ lives in internal DRAM.
DRAM_ATTR uint8_t offFrame[8]={1,6,0,1,0,0,0xD8,0x0A};
DRAM_ATTR uint8_t rxRing[ringCapacity];
DRAM_ATTR volatile uint32_t rxHead=0, rxTail=0, rxOverflow=0;
DRAM_ATTR volatile uint32_t routeGate=0, possible=0, faultRequested=0;
DRAM_ATTR volatile uint32_t guardAt=0, telemetryAt=0, stopAt=0, stopPending=0;
DRAM_ATTR volatile uint32_t lastRxAt=0, auxiliaryTx=0;
DRAM_ATTR volatile uint32_t completionPending=0, auxiliaryAt=0;
DRAM_ATTR volatile uint32_t quietResetRequested=0;
DRAM_ATTR Deadline deadline;
DRAM_ATTR volatile uint32_t installed=0, available=0, lease=0, parked=0;
DRAM_ATTR volatile uint32_t isolated=0, emitted=0, done=0, failedFlag=0;
DRAM_ATTR volatile uint32_t epoch=0, restoredEpoch=0;
DRAM_ATTR volatile uint32_t tripAt=0, isolationAt=0, emittedAt=0, doneAt=0;
DRAM_ATTR volatile uint32_t cacheAtTrip=0, cacheAtTx=0, rawOffWrites=0;
DRAM_ATTR volatile uint32_t echoVerified=0, offVerified=0, quietVerified=0;
DRAM_ATTR volatile uint32_t echoAt=0, offAt=0, quietAt=0;
DRAM_ATTR volatile uint32_t restoring=0, restored=0;
DRAM_ATTR volatile uint32_t offOnlyPassed=0, cacheOffPassed=0, qualificationPassed=0;
DRAM_ATTR volatile uint32_t lastSampleAt=0, sampleSequence=0, sampleContinuity=0;
DRAM_ATTR volatile uint32_t publishedCause=0;
intr_handle_t timerInterrupt=nullptr, uartInterrupt=nullptr;
QuietProof quietProof;

// A finite hardware-update wait, unlike timer_ll_get_counter_value's open
// loop. All deadline arithmetic uses this timer's native32-bit microseconds.
void IRAM_ATTR captureCounter(uint32_t &low, uint32_t &high) {
    auto &timer=TIMERG1.hw_timer[1];
    timer.update=1;
    for (unsigned i=0; i<32 && timer.update; ++i) asm volatile("nop");
    if (timer.update) failedFlag=1;
    high=timer.cnt_high;
    low=timer.cnt_low;
    // Another core may update the shared capture while this core reads it.
    // Reconcile the high word at the71-minute low-word wrap before rearming.
    for (unsigned i=0;i<4;++i) {
        const uint32_t nextHigh=timer.cnt_high;
        if (nextHigh==high) return;
        high=nextHigh; low=timer.cnt_low;
    }
    failedFlag=1;
}
uint32_t IRAM_ATTR nowUs() {
    uint32_t low,high;
    captureCounter(low,high);
    return low;
}

void IRAM_ATTR directionLow() { UART1.conf0.sw_rts=1; }

bool IRAM_ATTR ownCacheDisabled() {
    // Installation is restricted to APP/core1. Checking only "either CPU
    // disabled" would falsely credit an interrupt running with its own cache
    // still enabled while the OTHER CPU was entering a flash operation.
    return (DPORT_REG_READ(DPORT_APP_CACHE_CTRL_REG)&DPORT_APP_CACHE_ENABLE)==0;
}

void IRAM_ATTR completeTx(uint32_t now) {
    directionLow(); completionPending=0;
    if (auxiliaryTx) auxiliaryTx=0;
    else if (deadline.sent(now,true)) { done=1; doneAt=now; }
}

void IRAM_ATTR fence() {
    // No task lock, driver call or FIFO purge. Both peripheral outputs move
    // away from UART2; its old ISR/FIFO writes cannot reach the transceiver.
    directionLow();
    gpio_matrix_out(Pins::Hat::direction, U1RTS_OUT_IDX, false, false);
    gpio_matrix_out(Pins::Hat::tx, U1TXD_OUT_IDX, false, false);
    routeGate=gateIsolated;
    isolated=1;
}

void IRAM_ATTR drainRx(uint32_t now) {
    // FIFO capacity bounds work per IRQ even if noise is continuous.
    const unsigned count=UART1.status.rxfifo_cnt;
    for (unsigned i=0; i<count && i<128; ++i) {
        uint8_t value=0;
        uart_ll_read_rxfifo(&UART1, &value, 1);
        lastRxAt=now;
        if (isolated && !auxiliaryTx && deadline.phase != Phase::Transmit &&
            deadline.phase != Phase::Setup) {
            const uint32_t next=(rxHead+1)&(ringCapacity-1);
            if (next==rxTail) ++rxOverflow;
            else { rxRing[rxHead]=value; asm volatile("memw" ::: "memory"); rxHead=next; }
        }
    }
}

void IRAM_ATTR uartIsr(void *) {
    uint32_t now,high;
    captureCounter(now,high);
    const uint32_t flags=UART1.int_st.val;
    if (flags & errorMask) {
        if (flags & UART_INTR_RXFIFO_OVF) ++rxOverflow;
        // UART framing glitches are not replies. The task still requires an
        // exact CRC-valid remote echo and complete CRC-valid full reads; the
        // known leading-null RX transition must not masquerade as FIFO loss.
        UART1.int_clr.val=flags&errorMask;
    }
    if (flags & rxMask) { drainRx(now); UART1.int_clr.val=flags&rxMask; }
    if (flags & UART_INTR_TX_DONE) {
        UART1.int_ena.val &= ~UART_INTR_TX_DONE;
        UART1.int_clr.val=UART_INTR_TX_DONE;
        // ESP32 TX_DONE can precede actual shift-register idle. A bounded
        // <=100us final-character check avoids both a truncated stop bit and
        // an unmasked level3 interrupt storm starving the timer deadline.
        const uint32_t began=nowUs();
        for (unsigned i=0;i<2000 && !uart_ll_is_tx_idle(&UART1) && nowUs()-began<100;++i)
            asm volatile("nop");
        if (uart_ll_is_tx_idle(&UART1)) completeTx(nowUs());
        else completionPending=1; // timer timeout remains independent
    }
}

void IRAM_ATTR timerIsr(void *) {
    TIMERG1.int_clr_timers.val=2;
    uint32_t now,high;
    captureCounter(now,high);
    auto &timer=TIMERG1.hw_timer[1];
    const uint32_t next=now+1000;
    timer.alarm_high=high+(next<now ? 1U:0U);
    timer.alarm_low=next;
    timer.config.alarm_en=1;
    drainRx(now);
    if (completionPending && uart_ll_is_tx_idle(&UART1)) completeTx(now);
    if (auxiliaryTx && now-auxiliaryAt>txTimeoutUs) {
        directionLow(); auxiliaryTx=completionPending=0; failedFlag=1;
    }
    Inputs in;
    in.outputPossible=possible!=0; in.fault=faultRequested!=0;
    in.stopPending=stopPending!=0; in.guardAt=guardAt;
    in.telemetryAt=telemetryAt; in.stopAt=stopAt; in.lastRxAt=lastRxAt;
    in.rxIdle=UART1.status.st_urx_out==0 && UART1.status.rxfifo_cnt==0;
    in.txIdle=uart_ll_is_tx_idle(&UART1);
    const uint32_t actions=deadline.tick(now, in);
    if (actions & Isolate) {
        // A genuine later fault starts a new exchange too. Retained receipts
        // from a successful qualification may never authorize early reads or
        // report its old echo as the new emergency's acknowledgement.
        emitted=done=echoVerified=offVerified=quietVerified=0;
        emittedAt=doneAt=echoAt=offAt=quietAt=0;
        cacheAtTx=0; auxiliaryTx=completionPending=0;
        restored=restoring=qualificationPassed=0;
        GPIO.out_w1ts=1UL<<Pins::Driver::motorEnablePin; // uncredited best effort
        fence(); ++epoch; tripAt=isolationAt=now;
        publishedCause=static_cast<uint32_t>(deadline.cause);
        cacheAtTrip=ownCacheDisabled();
        lastRxAt=now;
        rxTail=rxHead;
    }
    if (actions & EnableDirection) UART1.conf0.sw_rts=0;
    if (actions & WriteOff) {
        // Exactly this fixed off command. No generic register/target writer
        // can be reached by either raw ISR. TX FIFO was verified empty.
        UART1.int_clr.val=UART_INTR_TX_DONE;
        UART1.int_ena.val |= UART_INTR_TX_DONE;
        cacheAtTx=ownCacheDisabled();
        uart_ll_write_txfifo(&UART1, offFrame, sizeof(offFrame));
        emitted=1; emittedAt=now; ++rawOffWrites;
    }
    if (actions & DisableDirection) { directionLow(); failedFlag=1; }
}

bool configureUart1() {
    if (uart_is_driver_installed(UART_NUM_1)) return false;
    periph_module_enable(PERIPH_UART1_MODULE);
    periph_module_reset(PERIPH_UART1_MODULE);
    UART1.int_ena.val=0; UART1.int_clr.val=0xFFFFFFFF;
    UART1.conf0.val=0;
    UART1.conf0.bit_num=3; UART1.conf0.stop_bit_num=1;
    uart_ll_set_sclk(&UART1, UART_SCLK_APB);
    uart_ll_set_baudrate(&UART1, 115200);
    uart_ll_set_mode_rs485_half_duplex(&UART1);
    UART1.conf1.rxfifo_full_thrhd=1;
    UART1.conf1.rx_tout_thrhd=2; UART1.conf1.rx_tout_en=1;
    directionLow();
    gpio_matrix_in(Pins::Hat::rx, U1RXD_IN_IDX, false);
    if (!uart_ll_is_tx_idle(&UART1) || UART1.status.txfifo_cnt) return false;
    const auto result=esp_intr_alloc(ETS_UART1_INTR_SOURCE,
        ESP_INTR_FLAG_LEVEL3|ESP_INTR_FLAG_IRAM, uartIsr, nullptr, &uartInterrupt);
    if (result!=ESP_OK) return false;
    UART1.int_ena.val=rxMask|errorMask;
    return true;
}

// All task-side mutations of the ISR-owned state are made by the pinned
// UART owner on the very core on which the raw interrupts were allocated.
uint32_t pauseLocal() { return portSET_INTERRUPT_MASK_FROM_ISR(); }
void resumeLocal(uint32_t saved) { portCLEAR_INTERRUPT_MASK_FROM_ISR(saved); }

void constantFence() {
    GPIO.out_w1tc=1UL<<Pins::Hat::direction;
    GPIO.out_w1ts=1UL<<Pins::Hat::tx;
    GPIO.enable_w1ts=(1UL<<Pins::Hat::direction)|(1UL<<Pins::Hat::tx);
    gpio_matrix_out(Pins::Hat::direction, SIG_GPIO_OUT_IDX, false, false);
    gpio_matrix_out(Pins::Hat::tx, SIG_GPIO_OUT_IDX, false, false);
}
} // namespace

bool installHardware() {
    if (installed) return available;
    if (xPortGetCoreID()!=1) return false;
    // This contract pins APB80MHz and disallows runtime power-management
    // changes. A PM-enabled profile must supply an independently reviewed
    // APB/no-light-sleep lease rather than silently changing this clock.
#ifdef CONFIG_PM_ENABLE
    return false;
#endif
    if (getApbFrequency()!=80000000U || !configureUart1()) return false;
    periph_module_enable(PERIPH_TIMG1_MODULE); // never reset the group's WDT
    auto &timer=TIMERG1.hw_timer[1];
    timer.config.val=0; timer.config.divider=80; timer.config.increase=1;
    timer.load_low=0; timer.load_high=0; timer.reload=1;
    timer.alarm_low=1000; timer.alarm_high=0;
    TIMERG1.int_clr_timers.val=2;
    if (esp_intr_alloc(ETS_TG1_T1_LEVEL_INTR_SOURCE,
        ESP_INTR_FLAG_LEVEL3|ESP_INTR_FLAG_IRAM, timerIsr, nullptr,
        &timerInterrupt)!=ESP_OK) {
        UART1.int_ena.val=0; esp_intr_free(uartInterrupt); uartInterrupt=nullptr;
        return false;
    }
    TIMERG1.int_ena.val |= 2;
    timer.config.level_int_en=1; timer.config.alarm_en=1; timer.config.enable=1;
    guardAt=telemetryAt=nowUs(); installed=available=1;
    return true;
}

void ownerLease(bool exclusive) { lease=exclusive; }
bool ordinaryTxAllowed() { return !isolated && !faultRequested && !(routeGate&gateProcedure); }
bool configurationAllowed() { return routeGate==0 || routeGate==gateRestoring; }
bool restoreVetoed() { return faultRequested!=0; }
bool protocolFailed() { return failedFlag!=0; }
bool beginConfiguration() {
    uint32_t expected=restoring ? gateRestoring:0;
    return __atomic_compare_exchange_n(&routeGate, &expected,
        expected|gateConfiguration, false, __ATOMIC_SEQ_CST, __ATOMIC_SEQ_CST);
}
void endConfiguration() { routeGate=restoring ? gateRestoring:0; }
void ownerParked(bool value) { parked=value; }

bool beginTest(OssmHatEmergencyCause cause) {
    if (!available || !lease || !parked || faultRequested || failedFlag || !expected(cause)) return false;
    uint32_t expectedGate=possible ? gatePossible:0;
    if (!__atomic_compare_exchange_n(&routeGate, &expectedGate,
        expectedGate|gateProcedure, false, __ATOMIC_SEQ_CST, __ATOMIC_SEQ_CST)) return false;
    const auto mask=pauseLocal();
    emitted=done=echoVerified=0;
    rxOverflow=0;
    tripAt=isolationAt=emittedAt=doneAt=echoAt=offAt=quietAt=0;
    cacheAtTrip=cacheAtTx=0;
    const bool ok=deadline.schedule(cause, nowUs());
    restored=0; offVerified=quietVerified=0; quietProof.reset();
    resumeLocal(mask);
    return ok;
}

void IRAM_ATTR cacheOffWindow() {
    // This pinned entry point acquires the SAME spi_flash_op_lock used by
    // normal flash/NVS operations (cache_utils.c:136); the paired enable
    // releases it (:220). Do not add another private or nested mutex here.
    spi_flash_disable_interrupts_caches_and_other_cpu();
    const uint32_t began=nowUs();
    // Finite loop, even if the hardware time base fails. The cache-disabled
    // path reaches only IRAM/MMIO/DRAM, including both raw ISR chains.
    for (uint32_t i=0; i<1000000 && static_cast<uint32_t>(nowUs()-began)<12000; ++i)
        asm volatile("nop");
    spi_flash_enable_interrupts_caches_and_other_cpu();
}

size_t receive(uint8_t *out, size_t capacity) {
    size_t count=0;
    while (count<capacity && rxTail!=rxHead) {
        out[count++]=rxRing[rxTail];
        asm volatile("memw" ::: "memory");
        rxTail=(rxTail+1)&(ringCapacity-1);
    }
    return count;
}

bool readFullRegisters() {
    if (!verificationReadAllowed(deadline.phase,isolated,done,echoVerified,auxiliaryTx,failedFlag) ||
        !uart_ll_is_tx_idle(&UART1) || UART1.status.st_urx_out!=0 ||
        static_cast<uint32_t>(nowUs()-lastRxAt)<quietUs) return false;
    uint8_t request[8];
    AimFrame::buildRead(request, 1, 0, 26);
    const auto mask=pauseLocal();
    auxiliaryTx=1;
    auxiliaryAt=nowUs();
    UART1.conf0.sw_rts=0;
    ets_delay_us(10); // bounded transceiver setup, not a scheduler wait
    // The task's preconfigured UART can transmit read-only verification.
    // Its IRQ still owns final-bit-safe DE release. No driver/ring is used.
    UART1.int_clr.val=UART_INTR_TX_DONE;
    UART1.int_ena.val |= UART_INTR_TX_DONE;
    uart_ll_write_txfifo(&UART1, request, sizeof(request));
    resumeLocal(mask);
    return true;
}

void acknowledgeEcho(uint32_t now) {
    const auto mask=pauseLocal();
    if (deadline.echoed()) { echoVerified=1; echoAt=now; }
    resumeLocal(mask);
}

void observe(const OssmHatTelemetry &sample, const OssmHatControlSnapshot &controls) {
    if (!sample.sequence || (sample.sequence==sampleSequence && sample.continuity==sampleContinuity) ||
        !controls.valid || controls.continuity!=sample.continuity ||
        controls.updatedAtMs!=sample.updatedAtMs) return;
    sampleSequence=sample.sequence; sampleContinuity=sample.continuity;
    lastSampleAt=sample.updatedAtMs; telemetryAt=nowUs();
    const bool off=controls.modbusEnable==1 && !(controls.outputEnable&1);
    const bool healthy=off && (sample.alarm==0 || sample.alarm==0x10) && std::isfinite(sample.currentAmps) &&
        sample.currentAmps<0.5f && sample.currentAmps>=0 &&
        std::isfinite(sample.voltageVolts) && sample.voltageVolts>10;
    if (controls.modbusEnable==1 && (controls.outputEnable&1) &&
        lease && !isolated && !(routeGate&gateConfiguration)) {
        // An unexpected enabled drive is an obligation too; never rely on
        // the guard's armed/disarmed state to remember actuator lifetime.
        if (!possible) { quietResetRequested=1; possible=1; routeGate=gatePossible; }
    }
    if (quietResetRequested) { quietProof.reset(); quietResetRequested=0; }
    if ((isolated && echoVerified) || restoring) {
        offVerified=off;
        if (off) offAt=sample.updatedAtMs;
    }
    if (off) stopPending=0; // fresh full readback satisfies the off deadline
    const bool quiet=quietProof.accept(sample.sequence,
        sample.continuity, sample.updatedAtMs, sample.positionCounts, healthy);
    quietVerified=quiet;
    if (quiet) {
        quietVerified=1; quietAt=sample.updatedAtMs;
        if (off) {
            possible=0; stopPending=0;
            uint32_t watching=gatePossible;
            __atomic_compare_exchange_n(&routeGate,&watching,0,false,
                                         __ATOMIC_SEQ_CST,__ATOMIC_SEQ_CST);
        }
        if (restoring && !faultRequested && !failedFlag) {
            restored=1; restoredEpoch=epoch; restoring=0; parked=0; routeGate=0;
            const auto reason=static_cast<OssmHatEmergencyCause>(publishedCause);
            if (reason==OssmHatEmergencyCause::OffOnlyPreflight) offOnlyPassed=1;
            if (reason==OssmHatEmergencyCause::CacheOffPreflight && cacheAtTrip && cacheAtTx)
                cacheOffPassed=1;
            if (reason==OssmHatEmergencyCause::Qualification) qualificationPassed=1;
        }
    }
}

void failed() {
    const auto mask=pauseLocal();
    failedFlag=1; faultRequested=1; deadline.vetoRestore=true;
    if (restoring) {
        // A fault can arrive after new UART2 pins were attached but before
        // restoration was credited. Reassert the constant physical fence;
        // an old isolated boolean alone must never describe that interval.
        constantFence(); isolated=1; routeGate=gateIsolated; restoring=0;
    }
    // A failure before/during the fixed off attempt must not suppress that
    // attempt. Only an already-completed/abandoned exchange is finalized here.
    if ((deadline.phase==Phase::Ready && !possible) ||
        deadline.phase==Phase::Echo || deadline.phase==Phase::Verify) {
        deadline.fail(); directionLow();
    }
    resumeLocal(mask);
}

bool beginRestore() {
    const auto mask=pauseLocal();
    const bool cacheValid=deadline.cause!=OssmHatEmergencyCause::CacheOffPreflight ||
        (cacheAtTrip && cacheAtTx);
    const bool good=!failedFlag && cacheValid && deadline.canRestore(parked, offVerified,
                                                                  quietVerified, faultRequested);
    if (good) { restoring=1; routeGate=gateRestoring; constantFence(); }
    resumeLocal(mask);
    return good;
}

bool resetUnderConstantFence() {
    if (!restoring || !parked || possible || !offVerified || !quietVerified ||
        faultRequested || !uart_ll_is_tx_idle(&UART1) || !uart_ll_is_tx_idle(&UART2) ||
        uart_is_driver_installed(UART_NUM_2)) return false;
    constantFence();
    UART1.int_ena.val=0;
    esp_intr_free(uartInterrupt); uartInterrupt=nullptr;
    // Separate peripheral resets only. DPORT_UART_MEM_RST would affect the
    // active UART0 console and is intentionally never touched.
    periph_module_reset(PERIPH_UART1_MODULE);
    periph_module_enable(PERIPH_UART2_MODULE);
    periph_module_reset(PERIPH_UART2_MODULE);
    rxHead=rxTail=0;
    if (!configureUart1() || !uart_ll_is_tx_idle(&UART2) || UART2.status.txfifo_cnt)
        return false;
    return true;
}

bool finishRestore() {
    if (!restoring || faultRequested || failedFlag) return false;
    const auto mask=pauseLocal();
    deadline=Deadline{}; isolated=0; auxiliaryTx=0;
    stopPending=0; possible=0;
    quietVerified=offVerified=0; quietProof.reset();
    sampleSequence=sampleContinuity=0;
    resumeLocal(mask);
    return true;
}
} // namespace ossm_hat_emergency

void ossmHatEmergencyGuardHeartbeat() {
    if (ossm_hat_emergency::installed) ossm_hat_emergency::guardAt=ossm_hat_emergency::nowUs();
}
bool ossmHatEmergencyMarkOutputPossible() {
    using namespace ossm_hat_emergency;
    if (!available || !lease || isolated || faultRequested || failedFlag || stopPending ||
        !offOnlyPassed || !cacheOffPassed) return false;
    uint32_t expected=0;
    if (possible) return routeGate==gatePossible;
    if (!__atomic_compare_exchange_n(&routeGate,&expected,gatePossible,false,
                                     __ATOMIC_SEQ_CST,__ATOMIC_SEQ_CST)) return false;
    const uint32_t now=nowUs();
    if (now-telemetryAt>40000 || now-guardAt>10000) {
        routeGate=0; return false;
    }
    offVerified=quietVerified=0; quietResetRequested=1; possible=1;
    return true;
}
void ossmHatEmergencyStopRequested() {
    using namespace ossm_hat_emergency;
    if (possible && !stopPending) { stopAt=nowUs(); stopPending=1; }
}
void ossmHatEmergencyFault() {
    using namespace ossm_hat_emergency;
    if (possible || isolated || parked) faultRequested=1;
}
void ossmHatEmergencyCancelProcedure() {
    ossm_hat_emergency::faultRequested=1;
}
OssmHatEmergencyStatus getOssmHatEmergencyStatus() {
    using namespace ossm_hat_emergency;
    OssmHatEmergencyStatus s;
    s.available=available; s.irqInstalled=installed; s.outputMayBeOn=possible;
    s.isolated=isolated; s.ownerParked=parked; s.transmitted=emitted; s.txDone=done;
    s.echoVerified=echoVerified; s.offVerified=offVerified; s.quietVerified=quietVerified;
    s.failed=failedFlag || faultRequested; s.restoring=restoring; s.restored=restored;
    s.offOnlyPassed=offOnlyPassed; s.cacheOffPassed=cacheOffPassed;
    s.qualificationPassed=qualificationPassed;
    s.cacheOffAtTrip=cacheAtTrip; s.cacheOffAtTx=cacheAtTx;
    s.cause=static_cast<OssmHatEmergencyCause>(publishedCause);
    s.epoch=epoch; s.restoredEpoch=restoredEpoch; s.tripAtUs=tripAt;
    s.nowUs=installed ? nowUs():0;
    s.capturedAtMs=millis();
    s.isolatedAtUs=isolationAt; s.transmittedAtUs=emittedAt; s.txDoneAtUs=doneAt;
    s.echoAtMs=echoAt; s.offAtMs=offAt; s.quietAtMs=quietAt;
    s.lastTelemetryAtMs=lastSampleAt; s.telemetrySequence=sampleSequence;
    s.telemetryContinuity=sampleContinuity; s.rawOffWrites=rawOffWrites;
    s.rxOverflows=rxOverflow;
    return s;
}
