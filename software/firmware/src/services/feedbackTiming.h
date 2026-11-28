#pragma once

#include <cstdint>

#include "feedback_timing.h"

// Timing evidence for the HAT's RS-485 shaft feedback on the millis() clock;
// see lib/OSSMLogic/src/feedback_timing.h and docs/ossm-hat.md. Diagnostic
// only: no freshness limit, stop or motion decision reads it.

// HAT reader task: once at start, then lock-free stamps in its loop.
void feedbackTimingReaderStarted();
void feedbackTimingMark(feedback_timing::Stage stage, uint32_t atMs);
void feedbackTimingPoint(feedback_timing::ReaderPoint point);
void feedbackTimingDrained(uint32_t bytes);

// Any task, for correlation with a stall.
void feedbackTimingMilestone(feedback_timing::Milestone milestone);

// Motor guard task: every tick, and when a fault latches.
void feedbackTimingObserve(uint32_t nowMs, bool feedbackExpected, uint32_t sampleAtMs);
void feedbackTimingRefused(uint32_t nowMs, uint8_t fault);

// Prints a RAD_FEEDBACK line for each stall that closed and for the refusal,
// once each. Call from a low-priority task; returns at once otherwise.
void feedbackTimingReportDue();
// The serial `feedback` command: the latest stall, and the refusal if any.
void feedbackTimingReportNow();
