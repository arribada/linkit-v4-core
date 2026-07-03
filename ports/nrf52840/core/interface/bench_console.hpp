/**
 * @file bench_console.hpp
 * @brief Bench-test serial console — autonomous hardware-in-the-loop validation hook.
 *
 * Compiled ONLY with -DBENCH_TEST=ON (debug bench builds); entirely absent from
 * production firmware. Provides a tiny '%'-prefixed command interpreter over the
 * USB-CDC link so a host driver can drive the device with no magnet, no antenna
 * and no operator:
 *
 *   %PING                       → handshake, replies "%BENCH OK state=<S>"
 *   %STATE                      → replies "%STATE <S>"
 *   %CFG                        → enter ConfigurationState (synthesised reed gesture)
 *   %OP                         → leave ConfigurationState back to Operational
 *   %GPS <lat> <lon> [hAcc_mm] [numSV]  → inject a synthetic 3D fix (no antenna)
 *
 * The '%' framing is disjoint from the DTE '$...#' framing, so bench commands and
 * DTE commands coexist on the same CDC stream. Debug logs also share the stream;
 * the host demuxes by the leading '%' on reply lines.
 */

#pragma once

#ifdef BENCH_TEST

#include <string>

namespace bench {

/// @brief Parse and execute one bench command line (leading '%').
/// @param line  Raw line as read from USB (trailing CR/LF tolerated).
/// @return true if the line was a bench command (started with '%') and was consumed.
bool handle_line(const std::string& line);

/// @brief Start the all-state USB poll task. Reads bench commands from USB-CDC in
/// every FSM state EXCEPT ConfigurationState (there the config USB poller owns the
/// RX and routes '%' lines back into handle_line). Call once after GenTracker::start().
void start_poll();

}  // namespace bench

#endif  // BENCH_TEST
