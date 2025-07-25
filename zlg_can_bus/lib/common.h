#ifndef COMMON_H_
#define COMMON_H_

#include "network.h"
#include <chrono>
#include <regex>
#include <thread>

#ifdef WIN32
#define snprintf _snprintf
#endif

#define cc_sleep(ms) std::this_thread::sleep_for(std::chrono::milliseconds(ms))

// Set the minimum timer resolution for an application or device driver.
void cc_time_begin_period(int ms);

// Clear a previously set minimum timer resolution.
void cc_time_end_period(int ms);

// set thread prio
bool thread_set_priority(std::thread &thread, int prio);

std::string GetHexString(const unsigned char *data, size_t len);
std::string GetAppPathA();

std::string UpperString(std::string &str);

//�ַ����и�
std::vector<std::string> cc_split(const std::string &str,
                                  const std::string &delim);
#endif
