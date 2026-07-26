/**
 * @file AppConfig.h
 * @brief 应用配置与数据目录校验接口。
 *        Application configuration and data-directory validation.
 */

#pragma once
#include <string>

/// 校验并解析数据目录 / Validate and resolve the data directory.
bool resolve_data_base_dir(std::string& data_base_dir, std::string& error_message);

/// 读取配置的数据目录 / Read the configured data directory.
std::string get_data_base_dir();
