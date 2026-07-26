/**
 * @file AppConfig.cpp
 * @brief 应用配置与数据目录校验实现。
 *        Application configuration and data-directory validation.
 */

#include "AppConfig.h"
#include <cstdlib>
#include <iostream>
#include <fstream>
#include <filesystem>
#include <sstream>
#include <vector>
#include <nlohmann/json.hpp>

namespace
{
const std::string DEFAULT_DATA_DIR = "data";

std::string strip_trailing_slash(std::string path)
{
    while (path.size() > 1 && path.back() == '/')
    {
        path.pop_back();
    }
    return path;
}

bool is_same_or_child_path(const std::filesystem::path& parent, const std::filesystem::path& child)
{
    std::string parent_str = strip_trailing_slash(parent.lexically_normal().generic_string());
    std::string child_str = strip_trailing_slash(child.lexically_normal().generic_string());

    if (parent_str == child_str)
    {
        return true;
    }

    return child_str.size() > parent_str.size()
        && child_str.compare(0, parent_str.size(), parent_str) == 0
        && child_str[parent_str.size()] == '/';
}

std::string decode_mount_path(const std::string& value)
{
    std::string decoded;
    decoded.reserve(value.size());

    for (size_t i = 0; i < value.size(); ++i)
    {
        if (value[i] == '\\' && i + 3 < value.size()
            && value[i + 1] >= '0' && value[i + 1] <= '7'
            && value[i + 2] >= '0' && value[i + 2] <= '7'
            && value[i + 3] >= '0' && value[i + 3] <= '7')
        {
            int ch = (value[i + 1] - '0') * 64
                + (value[i + 2] - '0') * 8
                + (value[i + 3] - '0');
            decoded.push_back(static_cast<char>(ch));
            i += 3;
        }
        else
        {
            decoded.push_back(value[i]);
        }
    }

    return decoded;
}

bool requires_real_mount(const std::filesystem::path& path)
{
    std::string normalized = strip_trailing_slash(path.lexically_normal().generic_string());
    return normalized == "/media"
        || normalized == "/mnt"
        || normalized.rfind("/media/", 0) == 0
        || normalized.rfind("/mnt/", 0) == 0
        || normalized.rfind("/run/media/", 0) == 0;
}

bool has_non_root_mount_ancestor(const std::filesystem::path& path)
{
    std::ifstream mountinfo("/proc/self/mountinfo");
    if (!mountinfo.good())
    {
        std::cerr << "[AppConfig] 无法读取 /proc/self/mountinfo，无法确认外置盘挂载状态" << std::endl;
        return false;
    }

    std::string line;
    while (std::getline(mountinfo, line))
    {
        std::istringstream iss(line);
        std::string id;
        std::string parent_id;
        std::string major_minor;
        std::string root;
        std::string mount_point;

        if (!(iss >> id >> parent_id >> major_minor >> root >> mount_point))
        {
            continue;
        }

        std::filesystem::path mounted_at = decode_mount_path(mount_point);
        if (mounted_at == "/")
        {
            continue;
        }

        if (is_same_or_child_path(mounted_at, path))
        {
            return true;
        }
    }

    return false;
}

std::string read_configured_data_base_dir()
{
    std::string configured = DEFAULT_DATA_DIR;
    std::filesystem::path config_path;
    std::vector<std::filesystem::path> candidates;

    if (const char* config_directory = std::getenv("GIMBALED_TRACKING_CONFIG_DIR"))
    {
        if (*config_directory != '\0')
        {
            candidates.emplace_back(std::filesystem::path(config_directory) / "app_config.json");
        }
    }
    candidates.emplace_back("config/app_config.json");
    candidates.emplace_back("../config/app_config.json");

    for (const auto& candidate : candidates)
    {
        std::ifstream config_file(candidate);
        if (config_file.good())
        {
            config_path = candidate;
            try
            {
                nlohmann::json config = nlohmann::json::parse(config_file);
                configured = config.value("data_base_dir", configured);
            }
            catch (const std::exception& e)
            {
                std::cerr << "[AppConfig] 解析配置文件失败 " << candidate << ": "
                          << e.what() << "，使用默认数据目录" << std::endl;
            }
            break;
        }
    }

    std::filesystem::path data_path(configured);
    if (data_path.is_relative())
    {
        std::filesystem::path project_directory = std::filesystem::current_path();
        if (!config_path.empty())
        {
            project_directory =
                std::filesystem::absolute(config_path).parent_path().parent_path();
        }
        data_path = project_directory / data_path;
    }

    return strip_trailing_slash(
        std::filesystem::absolute(data_path).lexically_normal().generic_string());
}
}

bool resolve_data_base_dir(std::string& data_base_dir, std::string& error_message)
{
    data_base_dir.clear();
    error_message.clear();

    std::string configured = read_configured_data_base_dir();
    if (configured.empty())
    {
        error_message = "配置的数据目录为空";
        return false;
    }

    std::filesystem::path configured_path(configured);
    if (requires_real_mount(configured_path) && !has_non_root_mount_ancestor(configured_path))
    {
        error_message = "配置的数据目录不在有效挂载点下: " + configured
            + "，请确认外置盘已挂载到该路径后再启动保存";
        return false;
    }

    data_base_dir = configured;
    return true;
}

std::string get_data_base_dir()
{
    static std::string cached_dir;
    static bool initialized = false;

    if (!initialized)
    {
        initialized = true;
        cached_dir = read_configured_data_base_dir();
        std::cout << "[AppConfig] 数据基目录: " << cached_dir << std::endl;
    }

    return cached_dir;
}
