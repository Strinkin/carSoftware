#include <fstream>
#include <sstream>
#include <string>
#include <map>

std::map<std::string, std::string> parseKeyValueFile(const std::string& filePath) {
    std::ifstream file(filePath);
    std::map<std::string, std::string> data;  // 存储键值对
    std::string line;

    while (std::getline(file, line)) {
        std::istringstream iss(line);
        std::string key, value;
        
        // 按 ":" 分割键和值
        if (std::getline(iss, key, ':') && std::getline(iss, value)) {
            // 去除 value 的前导空格（如果有）
            value.erase(0, value.find_first_not_of(" \t"));
            data[key] = value;
        }
    }

    file.close();
    return data;
}