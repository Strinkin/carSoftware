#include <iostream>
#include <fstream>

#include "printFileContent.h"

void printFileContent(const std::string& filePath) {
    std::ifstream file(filePath);  // 打开文件
    if (!file.is_open()) {
        std::cerr << "无法打开文件: " << filePath << std::endl;
        return;
    }

    std::string line;
    while (std::getline(file, line)) {  // 逐行读取
        std::cout << line << std::endl;  // 打印每一行
    }

    file.close();  // 关闭文件
}