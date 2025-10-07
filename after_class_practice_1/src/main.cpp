#include <iostream>
#include <string>
#include <fstream>
#include <map>
#if defined(_WIN32) || defined(_WIN64)
#include <conio.h>  // 用于 _getch()（Windows）
#else
#include <termios.h>
#include <unistd.h>
char _getch() {
    struct termios oldt, newt;
    char ch;
    tcgetattr(STDIN_FILENO, &oldt);
    newt = oldt;
    newt.c_lflag &= ~(ICANON | ECHO);
    tcsetattr(STDIN_FILENO, TCSANOW, &newt);
    ch = getchar();
    tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
    return ch;
}
#endif
#include "carInfo.h"
#include "studentInfo.h"
#include "addFixedParam.h"
#include "carInfoUtils.h"
#include "studentInfoUtils.h"
#include "printFileContent.h"
#include "parseKeyValue.h"

using namespace std;

int main(int argc, char* argv[]) {
    cout << "1: 写入数据 2: 读取数据" << endl;
    int choice;
    cin >> choice;
    string save_dic_path = "../data/relation.txt";
    string save_car_json_path = "../data/car/";
    string save_student_json_path = "../data/student/";
    if (choice == 1) {
        // 1、共采购了10台，请先分别完成10台小车的信息录入，并完成编号。
        CarInfo cars[10];
        for (int i = 0; i < 10; ++i) {
            string car_serial_number = "cqusn" + to_string(10000000000000 + i);         // 自动生成编号: cqusn + 14位数字
            string chassis_serial_number = "dp" + to_string(10000000 + i);              // 自动生成底盘编号: dp + 8位数字
            addCarParam(cars[i], car_serial_number, chassis_serial_number);             // 添加小车参数
            saveCarInfoJson(cars[i], save_car_json_path + cars[i].serial_number + ".json");   // 保存为JSON文件
        }
        cout << "已录入10台小车信息并保存为JSON文件: " + save_car_json_path << endl;

        // 2、根据编号，将每台小车分配给每名同学
        StudentInfo students[10];
        for (int i = 0; i < 10; ++i) {
            string id = "2025000" + to_string(i);
            string name = "小朋友" + to_string(i);
            addStudentParam(students[i], id, name);
            saveStudentInfoJson(students[i], save_student_json_path + students[i].id + ".json");   // 保存为JSON文件
        }
        cout << "已录入10名同学信息并保存为JSON文件: " + save_student_json_path << endl;
        
        // 分配将每台小车分配给每名同学
        {   // 只清空，不写入
            std::ofstream ofs(save_dic_path, std::ios::trunc);
            if (!ofs) throw std::runtime_error("Failed to open file: " + save_dic_path);
            ofs.close();  
        }
        for (int i = 0; i < 10; ++i) {
            std::ofstream ofs(save_dic_path, std::ios::app);
            if (!ofs) throw runtime_error("Failed to open file: " + save_dic_path);
            ofs << students[i].id << ": "<< cars[i].serial_number << "\n";
            ofs.close();
        }
        cout << "已完成小车与同学的分配，并保存分配记录: " + save_dic_path << endl;
    }
    else if (choice == 2) {
        map<std::string, std::string> map = parseKeyValueFile(save_dic_path);
        if (map.empty()) {
            cout << "没有分配记录。" << endl;
        } else {
            auto it = map.begin();
            while (true) {
                // 显示当前学生和小车信息
                cout << "学生信息: " << endl;
                printFileContent(save_student_json_path + it->first + ".json");
                cout << "该学生分配到的小车的信息: " << endl;
                printFileContent(save_car_json_path + it->second + ".json");

                cout << ">>>按 n (下一个) | p (上一个) | q (退出): ";
                char input = _getch();
                if (input == 'n') {
                    auto next = it;
                    ++next;
                    if (next != map.end()) {
                        it = next;
                    } else {
                        cout << "已经是最后一个记录。" << endl;
                    }
                } else if (input == 'p') {
                    if (it != map.begin()) {
                        --it;
                    } else {
                        cout << "已经是第一个记录。" << endl;
                    }
                } else if (input == 'q') {
                    break;
                }
            }
        }
    }
    return 0;
}