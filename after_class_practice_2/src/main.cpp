#include <iostream>
#include <vector>
#include <fstream>
#include <string>
#include <sstream>
#include <conio.h> // for _getch on Windows

#include "../include/carInfo.hpp"
#include "../include/studentInfo.hpp"
#include "../include/addFixedParam.h"
#include <iomanip>

using namespace std;

void printFileContent(const string &path) {
    ifstream ifs(path);
    if (!ifs) { cerr << "failed to open " << path << "\n"; return; }
    string line;
    while (getline(ifs, line)) {
        cout << line << "\n";
    }
    ifs.close();
}

int main() {
    cout << "== 信息录入 ==\n";
    vector<string> car_files;
    vector<string> stu_files;
    // create 10 car
    for (int i = 0; i < 10; ++i) {
        // 文件保存路径
        string fname_car = string("../after_class_practice_2/data/car_") + to_string(i) + string(".txt");
        string fname_stu = string("../after_class_practice_2/data/student_") + to_string(i) + string(".txt");
        // 创建车对象，填充信息
        CarInfo car;
        stringstream ss;
        ss << "cqusn" << setw(11) << setfill('0') << i;
        string car_sn = ss.str();
        string chassis_sn = string("dp") + to_string(10000000 + i);
        addCarParam(car, car_sn, chassis_sn);
        // 创建学生对象，填充信息
        StudentInfo stu;
        string sid = string("2025") + to_string(100 + i);
        string sname = string("Student_") + to_string(i);
        addStudentParam(stu, sid, sname);
        // 小车、学生信息保存到本地
        car.save(fname_car);
        stu.save(fname_stu);
        // 小车、学生信息保存到vector容器
        car_files.push_back(fname_car);
        stu_files.push_back(fname_stu);
        cout << "Saved " << fname_car << "\n";
        cout << "Saved " << fname_stu << "\n";
    }

    cout << "\n== 从文件读取并浏览 (按 n 下一辆, p 上一辆, q 退出) ==\n";
    int idx = 0;
    while (true) {
        cout << "\n--- Car " << (idx+1) << " / " << car_files.size() << " ---\n";
        // 读取本地文件并打印
        printFileContent(car_files[idx]);
        cout << "Assigned student info:\n";
        printFileContent(stu_files[idx]);
        // 通过int(idx)进行边界检查
        cout << "按键 (n/p/q): ";
        int c = _getch();
        if (c == 'n' || c == 'N') {
            if (idx + 1 < (int)car_files.size()) ++idx; else cout << "已经是最后一辆，无法向后。\n";
        } else if (c == 'p' || c == 'P') {
            if (idx - 1 >= 0) --idx; else cout << "已经是第一辆，无法向前。\n";
        } else if (c == 'q' || c == 'Q') {
            break;
        }
    }

    cout << "退出。\n";
    return 0;
}