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

// (Saving is done by CarInfo::save)

// helper: read file and print content (simple echo)
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
    cout << "== 智能小车信息录入 (示例化输入) ==\n";
    vector<string> files;
    // create 10 car entries (for demo we'll auto-generate values; user can modify to prompt)
    for (int i = 0; i < 10; ++i) {
        CarInfo car;
        stringstream ss;
        ss << "cqusn" << setw(11) << setfill('0') << i; // creates unique id-like string
        string car_sn = ss.str();
        string chassis_sn = string("dp") + to_string(10000000 + i);

        addCarParam(car, car_sn, chassis_sn);

        // example student assignment (saved separately)
        StudentInfo stu;
        string sid = string("2025") + to_string(100 + i);
        string sname = string("Student_") + to_string(i);
        addStudentParam(stu, sid, sname);

        // save car to file
        string fname = string("../after_class_practice_2/data/car_") + to_string(i) + string(".txt");
        car.save(fname);
        // save student to separate file
        string snamef = string("../after_class_practice_2/data/student_") + to_string(i) + string(".txt");
        {
            ofstream ofs(snamef);
            if (ofs) stu.save(ofs);
            ofs.close();
        }
        files.push_back(fname);
        cout << "Saved " << fname << "\n";
    }

    cout << "\n== 从文件读取并浏览 (按 n 下一辆, p 上一辆, q 退出) ==\n";
    int idx = 0;
    while (true) {
        cout << "\n--- Car " << (idx+1) << " / " << files.size() << " ---\n";
        printFileContent(files[idx]);
        // also print assigned student info if available
        string studentFile = string("../after_class_practice_2/data/student_") + to_string(idx) + string(".txt");
        cout << "\nAssigned student info:\n";
        printFileContent(studentFile);
        cout << "按键 (n/p/q): ";
        int c = _getch();
        if (c == 'n' || c == 'N') {
            if (idx + 1 < (int)files.size()) ++idx; else cout << "已经是最后一辆，无法向后。\n";
        } else if (c == 'p' || c == 'P') {
            if (idx - 1 >= 0) --idx; else cout << "已经是第一辆，无法向前。\n";
        } else if (c == 'q' || c == 'Q') {
            break;
        }
    }

    cout << "退出。\n";
    return 0;
}