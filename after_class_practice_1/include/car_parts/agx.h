// agx.h
#ifndef __AGX_H__
#define __AGX_H__

#include <string>

struct AGX {
    std::string model;           // 型号: AGX Xavier
    double AI;                   // AI算力: 32 TOPS
    double cuda_cores;           // CUDA核心数: 512
    double tensor_cores;         // Tensor核心数: 64
    double memory;               // 内存: 32GB
    double storage;              // 存储: 32GB
};

#endif // __AGX_H__