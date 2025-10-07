#ifndef __PARSEKEYVALUE_H__
#define __PARSEKEYVALUE_H__
#include <string>
#include <map>

std::map<std::string, std::string> parseKeyValueFile(const std::string& filePath);

#endif // __PARSEKEYVALUE_H__