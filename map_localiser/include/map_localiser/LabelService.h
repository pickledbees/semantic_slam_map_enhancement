//
// Created by han on 16/2/21.
//

#ifndef SRC_LABELSERVICE_H
#define SRC_LABELSERVICE_H

#include <cstdint>
#include <string>
#include <unordered_map>
#include <unordered_set>

class LabelService {
public:
    static int RGBToHash(uint8_t r, uint8_t g, uint8_t b);

    static bool isLandmark(int hash);

    static std::string hashToLabel(int hash);

private:
    static const std::unordered_map<int, std::string> map_;
    static const std::unordered_set<int> trivial_;
};


#endif //SRC_LABELSERVICE_H
