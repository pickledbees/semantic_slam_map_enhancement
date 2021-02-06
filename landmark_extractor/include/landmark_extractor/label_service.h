//
// Created by han on 30/1/21.
//

#ifndef SRC_LABEL_SERVICE_H
#define SRC_LABEL_SERVICE_H

#include <cstdint>
#include <unordered_map>
#include <unordered_set>


class LabelService {
public:
    static int RGBToIntHash(uint8_t r, uint8_t g, uint8_t b);

    static std::string getLabel(uint8_t r, uint8_t g, uint8_t b);

    static std::string getLabel(int hash);

    static bool isLandmark(uint8_t r, uint8_t g, uint8_t b);

    static bool isLandmark(int hash);

private:
    static const std::unordered_map<int, std::string> map_;
    static const std::unordered_set<int> trivial_;
};

#endif //SRC_LABEL_SERVICE_H
