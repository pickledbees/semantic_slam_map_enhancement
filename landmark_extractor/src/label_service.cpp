//
// Created by han on 30/1/21.
//

#include <landmark_extractor/label_service.h>

inline int LabelService::RGBToIntHash(uint8_t r, uint8_t g, uint8_t b) {
    return r << 16 | g << 8 | b;
}

std::string LabelService::getLabel(uint8_t r, uint8_t g, uint8_t b) {
    auto it = map_.find(RGBToIntHash(r, g, b));
    return it == map_.end() ? "unknown" : it->second;
}

std::string LabelService::getLabel(int hash) {
    auto it = map_.find(hash);
    return it == map_.end() ? "unknown" : it->second;
}

bool LabelService::isLandmark(uint8_t r, uint8_t g, uint8_t b) {
    return map_.find(RGBToIntHash(r, g, b)) != map_.end();
}

bool LabelService::isLandmark(int hash) {
    return map_.find(hash) != map_.end();
}

const std::unordered_map<int, std::string> LabelService::map_ = {
        //{0,        "background"},
        {64,       "nightstand"},
        {128,      "bed"},
        {192,      "bathtub"},
        {16384,    "curtain"},
        {16512,    "floormat"},
        //{32768,    "floor"},
        {32832,    "sink"},
        {32896,    "sofa"},
        {49152,    "pillow"},
        //{49280,    "ceiling"},
        {4194304,  "door"},
        {4194432,  "counter"},
        {4210688,  "fridge"},
        {4210816,  "shower_curtain"},
        {4227072,  "bookshelf"},
        {4227200,  "desk"},
        {4243456,  "paper"},
        {4243584,  "whiteboard"},
        //{8388608,  "wall"},
        {8388672,  "toilet"},
        {8388736,  "chair"},
        {8388800,  "bag"},
        {8404992,  "dresser"},
        {8405120,  "clothes"},
        {8421376,  "cabinet"},
        {8421440,  "lamp"},
        {8421504,  "table"},
        {8437760,  "mirror"},
        {8437888,  "books"},
        //{12582912, "window"},
        {12583040, "blinds"},
        {12599296, "tv"},
        {12599424, "box"},
        {12615680, "picture"},
        {12615808, "shelves"},
        {12632064, "towel"},
        //{12632192, "person"},
        {16777215, "air"}
};