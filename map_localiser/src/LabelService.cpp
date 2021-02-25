//
// Created by han on 16/2/21.
//

#include <map_localiser/LabelService.h>


int LabelService::RGBToHash(uint8_t r, uint8_t g, uint8_t b) {
    return r << 16 | g << 8 | b;
}

bool LabelService::isLandmark(int hash) {
    return trivial_.find(hash) != trivial_.end();
}

std::string LabelService::hashToLabel(int hash) {
    auto pair = map_.find(hash);
    return pair != map_.end() ? pair->second : "unknown";
}

const std::unordered_map<int, std::string> LabelService::map_ = {
        {0,        "background"},
        {64,       "nightstand"},
        {128,      "bed"},
        {192,      "bathtub"},
        {16384,    "curtain"},
        {16512,    "floormat"},
        {32768,    "floor"},
        {32832,    "sink"},
        {32896,    "sofa"},
        {49152,    "pillow"},
        {49280,    "ceiling"},
        {4194304,  "door"},
        {4194432,  "counter"},
        {4210688,  "fridge"},
        {4210816,  "shower_curtain"},
        {4227072,  "bookshelf"},
        {4227200,  "desk"},
        {4243456,  "paper"},
        {4243584,  "whiteboard"},
        {8388608,  "wall"},
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
        {12582912, "window"},
        {12583040, "blinds"},
        {12599296, "tv"},
        {12599424, "box"},
        {12615680, "picture"},
        {12615808, "shelves"},
        {12632064, "towel"},
        {12632192, "person"},
        {16777215, "air"}
};

const std::unordered_set<int> LabelService::trivial_ = {
        0,      //background
        32768,  //floor
        49280,  //ceiling
        8388608, //wall
        12582912,  //window
        12632192, //person
        16777215    //air
};