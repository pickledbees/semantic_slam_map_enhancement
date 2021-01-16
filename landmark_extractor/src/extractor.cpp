//
// Created by han on 4/1/21.
//

#include <landmark_extractor/extractor.h>
#include <octomap/ColorOcTree.h>
#include <octomap_msgs/conversions.h>

double inline euclidDistance(double x1, double y1, double z1, double x2, double y2, double z2) {
    return sqrt(pow(x1 - x2, 2) + pow(y1 - y2, 2) + pow(z1 - z2, 2));
}

Extractor::Extractor(int bufferSize) : found_(0), buffer_size_(bufferSize) {
    landmarks_buffer_ = new Landmark[bufferSize];

    insertLandmark(0, 0, 64);// "nightstand"
    insertLandmark(0, 0, 128);// "bed");
    insertLandmark(0, 0, 192);// "bathtub");
    insertLandmark(0, 64, 0);// "curtain");
    insertLandmark(0, 64, 128);// "floormat");
    insertLandmark(0, 128, 64);// "sink");
    insertLandmark(0, 128, 128);// "sofa");
    insertLandmark(0, 192, 0);// "pillow");
    insertLandmark(64, 0, 0);//"door");
    insertLandmark(64, 0, 128);//"counter");
    insertLandmark(64, 64, 0);//"fridge");
    insertLandmark(64, 64, 128);//"shower_curtain");
    insertLandmark(64, 128, 0);//"bookshelf");
    insertLandmark(64, 128, 128);//"desk");
    insertLandmark(64, 192, 0);//"paper");
    insertLandmark(64, 192, 128);//"whiteboard");
    insertLandmark(128, 0, 64);// "toilet");
    insertLandmark(128, 0, 128);//"chair");
    insertLandmark(128, 0, 192);//"bag");
    insertLandmark(128, 64, 0);//"dresser");
    insertLandmark(128, 64, 128);//"clothes");
    insertLandmark(128, 128, 0);//"cabinet");
    insertLandmark(128, 128, 64);//"lamp");
    insertLandmark(128, 128, 128);//"table");
    insertLandmark(128, 192, 0);//"mirror");
    insertLandmark(128, 192, 128);//"books");
    insertLandmark(192, 0, 0);//"window");
    insertLandmark(192, 0, 128);//"blinds");
    insertLandmark(192, 64, 0);//"tv");
    insertLandmark(192, 64, 128);//"box");
    insertLandmark(192, 128, 0);//"picture");
    insertLandmark(192, 128, 128);//"shelves");
    insertLandmark(192, 192, 0);//"towel");
    insertLandmark(192, 192, 128);//"person");
}

void Extractor::extract(float radius, int strategy,
                        octomap_msgs::Octomap::ConstPtr &msg, const geometry_msgs::TransformStamped &tf) {
    switch (strategy) {
        case NEAREST:
            extractNearest(radius, msg, tf);
            break;
        case UNIQUE:
            extractUnique(radius, msg, tf);
            break;
    }
}

Landmark *Extractor::getLandmarks() const {
    return landmarks_buffer_;
}

int Extractor::getFound() const {
    return found_;
}

int Extractor::keyFromRGB(uint8_t r, uint8_t g, uint8_t b) {
    return r << 16 | g << 8 | b;
}

void Extractor::insertLandmark(uint8_t r, uint8_t g, uint8_t b) {
    int key = keyFromRGB(r, g, b);
    landmark_colors_.insert(key);
}

bool Extractor::isLandmark(uint8_t r, uint8_t g, uint8_t b) {
    int key = keyFromRGB(r, g, b);
    return landmark_colors_.find(key) != landmark_colors_.end();
}

void Extractor::extractNearest(float radius, octomap_msgs::Octomap::ConstPtr &msg,
                               const geometry_msgs::TransformStamped &tf) {
    auto *octree = dynamic_cast<octomap::ColorOcTree *>(octomap_msgs::msgToMap(*msg));

    double distance;
    octomap::ColorOcTreeNode::Color color;

    //transform xyz of camera to world == coordinates of camera in world
    float cx = tf.transform.translation.x;
    float cy = tf.transform.translation.y;
    float cz = tf.transform.translation.z;

    for (octomap::ColorOcTree::leaf_iterator it = octree->begin_leafs(), end = octree->end_leafs();
         it != end;
         it++) {

        color = it->getColor();
        if (!isLandmark(color.r, color.g, color.b)) continue;

        distance = euclidDistance(it.getX(), it.getY(), it.getZ(), cx, cy, cz);
        if (distance > radius) continue;    //check if within range

        pq_.push(Landmark(color.r, color.g, color.b, it.getX(), it.getY(), it.getZ(), distance, it.getDepth()));

        while (pq_.size() > buffer_size_) pq_.pop();    //furthest distance item is always removed
    }

    found_ = pq_.size();

    int i = 0;
    while (!pq_.empty()) {
        landmarks_buffer_[i++] = pq_.top();
        pq_.pop();
    }

    delete octree;
}

void Extractor::extractUnique(float radius, octomap_msgs::Octomap::ConstPtr &msg,
                              const geometry_msgs::TransformStamped &tf) {
    auto *octree = dynamic_cast<octomap::ColorOcTree *>(octomap_msgs::msgToMap(*msg));

    double distance;
    octomap::ColorOcTreeNode::Color color;

    //transform xyz of camera to world == coordinates of camera in world
    float cx = tf.transform.translation.x;
    float cy = tf.transform.translation.y;
    float cz = tf.transform.translation.z;

    //capture unique nodes within the radius, choose top nearest
    for (octomap::ColorOcTree::leaf_iterator it = octree->begin_leafs(), end = octree->end_leafs();
         it != end;
         it++) {

        color = it->getColor();
        if (!isLandmark(color.r, color.g, color.b)) continue;

        distance = euclidDistance(it.getX(), it.getY(), it.getZ(), cx, cy, cz);
        if (distance > radius) continue;    //check if within range

        int key = keyFromRGB(color.r, color.g, color.b);

        auto pair = map_.find(key);
        if (pair == map_.end() || (pair->second.depth_ >= it.getDepth() &&
                                   pair->second.distance_ > distance)) {
            map_[key] = Landmark(color.r, color.g, color.b, it.getX(), it.getY(), it.getZ(), distance, it.getDepth());
        }
    }

    //sort
    for (auto &item: map_) {
        pq_.push(item.second);
    }
    map_.clear();

    //trim
    while (pq_.size() > buffer_size_) pq_.pop();

    found_ = pq_.size();
    int i = 0;
    while (!pq_.empty()) {
        landmarks_buffer_[i++] = pq_.top();
        pq_.pop();
    }

    delete octree;
}

Extractor::~Extractor() {
    delete landmarks_buffer_;
}