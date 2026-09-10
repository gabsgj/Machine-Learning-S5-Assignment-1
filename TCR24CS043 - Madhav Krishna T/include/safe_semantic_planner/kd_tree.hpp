#pragma once

#include <vector>
#include <memory>
#include <limits>
#include <cmath>
#include <algorithm>
#include <string>

namespace safe_semantic_planner {

struct KdPoint {
    std::string id;
    std::vector<double> coords;

    KdPoint() = default;
    KdPoint(std::string id_, std::vector<double> coords_)
        : id(std::move(id_)), coords(std::move(coords_)) {}
};

struct KdNode {
    KdPoint point;
    int axis = 0;
    std::unique_ptr<KdNode> left;
    std::unique_ptr<KdNode> right;

    explicit KdNode(KdPoint p, int ax = 0)
        : point(std::move(p)), axis(ax), left(nullptr), right(nullptr) {}
};

class KdTree {
public:
    KdTree() = default;
    explicit KdTree(const std::vector<KdPoint>& points);

    void build(const std::vector<KdPoint>& points);
    void clear();
    bool empty() const { return root_ == nullptr; }
    size_t size() const { return size_; }

    // Nearest neighbor search: returns min Euclidean distance to any point in the tree
    double nearestDistance(const std::vector<double>& target, std::string* nearestId = nullptr) const;

    // Range search: returns all point IDs within Euclidean radius r of target
    std::vector<std::string> radiusSearch(const std::vector<double>& target, double radius) const;

    // Memory accounting in bytes
    size_t getAnalyticalMemoryBytes() const;

private:
    std::unique_ptr<KdNode> buildRecursive(std::vector<KdPoint>& points, int depth, size_t start, size_t end);
    void nearestRecursive(const KdNode* node, const std::vector<double>& target, double& bestDistSq, std::string& bestId) const;
    void radiusRecursive(const KdNode* node, const std::vector<double>& target, double radiusSq, std::vector<std::string>& results) const;

    static double distanceSq(const std::vector<double>& a, const std::vector<double>& b);

    std::unique_ptr<KdNode> root_;
    size_t size_ = 0;
    size_t dimensions_ = 2;
};

} // namespace safe_semantic_planner
