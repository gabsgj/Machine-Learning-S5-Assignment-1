#include "safe_semantic_planner/kd_tree.hpp"

namespace safe_semantic_planner {

double KdTree::distanceSq(const std::vector<double>& a, const std::vector<double>& b) {
    size_t dim = std::min(a.size(), b.size());
    double sum = 0.0;
    for (size_t i = 0; i < dim; ++i) {
        double diff = a[i] - b[i];
        sum += diff * diff;
    }
    return sum;
}

KdTree::KdTree(const std::vector<KdPoint>& points) {
    build(points);
}

void KdTree::clear() {
    root_.reset();
    size_ = 0;
}

void KdTree::build(const std::vector<KdPoint>& points) {
    clear();
    if (points.empty()) return;

    dimensions_ = points[0].coords.size();
    if (dimensions_ == 0) dimensions_ = 2;

    std::vector<KdPoint> pts = points;
    size_ = pts.size();
    root_ = buildRecursive(pts, 0, 0, pts.size());
}

std::unique_ptr<KdNode> KdTree::buildRecursive(std::vector<KdPoint>& points, int depth, size_t start, size_t end) {
    if (start >= end) return nullptr;

    int axis = depth % dimensions_;
    size_t mid = start + (end - start) / 2;

    auto comparator = [axis](const KdPoint& a, const KdPoint& b) {
        double valA = (axis < static_cast<int>(a.coords.size())) ? a.coords[axis] : 0.0;
        double valB = (axis < static_cast<int>(b.coords.size())) ? b.coords[axis] : 0.0;
        return valA < valB;
    };

    std::nth_element(points.begin() + start, points.begin() + mid, points.begin() + end, comparator);

    auto node = std::make_unique<KdNode>(points[mid], axis);
    node->left = buildRecursive(points, depth + 1, start, mid);
    node->right = buildRecursive(points, depth + 1, mid + 1, end);
    return node;
}

double KdTree::nearestDistance(const std::vector<double>& target, std::string* nearestId) const {
    if (!root_) {
        return std::numeric_limits<double>::infinity();
    }
    double bestDistSq = std::numeric_limits<double>::infinity();
    std::string bestId = "";
    nearestRecursive(root_.get(), target, bestDistSq, bestId);
    if (nearestId) {
        *nearestId = bestId;
    }
    return std::sqrt(bestDistSq);
}

void KdTree::nearestRecursive(const KdNode* node, const std::vector<double>& target, double& bestDistSq, std::string& bestId) const {
    if (!node) return;

    double dSq = distanceSq(node->point.coords, target);
    if (dSq < bestDistSq) {
        bestDistSq = dSq;
        bestId = node->point.id;
    }

    int axis = node->axis;
    double nodeVal = (axis < static_cast<int>(node->point.coords.size())) ? node->point.coords[axis] : 0.0;
    double targetVal = (axis < static_cast<int>(target.size())) ? target[axis] : 0.0;
    double planeDiff = targetVal - nodeVal;

    const KdNode* first = (planeDiff < 0.0) ? node->left.get() : node->right.get();
    const KdNode* second = (planeDiff < 0.0) ? node->right.get() : node->left.get();

    nearestRecursive(first, target, bestDistSq, bestId);

    if ((planeDiff * planeDiff) < bestDistSq) {
        nearestRecursive(second, target, bestDistSq, bestId);
    }
}

std::vector<std::string> KdTree::radiusSearch(const std::vector<double>& target, double radius) const {
    std::vector<std::string> results;
    if (!root_ || radius < 0.0) return results;

    double radiusSq = radius * radius;
    radiusRecursive(root_.get(), target, radiusSq, results);
    return results;
}

void KdTree::radiusRecursive(const KdNode* node, const std::vector<double>& target, double radiusSq, std::vector<std::string>& results) const {
    if (!node) return;

    double dSq = distanceSq(node->point.coords, target);
    if (dSq <= radiusSq) {
        results.push_back(node->point.id);
    }

    int axis = node->axis;
    double nodeVal = (axis < static_cast<int>(node->point.coords.size())) ? node->point.coords[axis] : 0.0;
    double targetVal = (axis < static_cast<int>(target.size())) ? target[axis] : 0.0;
    double planeDiff = targetVal - nodeVal;

    if (planeDiff <= 0.0) {
        radiusRecursive(node->left.get(), target, radiusSq, results);
        if ((planeDiff * planeDiff) <= radiusSq) {
            radiusRecursive(node->right.get(), target, radiusSq, results);
        }
    } else {
        radiusRecursive(node->right.get(), target, radiusSq, results);
        if ((planeDiff * planeDiff) <= radiusSq) {
            radiusRecursive(node->left.get(), target, radiusSq, results);
        }
    }
}

size_t KdTree::getAnalyticalMemoryBytes() const {
    // Each node holds KdPoint (id string + vector coords), int axis, and 2 unique_ptr
    size_t nodeBytes = sizeof(KdNode) + sizeof(std::string) + dimensions_ * sizeof(double);
    return sizeof(KdTree) + size_ * nodeBytes;
}

} // namespace safe_semantic_planner
