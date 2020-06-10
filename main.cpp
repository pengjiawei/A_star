#include <algorithm>
#include <iostream>
#include <set>
#include <tuple>
#include <vector>
#include "AStar.hpp"

using namespace std::placeholders;
struct Point2D {
  Point2D() {}
  Point2D(int x, int y) : x(x), y(y) {}
  bool operator==(const Point2D& coordinates_) {
    return (x == coordinates_.x && y == coordinates_.y);
  }

  int x = 0;
  int y = 0;
};

Point2D operator+(const Point2D& left_, const Point2D& right_) {
  return {left_.x + right_.x, left_.y + right_.y};
}

typedef Point2D Point;
// it will be more rational if using setter and getter
struct Node {
  uint G_, H_;
  Point2D xy_point_;
  Node* parent_;

  Node(Point2D coord_, Node* parent_ = nullptr);
  uint getScore();
};

Node::Node(Point xy_point, Node* parent) {
  parent_ = parent;
  xy_point_ = xy_point;
  G_ = H_ = 0;
}

uint Node::getScore() { return G_ + H_; }

// data structure definition
std::set<Node*> open_set, closed_set;
std::vector<Point> direction = {{0, 1},   {1, 0}, {0, -1}, {-1, 0},
                                {-1, -1}, {1, 1}, {-1, 1}, {1, -1}};
int directions_index_range = 8;
int world_width = 10, world_height = 10;
// obs is circle shape,tuple means x,y,radius
std::vector<std::tuple<double, double, double>> obs_list = {
    std::make_tuple(0.2, 0.3, 0.2), std::make_tuple(0.8, 0.2, 0.1),
    std::make_tuple(0.5, 0.6, 0.3)};

bool sanityCheck(Point point) const {
  if ((point.x < 0) || (point.x >= world_width) || (point.y < 0) ||
      (point.y >= world_height)) {
    return false;
  }
  return true;
}

bool collisionCheck(Point point) {
  // point in world
  if (sanityCheck(point)) {
    // TODO(pengjiawei) collision check
    return true;
  }
  return false;
}

Node* findNodeOnList(std::set<Node*>& nodes, Point point) {
  for (auto node : nodes) {
    if (node->xy_point_ == point) {
      return node;
    }
  }
  return nullptr;
}

Point Heuristic::getDelta(Point source, Point target) {
  return {abs(source.x - target.x), abs(source.y - target.y)};
}

uint Heuristic::manhattan(Point source, Point target) {
  auto delta = std::move(getDelta(source, target));
  return static_cast<uint>(10 * (delta.x + delta.y));
}

uint Heuristic::euclidean(Point source, Point target) {
  auto delta = std::move(getDelta(source, target));
  return static_cast<uint>(10 * sqrt(pow(delta.x, 2) + pow(delta.y, 2)));
}

uint Heuristic::octagonal(Point source, Point target) {
  auto delta = std::move(getDelta(source, target));
  return 10 * (delta.x + delta.y) + (-6) * std::min(delta.x, delta.y);
}
int heuristic(Point source, Point target) {}

void clearNodes(std::set<Node*>& nodes_) {
  for (auto it = nodes_.begin(); it != nodes_.end();) {
    delete *it;
    it = nodes_.erase(it);
  }
}

bool findPath(Point source, Point target, std::vector<Point>* result) {
  sanityCheck(source);
  sanityCheck(target);

  Node* current = nullptr;
  open_set.insert(new Node(source));

  while (!open_set.empty()) {
    current = *open_set.begin();
    for (auto node : open_set) {
      if (node->getScore() <= current->getScore()) {
        current = node;
      }
    }

    if (current->xy_point_ == target) {
      break;
    }

    closed_set.insert(current);
    open_set.erase(std::find(open_set.begin(), open_set.end(), current));

    for (uint i = 0; i < directions_index_range; ++i) {
      Point new_point(current->xy_point_ + direction[i]);
      if (collisionCheck(new_point) || findNodeOnList(closed_set, new_point)) {
        continue;
      }
      // means 1 or sqrt(2)
      uint state_transition_cost = current->G_ + ((i < 4) ? 10 : 14);

      Node* successor = findNodeOnList(open_set, new_point);
      if (successor == nullptr) {
        successor = new Node(new_point, current);
        successor->G_ = state_transition_cost;
        successor->H_ = heuristic(successor->xy_point_, target);
        open_set.insert(successor);
      } else if (state_transition_cost < successor->G) {
        successor->parent_ = current;
        successor->G_ = state_transition_cost;
      }
    }
  }

  while (current != nullptr) {
    result->emplace_back(current->xy_point_);
    current = current->parent_;
  }

  clearNodes(open_set);
  clearNodes(closed_set);
  return true;
}

int main() {
  std::cout << "Hello, World!" << std::endl;
  return 0;
}