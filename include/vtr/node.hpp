#ifndef NODE_HPP
#define NODE_HPP

#include <vector>

#include <boost/shared_ptr.hpp>

#include "vtr/congfig.h"


class Neighbour;

typedef std::vector<Neighbour> NeighbourList;

struct Node
{
  typedef boost::shared_ptr<Node> Ptr;

  NeighbourList neighbours;

  int id;

  bool operator == (const Node& node){ return this->id == node.id; }
  bool operator != (const Node& node){ return this->id != node.id; }
};

class Neighbour : public std::pair<Node::Ptr, Transform>
{
public:

  Neighbour(const Node::Ptr& a = Node::Ptr(), const Transform& b = Transform::Identity(), int hop = 1, bool reachable = true) :
    std::pair<Node::Ptr, Transform>(a, b), hop_(hop), reachable_(reachable) {}

  inline int getHop() const { return hop_; }

  inline bool isReachable() { return reachable_; }

  friend inline std::ostream& operator << (std::ostream& os, std::vector<Neighbour>& list)
  {
    os << "[" << list.size() << "] {";
    for (auto it = list.begin(); it != list.end(); ++it)
      os << it->first->id << ((it != (list.end() - 1)) ? "," : "");
    os << "}";
    return os;
  }

private:
  int hop_;

  bool reachable_;
};


namespace std {
  template<>
  struct hash<Node::Ptr> {
    inline size_t operator()(const Node::Ptr& n) const {
      return n->id;
    }
  };
}

#endif // NODE_HPP
