#ifndef TOPOLOGICAL_MAP_HPP
#define TOPOLOGICAL_MAP_HPP

#include <vector>
#include <algorithm>
#include <queue>
#include <unordered_map>

#include <boost/shared_ptr.hpp>
#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/xml_parser.hpp>

#include "vtr/node.hpp"
#include "vtr/congfig.h"

class TopologicalMap
{
public:
  typedef boost::shared_ptr<TopologicalMap> Ptr;

private:
  std::vector<Node::Ptr> nodes_;

public:
  TopologicalMap() {}

  Node::Ptr createNode()
  {
    Node::Ptr n(new Node);
    n->id = nodes_.size();
    nodes_.push_back(n);
    return n;
  }

  const std::vector<Node::Ptr>& getNodeList() const { return nodes_; }

  Node::Ptr getNode(int id)
  {
    if (id < 0 || id >= nodes_.size())
    {
      std::cerr << "Id execceded:" << id << std::endl;
      return Node::Ptr();
    }
    return nodes_[id];
  }

  int size() const { return nodes_.size(); }

  NeighbourList neighbourSearch(Node::Ptr center, int max_depth, int max = 0) const
  {
    NeighbourList nei;
    NeighbourList ans;

    if (!center)
      return nei;

    std::vector<bool> visited(nodes_.size(), false);
    visited[center->id] = true;
    Neighbour n(center, Transform::Identity(), 0);
    nei.push_back(n);
    neighbourSearchDFS(n, max_depth, visited, nei);

    if (max > 0)
    {
      std::vector<int> perm =randperm(nei.size());
      std::stringstream ss_perm;
      for (int i = 0; i < perm.size(); ++i)
        ss_perm << perm[i] << " ";
      for (int i = 0; i < std::min<int>(max, nei.size()); ++i)
        ans.push_back(nei[perm[i]]);
    }
    else
      ans = nei;

    return ans;
  }

  void addEdge(Node::Ptr& n1, Node::Ptr& n2, const Transform& transform)
  {
    n1->neighbours.push_back(Neighbour(n2, transform, 1, true));
    n2->neighbours.push_back(Neighbour(n1, transform.inverse(), 1, false));
  }

  bool read(const std::string& filename)
  {
    nodes_.clear();

    try {
      boost::property_tree::ptree p_map;
      boost::property_tree::read_xml(filename, p_map);

      int node_count = p_map.get<int>("map.property.node_count");
      nodes_.resize(node_count);
      for (auto node_it = nodes_.begin(); node_it != nodes_.end(); ++node_it)
        node_it->reset(new Node);

      boost::property_tree::ptree p_node_list = p_map.get_child("map.node_list");
      for (auto p_node_it = p_node_list.begin(); p_node_it != p_node_list.end(); ++p_node_it)
      {
        int node_id = p_node_it->second.get<int>("id");
        assert(node_id >= 0 && node_id < nodes_.size());
        Node::Ptr node = nodes_[node_id];
        node->id = node_id;
        boost::property_tree::ptree p_neighbour_list = p_node_it->second.get_child("neighbour_list");
        for (auto p_neighbour_it = p_neighbour_list.begin(); p_neighbour_it != p_neighbour_list.end(); ++p_neighbour_it)
        {
          int neighbour_id = p_neighbour_it->second.get<int>("id");
          assert(neighbour_id >= 0 && neighbour_id < nodes_.size());
          Node::Ptr neighbour = nodes_[neighbour_id];
          std::string transform_str = p_neighbour_it->second.get<std::string>("transform");
          std::stringstream ss(transform_str);
          double x, y, z, qw, qx, qy, qz;
          ss >> x >> y >> z >> qw >> qx >> qy >> qz;
          Vector t(x, y, z);
          Quaternion q(qw, qx, qy, qz);
          Transform transform;
          transform.setIdentity();
          transform.translate(t);
          transform.rotate(q);
          bool reachable = p_neighbour_it->second.get<bool>("reachable");
          node->neighbours.push_back(Neighbour(neighbour, transform, 1, reachable));
        }
      }
    } catch (boost::property_tree::ptree_error&) {
      return false;
    }

    return true;
  }

  std::vector<Neighbour> search(const Node::Ptr& source, const Node::Ptr& target/*, transform& transform, int& hop*/)
  {
    std::vector<Neighbour> ans;

    std::unordered_map<Node::Ptr, Neighbour> parent;
    parent[source] = Neighbour(Node::Ptr(), Transform::Identity());

    std::queue<Neighbour> queue;
    queue.push(Neighbour(source, Transform::Identity()/*, 0*/));

    std::vector<bool> visited(nodes_.size(), false);

    while (!queue.empty() && !visited[target->id])
    {
      Neighbour cur = queue.front();
      queue.pop();
      visited[cur.first->id] = true;

      if (*target == *cur.first)
      {
        Node::Ptr n = target;
        Transform trans = Transform::Identity();
        while (!!n)
        {
          ans.push_back(Neighbour(n, trans));
          Neighbour nei = parent[n];
          n = nei.first;
          trans = trans * nei.second;
        }

        trans = ans.back().second.inverse();
        for (int i = 0; i < ans.size(); ++i)
          ans[i].second = trans * ans[i].second;
        std::reverse(ans.begin(), ans.end());
        break;
      }

      for (auto it = cur.first->neighbours.begin(); it != cur.first->neighbours.end(); ++it)
      {
        if (!visited[it->first->id])
        {
          queue.push(Neighbour(it->first, cur.second * it->second/*, cur.getHop() + 1)*/));
          parent[it->first] = Neighbour(cur.first, it->second.inverse());
        }
      }
    }

    return ans;
  }

  void save(const std::string& filename)
  {
    boost::property_tree::ptree p_map;
    p_map.put("map.property.node_count", nodes_.size());

    boost::property_tree::ptree p_node_list;
    for (auto node_it = nodes_.begin(); node_it != nodes_.end(); ++node_it)
    {
      boost::property_tree::ptree p_node;
      p_node.put("id", (*node_it)->id);
      boost::property_tree::ptree p_neighbour_list;
      for (auto neighbour_it = (*node_it)->neighbours.begin(); neighbour_it != (*node_it)->neighbours.end(); ++neighbour_it)
      {
        boost::property_tree::ptree p_neighbour;
        p_neighbour.put("id", neighbour_it->first->id);
        Transform T = neighbour_it->second;
        Quaternion q(T.rotation());
        Vector t(T.translation());
        std::stringstream ss;
        ss << t[0] << " " << t[1] << " " << t[2] << " " << q.w() << " " << q.x() << " " << q.y() << " " << q.z();
        p_neighbour.put("transform", ss.str());
        p_neighbour.put("reachable", neighbour_it->isReachable());
        p_neighbour_list.add_child("neighbour", p_neighbour);
      }
      p_node.add_child("neighbour_list", p_neighbour_list);
      p_node_list.add_child("node", p_node);
    }

    p_map.add_child("map.node_list", p_node_list);

    boost::property_tree::xml_writer_settings<char> setting(' ', 2);
    boost::property_tree::write_xml(filename, p_map, std::locale(), setting);
  }

  friend inline std::ostream& operator << (std::ostream& os, TopologicalMap& map)
  {
    os << "Map info: {" <<
          "node_count=" << map.size() <<
          "}";
    return os;
  }

private:
  void neighbourSearchDFS(const Neighbour& center, int max_depth, std::vector<bool>& visited, NeighbourList& neighbours) const
  {
    if (max_depth == 0)
      return;

    for (auto it = center.first->neighbours.begin(); it!= center.first->neighbours.end(); ++it)
    {
      if (!visited[it->first->id])
      {
        visited[it->first->id] = true;
        Transform npose = center.second * it->second;
        Neighbour n(it->first, npose, center.getHop() + 1);
        neighbours.push_back(n);
        neighbourSearchDFS(/*it->first, npose*/n, max_depth-1, visited, neighbours);
      }
    }
  }

  std::vector<int> randperm(int n) const
  {
    std::vector<int> a(n);
    for (int i = 0; i < n; ++i)
      a[i] = i;
    std::random_shuffle(a.begin(), a.end());
    return a;
  }
};

#endif // TOPOLOGICAL_MAP_HPP
