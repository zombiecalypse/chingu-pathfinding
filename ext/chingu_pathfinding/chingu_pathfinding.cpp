#include <ruby.h>
#include <math.h>
#include <vector>
#include <unordered_map>
#include <queue>
#include <unordered_set>
#include <iostream>
#include <algorithm>
#include <functional>
#include <stdexcept>
#include <sstream>
#include <limits>

static void Init_pathfinding();

extern "C" {
  void Init_chingu_pathfinding() {
    Init_pathfinding();
  }
}

namespace std {
  template<typename a, typename b>
  struct hash< std::pair<a, b> > {
  private:
     const hash<a> ah;
     const hash<b> bh;
  public:
     hash() : ah(), bh() {}
     size_t operator()(const std::pair<a, b> &p) const {
        return ah(p.first) ^ bh(p.second) << 5 ^ bh(p.second);
     }
  };
}

inline int dir(const int a, const int b) {
  if (a == b) return 0;
  if (a < b)  return 1;
  return -1;
}

struct point_t {
  int x, y;
  bool valid;
  point_t(int xx, int yy) {
    x = xx;
    y = yy;
    valid = true;
  }
  point_t() : x(0), y(0), valid(false) { }
  point_t(const point_t& p) : x(p.x), y(p.y), valid(p.valid) { }
};
typedef std::pair<int, int> location_t;

float dist(const location_t& a, const location_t& b) {
  float dx = a.first-b.first;
  float dy = a.second-b.second;
  return std::sqrt(dx*dx + dy*dy);
}

class path_t {
  std::unordered_map< location_t, point_t >* map_p;
  std::unordered_multimap< location_t, location_t >* connectivity_p;
  float cost_paid;

  public:
  std::vector<location_t> path;
  path_t() : cost_paid(0), map_p(NULL), connectivity_p(NULL) { }
  path_t(const path_t& p) : map_p(p.map_p), connectivity_p(p.connectivity_p), path(p.path), cost_paid(p.cost_paid) { }
  path_t(std::unordered_map< location_t, point_t >* mm,
      std::unordered_multimap< location_t, location_t >* cc) 
    : map_p(mm), connectivity_p(cc), cost_paid(0), path() { }
  float get_cost_paid() const { return cost_paid; }
  void add_loc(const location_t& l, const location_t& goal) {
    if (!path.empty()) {
      location_t last = *path.crbegin();
      cost_paid += dist(last, l);
    }
    path.push_back(l);
  }
  float dist_to(const location_t& goal) const {
    return dist(*path.crbegin(), goal);
  }

  location_t end() const {
    if (path.empty()) throw std::runtime_error("no elements in path");
    return *path.crbegin();
  }
};

class by_estimated_cost {
  public:
    location_t goal;
    by_estimated_cost() : goal(0,0) { }
    by_estimated_cost(const by_estimated_cost& p) : goal(p.goal) { }
    by_estimated_cost(location_t& goal) : goal(goal) { }
    bool operator() (const path_t& l, const path_t& r) const {
      return l.get_cost_paid() + l.dist_to(goal) > r.get_cost_paid() + l.dist_to(goal);
    }
};


class pathfinder_t {
  std::unordered_map< location_t, point_t > map; // Spartial hashing
  std::unordered_multimap< location_t, location_t > connectivity; // sparse connectivity
  int height, width, block_size;
  ID blocked_method;
  VALUE self;

  bool is_blocked(const int x, const int y) {
    VALUE blocked = rb_funcall(self, blocked_method, 2, INT2NUM(x), INT2NUM(y));
    return blocked != Qfalse && blocked != Qnil;
  }

  bool line_blocked(const int x1, const int y1, const int x2, const int y2) {
    if (block_size == 1) return false;
    int dx = dir(x1, x2);
    int dy = dir(y1, y2);
    int runner_x = x1;
    int runner_y = y1;
    while (runner_x * dx <= x2 * dx &&
        runner_y * dy <= y2 * dy) {
      if (is_blocked(runner_x, runner_y)) {
        return true;
      }
      runner_x += dx;
      runner_y += dy;
    }
    return false;
  }

  typedef std::priority_queue<path_t, std::vector<path_t>, by_estimated_cost> queue_t;
  path_t find_path_internal(queue_t& queue, location_t& goal) {
    std::unordered_set<location_t> visited;
    while (!queue.empty()) {
      path_t current = queue.top();
      const location_t& end = current.end();
      queue.pop();
      if (visited.find(end) != visited.end()) continue;
      visited.insert(end);
      if (dist(end, goal) < block_size) {
        if (end != goal) {
          current.add_loc(goal, goal);
        }
        return current;
      }
      auto range = connectivity.equal_range(end);
      for (auto c = range.first; c != range.second; c++) {
        auto to = c->second;
        if (visited.find(to) != visited.end()){
          continue;
        }
        path_t new_path(current);
        new_path.add_loc(to, goal);
        queue.push(new_path);
      }
    }
    return path_t();
  }

  public:
  path_t find_path(int x1, int y1, int x2, int y2) {
    location_t start(x1,y1);
    location_t goal(x2,y2);
    by_estimated_cost estimator(goal);
    queue_t queue(estimator);
    path_t init_path(&map, &connectivity);
    int next_node_x = (x1 / block_size)*block_size + block_size/2;
    int next_node_y = (y1 / block_size)*block_size + block_size/2;
    init_path.add_loc(start, goal);
    location_t start_block(next_node_x, next_node_y);
    if (start != start_block) {
      init_path.add_loc(start_block, goal);
    }
    queue.push(init_path);
    return find_path_internal(queue, goal);
  }

  path_t find_path_update(int x1, int y1, int x2, int y2, std::vector<location_t> current) {
    location_t start(x1,y1);
    location_t goal(x2,y2);
    path_t init_path;
    by_estimated_cost estimator(goal);
    queue_t queue(estimator);
    init_path.add_loc(start, goal);
    // Cut out the part of the path already visited
    bool drop = true;
    float last_dist = std::numeric_limits<float>::max();
    for (location_t l : current) {
      float d = dist(l, start);
      if (d > last_dist) {
        drop = false;
      }
      if (!drop && start != l) {
        init_path.add_loc(l, goal);
        queue.push(init_path);
      }
      last_dist = d;
    }
    if (queue.empty()) {
      queue.push(init_path);
    }
    // Add every prefix of current to initial guess and converge to new location.
    return find_path_internal(queue, goal);
  }

  void Init(VALUE self, int width, int height, int block_size) {
    this->width = width;
    this->height = height;
    this->block_size = block_size;
    this->self = self;
    this->map.reserve(width/block_size * height/block_size);
    this->connectivity.reserve(width/block_size * height/block_size*4);
    blocked_method = rb_intern("blocked?");
    // Build internal map:
    for (int x = block_size/2; x < width; x += block_size) {
      for (int y = block_size/2; y < height; y += block_size) {
        //   add point if it is not blocked
        if (is_blocked(x, y)) continue;
        auto cur_loc = location_t(x,y);
        map[cur_loc] = point_t(x,y);
        //   check if point is connected to neighbors
        //   ^
        //   | o  .  .
        //   |  \
        //   | o--*  .
        //  y|  / |
        //   | o  o  .
        //   +---------->
        //       x
        // This means that every possible connection is stored as the map is
        // built.
        auto left = location_t(x-block_size, y);
        if (map[left].valid && !line_blocked(x, y, x-block_size, y)) {
          connectivity.emplace(cur_loc, left);
          connectivity.emplace(left, cur_loc);
        }
        auto down = location_t(x, y-block_size);
        if (map[down].valid && !line_blocked(x, y, x, y-block_size)) {
          connectivity.emplace(cur_loc, down);
          connectivity.emplace(down, cur_loc);
        }
        auto left_down = location_t(x-block_size, y-block_size);
        if (map[left_down].valid && !line_blocked(x, y, x-block_size, y-block_size)) {
          connectivity.emplace(cur_loc, left_down);
          connectivity.emplace(left_down, cur_loc);
        }
        auto left_up = location_t(x-block_size, y+block_size);
        if (map[left_up].valid && !line_blocked(x, y, x-block_size, y+block_size)) {
          connectivity.emplace(cur_loc, left_up);
          connectivity.emplace(left_up, cur_loc);
        }
      }
    }
  } // END Init

  std::string print_map() {
    // DEBUGGING

    auto mapp = std::unordered_map<location_t, char>();
    for (int x = 0; x < width; x++) {
      for (int y = 0; y < height; y++) {
        mapp[location_t(x, y)] = ' ';
      }
    }
    for (int x = block_size/2; x < width; x += block_size) {
      for (int y = block_size/2; y < height; y += block_size) {
        if (!map[location_t(x, y)].valid) {
          mapp[location_t(x, y)] = '~';
          continue;
        }
        location_t cur_loc(x,y);
        mapp[cur_loc] = 'o';
        auto c = connectivity.equal_range(cur_loc);
        for (int i = -1; i <= 1; i++) {
          for (int j = -1; j <= 1; j++) {
            std::pair<const location_t, location_t> item(cur_loc, location_t(x+ i * block_size, y + j*block_size));
            if (std::find(c.first, c.second, item) != c.second) {
              mapp[location_t(x+i, y+j)] = '.';
            }
          }
        }
      }
    }
    std::stringstream out;
    out << "map:";
    for (int y = 0; y < height; y++) {
      out << std::endl;
      for (int x = 0; x < width; x++) {
        out << mapp[location_t(x, y)];
      }
    }
    out << std::endl;
    return out.str();
  } // END print_map
};

static void free_pathfinder(pathfinder_t* pf) {
  if (pf) {
    delete pf;
  }
}

static VALUE alloc_pathfinder(VALUE klass) {
  pathfinder_t* pf = new pathfinder_t();
  return Data_Wrap_Struct(klass, 0, free_pathfinder, pf);
}

static VALUE init(VALUE self, VALUE width_a, VALUE height_a, VALUE block_size_a) {
  pathfinder_t* pf;
  Data_Get_Struct(self, pathfinder_t, pf);
  pf->Init(self, NUM2INT(width_a), NUM2INT(height_a), NUM2INT(block_size_a));
  return self;
}

static VALUE find_path(VALUE self, VALUE x1, VALUE y1, VALUE x2, VALUE y2) {
  pathfinder_t* pf;
  Data_Get_Struct(self, pathfinder_t, pf);
  path_t path = pf->find_path(NUM2INT(x1), NUM2INT(y1), NUM2INT(x2), NUM2INT(y2));
  if (path.path.empty()) return Qnil;
  VALUE* path_ruby = new VALUE[path.path.size()];
  int i = 0;
  for (auto loc: path.path) {
    path_ruby[i++] = rb_ary_new3(2, INT2NUM(loc.first), INT2NUM(loc.second));
  }
  return rb_ary_new4(i, path_ruby);
}

static VALUE find_path_update(VALUE self, VALUE x1, VALUE y1, VALUE x2, VALUE y2, VALUE prev) {
  pathfinder_t* pf;
  Data_Get_Struct(self, pathfinder_t, pf);
  unsigned int len = RARRAY_LEN(prev);
  std::vector<location_t> p;
  for (int i = 0; i < len; i++) {
    VALUE pt = rb_ary_shift(prev);
    int x = NUM2INT(rb_ary_shift(pt));
    int y = NUM2INT(rb_ary_shift(pt));
    p.push_back(location_t(x, y));
  }
  path_t path = pf->find_path_update(NUM2INT(x1), NUM2INT(y1), NUM2INT(x2), NUM2INT(y2), p);
  if (path.path.empty()) return Qnil;
  VALUE* path_ruby = new VALUE[path.path.size()];
  int i = 0;
  for (auto loc: path.path) {
    path_ruby[i++] = rb_ary_new3(2, INT2NUM(loc.first), INT2NUM(loc.second));
  }
  return rb_ary_new4(i, path_ruby);
}

static VALUE print_map(VALUE self) {
  pathfinder_t* pf;
  Data_Get_Struct(self, pathfinder_t, pf);
  std::string s = pf->print_map();
  return rb_str_new(s.c_str(), s.length());
}

typedef VALUE (*rb_method)(...);

static void Init_pathfinding() {
  VALUE klass;
  klass = rb_const_get(rb_cObject, rb_intern("Pathfinding"));
  rb_define_alloc_func(klass, alloc_pathfinder);
  rb_define_method(klass, "initialize", (rb_method) init, 3);
  rb_define_method(klass, "to_s", (rb_method) print_map, 0);
  rb_define_private_method(klass, "_find_path", (rb_method) find_path, 4);
  rb_define_private_method(klass, "_find_path_update", (rb_method) find_path_update, 5);
}
