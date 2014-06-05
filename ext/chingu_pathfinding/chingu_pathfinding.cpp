#include <ruby.h>
#include <math.h>
#include <vector>
#include <map>
#include <queue>
#include <set>
#include <iostream>
#include <algorithm>
#include <functional>
#include <stdexcept>
#include <sstream>

static void Init_pathfinding();

extern "C" {
  void Init_chingu_pathfinding() {
    Init_pathfinding();
  }
}

int dir(const long int a, const long int b) {
  if (a == b) return 0;
  if (a < b)  return 1;
  return -1;
}

struct point_t {
  long int x, y;
  bool valid;
  point_t(long int xx, long int yy) {
    x = xx;
    y = yy;
    valid = true;
  }
  point_t() : x(0), y(0), valid(false) { }
  point_t(const point_t& p) : x(p.x), y(p.y), valid(p.valid) { }
};
typedef std::pair<long int, long int> location_t;

long int dist(const location_t& a, const location_t& b) {
  long int dx = a.first-b.first;
  long int dy = a.second-b.second;
  return std::abs(dx) + std::abs(dy);
}

class path_t {
  std::map< location_t, point_t >* map_p;
  std::multimap< location_t, location_t >* connectivity_p;
  long int cost_paid;

  public:
  std::vector<location_t> path;
  path_t() : cost_paid(0) { }
  path_t(const path_t& p) : map_p(p.map_p), connectivity_p(p.connectivity_p), path(p.path), cost_paid(p.cost_paid) { }
  path_t(std::map< location_t, point_t >* mm,
      std::multimap< location_t, location_t >* cc) 
    : map_p(mm), connectivity_p(cc), cost_paid(0), path() { }
  long int get_cost_paid() const { return cost_paid; }
  void add_loc(location_t l, const location_t& goal) {
    if (!path.empty()) {
      location_t last = *path.crbegin();
      cost_paid += dist(last, l);
    }
    path.push_back(l);
  }
  long int dist_to(const location_t& goal) const {
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
  std::map< location_t, point_t > map; // Spartial hashing
  std::multimap< location_t, location_t > connectivity; // sparse connectivity
  long int height, width, block_size;
  ID blocked_method;
  VALUE self;

  bool is_blocked(const long int x, const long int y) {
    VALUE blocked = rb_funcall(self, blocked_method, 2, INT2NUM(x), INT2NUM(y));
    return blocked != Qfalse && blocked != Qnil;
  }

  bool line_blocked(const long int x1, const long int y1, const long int x2, const long int y2) {
    long int dx = dir(x1, x2);
    long int dy = dir(y1, y2);
    long int runner_x = x1;
    long int runner_y = y1;
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

  public:
  path_t find_path(long int x1, long int y1, long int x2, long int y2) {
    typedef std::priority_queue<path_t, std::vector<path_t>, by_estimated_cost> queue_t;
    location_t start(x1,y1);
    location_t goal(x2,y2);
    by_estimated_cost estimator(goal);
    queue_t queue(estimator);
    std::set<location_t> visited;
    path_t init_path(&map, &connectivity);
    long int next_node_x = (x1 / block_size)*block_size + block_size/2;
    long int next_node_y = (y1 / block_size)*block_size + block_size/2;
    init_path.add_loc(start, goal);
    location_t start_block(next_node_x, next_node_y);
    if (start != start_block) {
      init_path.add_loc(start_block, goal);
    }
    queue.push(init_path);
    while (!queue.empty()) {
      path_t current = queue.top();
      location_t end = current.end();
      queue.pop();
      if (visited.find(end) != visited.end()) continue;
      visited.insert(end);
      if (dist(end, goal) < block_size/2) {
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
        path_t new_path = current;
        new_path.add_loc(to, goal);
        queue.push(new_path);
      }
    }
    return path_t();
  }

  path_t find_path_update(long int x1, long int y1, long int x2, long int y2, std::vector<location_t> current) {
    path_t x;
    // Add every prefix of current to initial guess and converge to new location.
    return x;
  }

  void Init(VALUE self, long int width, long int height, long int block_size) {
    this->width = width;
    this->height = height;
    this->block_size = block_size;
    this->self = self;
    blocked_method = rb_intern("blocked?");
    // Build internal map:
    for (long int x = block_size/2; x < width; x += block_size) {
      for (long int y = block_size/2; y < height; y += block_size) {
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
    // print_map();
  } // END Init

  std::string print_map() {
    // DEBUGGING

    auto mapp = std::map<location_t, char>();
    for (long int x = 0; x < width; x++) {
      for (long int y = 0; y < height; y++) {
        mapp[location_t(x, y)] = ' ';
      }
    }
    for (long int x = block_size/2; x < width; x += block_size) {
      for (long int y = block_size/2; y < height; y += block_size) {
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
    for (long int y = 0; y < height; y++) {
      out << std::endl;
      for (long int x = 0; x < width; x++) {
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
  long int width, height, block_size;
  long int x, y;
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
  rb_define_method(klass, "find_path", (rb_method) find_path, 4);
  rb_define_method(klass, "to_s", (rb_method) print_map, 0);
}
