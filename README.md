Chingu Pathfinding
==================

This library is a C++ backed Ruby implementation of the A\* pathfinding
algorithm. All you have to do is to create a subclass of `Pathfinding` 
and implement the `blocked?(x, y)` method. It assumes that the path needs only
to be one pixel thick and that each point is connected in each coordinate
direction and the diagonals (8-connected).

## Initializing the pathfinder
The scenario this library assumes is that there is a rectangular area, where
some pixels are blocked. 

Optionally you can set a `block_size, so that not all pixels need to be checked.
This leads to suboptimal results and blocky moves however, but the better
performance might be worth it.

```ruby
class Map < Pathfinding
  def initialize(width, height)
    super(width, height, 5)
  end

  ...
end
```

initializes the map to have a `block_size` of 5, which means only about half of
the pixels needs to be checked.

```ruby
class Map < Pathfinding
  def initialize(width, height)
    super(width, height, 1)
  end

  ...
end
```

gives a map, that doesn't use bigger blocks at all.

## Implementing the `blocked?(x, y)` method
The method must return false if the pixel (x, y) can be walked over and true
otherwise. So basically if there is a wall, it should return true and if there
is nothing it should return false.

```ruby
class Map < Pathfinding
  def initialize(width, height)
    super(width, height, 1)
  end

  def blocked?(x,y)
    x < 10 or x > 90 or y < 10 or y > 90
  end

  ...
end
```

would be a complete (abeit boring) setup for the pathfinding.

# Using the path finding methods

When in doubt, call `find_path(start_x, start_y, goal_x, goal_y)`, which will
return a list of points (arrays of 2 elements) to pass through with the first
points being `[start_x, start_y]` and the last being `[goal_x, goal_y]`. If you
move linearly between the points, you will not hit any blocked pixels. If you
can't reach the goal for some reason, `nil` is returned.

There is a second path finding method:
`find_path_update(start_x, start_y, goal_x, goal_y, path)`
This uses the observation that if we move along the path and maybe the goal
moves a bit, then it would be wasteful to throw the previous ideal path away. If
your map is big enough it might be worth to use this instead of using
`find_path` for updates on the path finding.

# Tips

* Initialize only one Pathfinding instance, it will use a cached internal map
  that would need to be rebuilt if you initialize it for every path to be found.
* Use `find_path_update` if you walked a path before, but need to do some
  adjustments. Maybe your target moves, maybe you move.

# License
MIT License (2014)
