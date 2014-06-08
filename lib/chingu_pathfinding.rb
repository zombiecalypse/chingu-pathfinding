class Pathfinding
  VERSION = '1.1.1'
  def find_path(x1, y1, x2, y2)
    throw "Require points to be numeric, got #{[x1, y1, x2, y2]}." unless [x1,y1,x2,y2].all? {|x| x.kind_of?(Numeric)}
    _find_path(x1, y1, x2, y2)
  end
  def find_path_update(x1, y1, x2, y2, current)
    throw "Require points to be numeric, got #{[x1, y1, x2, y2]}." unless [x1,y1,x2,y2].all? {|x| x.kind_of?(Numeric)}
    unless current.kind_of?(Array) and current.all? {|p| p.kind_of?(Array) and p.length == 2 and p.all? {|e| e.kind_of?(Numeric)} }
      throw "Require current to be list of points, got #{current}."
    end
    _find_path_update(x1,y1, x2, y2, current.collect(&:dup))
  end
end

require_relative 'chingu_pathfinding/chingu_pathfinding'
