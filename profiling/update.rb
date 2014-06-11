
require 'ruby-prof'
require_relative '../lib/chingu_pathfinding.rb'

class Dummy < Pathfinding
  MAP = [
    #                          |25
    "..................................................", # 00
    "..................................................", # 01
    "..................................................", # 02
    ".....*************************....................", # 03
    ".....*************************....................", # 04
    ".....*****............********....................", # 05
    ".....*****............********....................", # 06
    ".....*****............********....................", # 07
    "......................********************........", # 08
    "......................********************........", # 09
    "......................********************........", # 10
    "......................********************........", # 11
    "......................********....................", # 12
    "......................********....................", # 13
    "......................********...................."] # 15
      .map {|l| l.split('').map{|e| e != '.'} }
  def blocked?(x, y)
    MAP[y][x]
  end
end

map = Dummy.new(Dummy::MAP.first.length, Dummy::MAP.length, 1)

result = RubyProf.profile do
  100.times { map.find_path_update(2,12, 37, 12) }
end

printer = RubyProf::GraphPrinter.new(result)
printer.print(STDOUT, {})