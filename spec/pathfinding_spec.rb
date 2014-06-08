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

describe Pathfinding do
  subject { Dummy.new(Dummy::MAP.first.length, Dummy::MAP.length, 5) }

  context "#find_path" do
    it "should find a straight line" do
      expect(subject.find_path(2,2,12,2)).to eq([[2, 2], [7, 2], [12, 2]])
    end

    it "should find a straight line with prefix" do
      expect(subject.find_path(0,0,12,2)).to eq([[0,0], [2, 2], [7, 2], [12, 2]])
    end

    it "should find a straight line with postfix" do
      expect(subject.find_path(0,0,14,2)).to eq([[0,0], [2, 2], [7, 2], [12, 2], [14, 2]])
    end

    it "should find a complex path" do
      expect(subject.find_path(2,12, 37, 12)).to eq([
        [2, 12], [2, 7], [2, 2], [7, 2], [12, 2],
        [17, 2], [22, 2], [27, 2], [32, 2], [37, 2],
        [42, 2], [42, 7], [42, 12], [37, 12]])
    end
  end

  context "#find_path_update" do
    let(:path) { [
        [2, 12], [2, 7], [2, 2], [7, 2], [12, 2],
        [17, 2], [22, 2], [27, 2], [32, 2], [37, 2],
        [42, 2], [42, 7], [42, 12], [37, 12]] }
    it "should give same path if no changes were made" do
      expect(subject.find_path_update(2,12, 37, 12, path)).to eq(path)
    end
    it "should make simple modifications in the goal" do
      expect(subject.find_path_update(2,12, 39, 12, path)).to eq(path - [[37, 12]] +[[39, 12]])
    end
    it "should make simple modifications in the beginning" do
      expect(subject.find_path_update(0,12, 37, 12, path)).to eq([[0,12]] + path)
    end

    it "should remove visited path in the beginning" do
      expect(subject.find_path_update(22,2, 37, 12, path)).to eq(path.drop_while {|e| e != [22, 2]} )
    end
  end

  context "#to_s" do
    it "should display itself" do
      expect(subject.to_s).to eq(['map:',
         '                                                  ',
         '                                                  ',
         '  o.  .o.  .o.  .o.  .o.  .o.  .o.  .o.  .o.  .o  ',
         '  .                             ..  ...  ...  ..  ',
         '                                                  ',
         '                                                  ',
         '  .                             ..  ...  ...  ..  ',
         '  o    ~    o.  .o    ~    ~    o.  .o.  .o.  .o  ',
         '  ..       ...  ..                        ..  ..  ',
         '                                                  ',
         '                                                  ',
         '  .   . .   ..  ..                        ..  ..  ',
         '  o.  .o.  .o.  .o    ~    ~    o.  .o.  .o.  .o  ',
         '                                                  ',
         '                                                  ',
         ''].join("\n"))
    end
  end
end
