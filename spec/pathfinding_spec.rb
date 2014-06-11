require_relative '../lib/chingu_pathfinding.rb'

RSpec::Matchers.define :be_path_from do |x,y|
  chain :to do |gx,gy|
    @gx= gx
    @gy = gy
  end

  chain :on do |map|
    @map = map
  end

  chain :of_length do |len|
    @len = len
  end

  match do |actual|
    return fail_with(:not_list) unless actual.kind_of?(Array)
    return fail_with(:not_list) unless actual.all? {|e| e.kind_of?(Array)}
    return fail_with(:not_start) unless actual.first == [x,y]
    return fail_with(:not_end) unless actual.last == [@gx, @gy] or @gx.nil? or @gy.nil?
    linear_components = actual.zip(actual.drop(1)).take(actual.length - 1)
    @actual_length = 0.0
    linear_components.each do |from, to|
      component_length = Math.sqrt(from.zip(to).collect {|a,b| (a.to_f-b)**2}.inject(:+))
      @actual_length += component_length
      if component_length > 2
        (0..10).each do |t|
          px, py = from.zip(to).collect {|a, b| ((a*(10.0-t) + b * t)/10.0).round }
          if @map.blocked?(px, py)
            @blocked_point = [px, py]
            return fail_with(:blocked)
          end
        end
      else
        [from, to].each do |px, py|
          if @map.blocked?(px, py)
            @blocked_point = [px, py]
            return fail_with(:blocked)
          end
        end
      end
    end

    return fail_with(:too_long) if @len and @actual_length > @len
    true
  end

  failure_message do |actual|
    error = "expected #{actual.inspect} to be a path from (#{x}, #{y})"
    error << " to (#{@gx}, #{@gy})" if @gx and @gy
    error << " of length at most #{@len}" if @len
    error << ", but "
    case @fail
    when :not_list
      error << "it wasn't a list of points."
    when :not_start
      error << "it didn't start with (#{x}, #{y})."
    when :not_end
      error << "it didn't end with (#{@gx}, #{@gy})."
    when :blocked
      bx, by = @blocked_point
      error << "it passes through (#{bx}, #{by}), which is blocked."
    when :too_long
      error << "it is %.3f units long." % @actual_length
    end

    error
  end

  def fail_with(x)
    @fail = x
    false
  end
end

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
  context "block_size of 5" do
    let(:map) { Dummy.new(Dummy::MAP.first.length, Dummy::MAP.length, 5) }
    subject { map }

    context "#find_path", :focus do
      it "should find a straight line" do
        expect(subject.find_path(2,2,12,2)).to be_path_from(2,2).to(12,2).of_length(10).on(map)
      end

      it "should find a straight line with prefix" do
        expect(subject.find_path(0,0,12,2)).to be_path_from(0,0).to(12,2).of_length(13).on(map)
      end

      it "should find a straight line with postfix" do
        expect(subject.find_path(0,0,14,2)).to be_path_from(0,0).to(14,2).of_length(15).on(map)
      end

      it "should find a complex path" do
        expect(subject.find_path(2,12, 37, 12)).to be_path_from(2,12).to(37,12).of_length(64).on(map)
      end
    end

    context "#find_path_update" do
      let(:path) { [
          [2, 12], [2, 7], [2, 2], [7, 2], [12, 2],
          [17, 2], [22, 2], [27, 2], [32, 2], [37, 2],
          [42, 2], [42, 7], [42, 12], [37, 12]] }
      it "should give same path if no changes were made" do
        expect(subject.find_path_update(2,12, 37, 12, path)).to be_path_from(2,12).to(37,12).of_length(64).on(map)
      end
      it "should make simple modifications in the goal" do
        expect(subject.find_path_update(2,12, 39, 12, path)).to be_path_from(2,12).to(39,12).of_length(64).on(map)
      end
      it "should make simple modifications in the beginning" do
        expect(subject.find_path_update(0,12, 37, 12, path)).to be_path_from(0,12).to(37,12).of_length(66).on(map)
      end

      it "should remove visited path in the beginning" do
        expect(subject.find_path_update(22,2, 37, 12, path)).to be_path_from(22,2).to(37,12).of_length(43).on(map)
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

  context "block_size of 1" do
    let(:map) { Dummy.new(Dummy::MAP.first.length, Dummy::MAP.length, 1) }
    subject { map }

    context "#find_path" do
      it "should find a straight line" do
        expect(subject.find_path(2,2,12,2)).to be_path_from(2,2).to(12,2).of_length(10).on(map)
      end

      it "should find a complex path" do
        expect(subject.find_path(2,12, 37, 12)).to be_path_from(2,12).to(37,12).of_length(64).on(map)
      end
    end

    context "#find_path_update" do
      let(:path) { subject.find_path(2,12, 37, 12) }
      it "should give same path if no changes were made" do
        expect(subject.find_path_update(2,12, 37, 12, path)).to be_path_from(2,12).to(37,12).of_length(64).on(map)
      end
      it "should make simple modifications in the goal" do
        expect(subject.find_path_update(2,12, 39, 12, path)).to be_path_from(2,12).to(39,12).of_length(64).on(map)
      end

      it "should make simple modifications in the beginning" do
        expect(subject.find_path_update(0,12, 37, 12, path)).to be_path_from(0,12).to(37,12).of_length(66).on(map)
      end

      it "should remove visited path in the beginning" do
        expect(subject.find_path_update(22,2, 37, 12, path)).to be_path_from(22,2).to(37,12).of_length(34).on(map)
      end
    end
  end
end
