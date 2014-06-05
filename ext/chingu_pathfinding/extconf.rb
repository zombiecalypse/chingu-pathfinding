require 'mkmf'

def try_compiler_option(opt, &block)
  result = false
  checking_for "#{opt} option to compiler" do
    if result = try_cpp('', opt, &block)
      $CPPFLAGS += ' ' unless $CFLAGS.empty?
      $CPPFLAGS += opt
    end
  end
  result
end

have_header("ruby.h")
have_library("stdc++")
try_compiler_option('-std=c++11')
create_makefile('chingu_pathfinding/chingu_pathfinding')
