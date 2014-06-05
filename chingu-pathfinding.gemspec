require_relative 'lib/chingu_pathfinding'

Gem::Specification.new do |s|
  s.name        = 'chingu-pathfinding'
  s.version     = Pathfinding::VERSION
  s.licenses    = ['MIT']
  s.homepage    = 'https://github.com/zombiecalypse/chingu-pathfinding'
  s.summary     = "Find paths fast with the A* algorithm"
  s.description = "Find paths C supported on a static map with the A* algorithm."
  s.authors     = ["zombiecalypse"]
  s.email       = 'maergil@gmail.com'
  s.files       = Dir['lib/**/*.rb'] + Dir['ext/**/*.cpp']
  s.extensions  = ['ext/chingu_pathfinding/extconf.rb']
end
