from os import environ

VariantDir('build/src', 'src', duplicate=0)
VariantDir('build/lib', 'lib', duplicate=0)
flags = ['-O3', '-march=native', '-std=c++14']

env = Environment(ENV       = environ,
                  CXX       = 'clang++',
                  CPPFLAGS  = ['-Wno-unused-value', '-Wno-deprecated-declarations'],
                  CXXFLAGS  = flags,
                  LINKFLAGS = flags,
                  CPPPATH   = ['#simpleini', '#lib/include', '#src/include'],
                  LIBS      = ['SDL2', 'SDL2_image', 'SDL2_ttf', 'rt'])

# Build main emulator
env.Program('laines', Glob('build/*/*.cpp') + Glob('build/*/*/*.cpp'))

# Build test reader utility
test_env = Environment(ENV       = environ,
                       CXX       = 'g++',
                       CXXFLAGS  = ['-O2', '-std=c++11'],
                       LIBS      = ['rt', 'pthread'])
test_env.Program('test_reader', 'test_reader.cpp')
