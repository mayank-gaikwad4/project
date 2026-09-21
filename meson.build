project('delivery_fleet_sim', ['cpp', 'c'],
  version : '1.0',
  default_options : [
    'cpp_std=c++23',
    'c_std=c11',
    'warning_level=3',
    'default_library=static',        # Force all subprojects/wraps to build as static libraries
    'b_vscrt=static_from_buildtype'  # Statically links MSVC CRT (/MT for release, /MTd for debug)
  ]
)

cpp = meson.get_compiler('cpp')
c = meson.get_compiler('c')
is_windows = host_machine.system() == 'windows'

# ==============================================================================
# Windows Static Linking (MinGW / GCC / Clang)
# ==============================================================================
if is_windows and cpp.get_id() != 'msvc'
  add_project_link_arguments(
    '-static',
    '-static-libgcc',
    '-static-libstdc++',
    language : ['cpp', 'c']
  )
endif

# ==============================================================================
# Architecture: x86-64-v3 Baseline (Applies to ALL build types)
# ==============================================================================
if cpp.get_id() == 'msvc'
  # MSVC equivalent of x86-64-v3 is AVX2 (Includes AVX2, FMA, BMI1, BMI2)
  add_project_arguments('/arch:AVX2', language : ['cpp', 'c'])
else
  add_project_arguments('-march=x86-64-v3', language : ['cpp', 'c'])
endif

# ==============================================================================
# Build Type Configurations (Debug vs Release)
# ==============================================================================
buildtype = get_option('buildtype')

if buildtype.startswith('debug')
  # --- Robust Debug & Sanitization ---
  if cpp.get_id() == 'msvc'
    add_project_arguments(
      '/sdl',               # Security Development Lifecycle checks
      '/RTC1',              # Run-Time Error Checks
      '/fsanitize=address', # Address Sanitizer (See note below regarding MSVC CRT)
      language : ['cpp', 'c']
    )
  else
    add_project_arguments(
      '-fno-omit-frame-pointer',
      '-fstack-protector-strong',
      '-D_GLIBCXX_DEBUG',
      '-D_LIBCPP_ENABLE_ASSERTIONS=1',
      '-D_FORTIFY_SOURCE=3',
      '-fsanitize=address,undefined',
      '-fsanitize-address-use-after-scope',
      language : ['cpp', 'c']
    )
    add_project_link_arguments('-fsanitize=address,undefined', language : ['cpp', 'c'])
  endif

elif buildtype == 'release'
  # --- Max Speed Release Mode ---
  if cpp.get_id() == 'msvc'
    add_project_arguments(
      '/O2', '/Ob2', '/Oi', '/Ot', '/Oy', '/GL', '/DNDEBUG',
      language : ['cpp', 'c']
    )
    add_project_link_arguments('/LTCG', language : ['cpp', 'c'])
  else
    add_project_arguments(
      '-O3', '-flto', '-DNDEBUG',
      language : ['cpp', 'c']
    )
    add_project_link_arguments('-flto', '-O3', language : ['cpp', 'c'])
  endif
endif

# ==============================================================================
# Dependencies
# ==============================================================================
# Force static target selection for SDL3 and pass static default options to fallback subprojects
sdl3_dep = dependency(
  'sdl3',
  required : true,
  static : true,
  fallback : ['sdl3', 'sdl3_dep'],
  default_options : ['default_library=static']
)

# SQLite requires threading and dynamic linking on Linux
thread_dep = dependency('threads')
dl_dep = c.find_library('dl', required : false)

# ==============================================================================
# Sources
# ==============================================================================
sources = [
  'main.cpp',
  'sqlite3.c'  # Compiled directly from your shipped amalgamation file
]

# ==============================================================================
# Executable
# ==============================================================================
executable('fleet_sim',
  sources,
  dependencies : [sdl3_dep, thread_dep, dl_dep],
  install : true
)