project('delivery_fleet_sim', ['cpp', 'c'],
  version : '1.0',
  default_options : [
    'cpp_std=c++latest',
    'c_std=c11',
    'warning_level=3'
  ]
)

cpp = meson.get_compiler('cpp')
c = meson.get_compiler('c')
fs = import('fs')

is_windows = host_machine.system() == 'windows'
is_msvc_syntax = cpp.get_argument_syntax() == 'msvc' # True for MSVC and Clang-CL

# ==============================================================================
# Architecture: x86-64-v3 Baseline (Applies to ALL build types)
# ==============================================================================
if is_msvc_syntax
  add_project_arguments('/arch:AVX2', language : ['cpp', 'c'])
else
  add_project_arguments('-march=x86-64-v3', language : ['cpp', 'c'])
endif

# ==============================================================================
# Build Type Configurations (Debug vs Release)
# ==============================================================================
buildtype = get_option('buildtype')

if buildtype.startswith('debug')
  if is_msvc_syntax
    add_project_arguments('/sdl', '/RTC1', language : ['cpp', 'c'])
  else
    debug_flags = [
      '-fno-omit-frame-pointer',
      '-fstack-protector-strong',
      '-D_GLIBCXX_DEBUG',
      '-D_LIBCPP_ENABLE_ASSERTIONS=1'
    ]
    if not is_windows
      debug_flags += [
        '-D_FORTIFY_SOURCE=3',
        '-fsanitize=address,undefined',
        '-fsanitize-address-use-after-scope'
      ]
      add_project_link_arguments('-fsanitize=address,undefined', language : ['cpp', 'c'])
    endif
    add_project_arguments(debug_flags, language : ['cpp', 'c'])
  endif

elif buildtype == 'release'
  if is_msvc_syntax
    add_project_arguments('/Ob2', '/Oi', '/Ot', '/Oy', '/GL', '/DNDEBUG', language : ['cpp', 'c'])
    add_project_link_arguments('/LTCG', language : ['cpp', 'c'])
  else
    add_project_arguments('-O3', '-flto', '-DNDEBUG', language : ['cpp', 'c'])
    add_project_link_arguments('-flto', '-O3', language : ['cpp', 'c'])
  endif
endif

# ==============================================================================
# Dependencies
# ==============================================================================
if is_windows
  sdl3_dir = meson.current_source_dir() / 'deps' / 'sdl3_win'

  # Download and extract the development ZIP (SDL3-devel-*-VC.zip) if missing
  if not fs.is_dir(sdl3_dir)
    message('Downloading prebuilt SDL3 3.4.16 VC development package for Windows...')
    py = import('python').find_installation('python3')
    dl_script = '''
import io, os, zipfile, urllib.request

url = "https://github.com/libsdl-org/SDL/releases/download/release-3.4.16/SDL3-devel-3.4.16-VC.zip"
req = urllib.request.urlopen(url)
with zipfile.ZipFile(io.BytesIO(req.read())) as z:
    for member in z.infolist():
        # Strip top-level root folder from zip (SDL3-3.4.16/)
        parts = member.filename.split("/", 1)
        if len(parts) > 1 and parts[1]:
            target_path = os.path.join("deps", "sdl3_win", parts[1])
            if member.is_dir():
                os.makedirs(target_path, exist_ok=True)
            else:
                os.makedirs(os.path.dirname(target_path), exist_ok=True)
                with z.open(member) as source, open(target_path, "wb") as target:
                    target.write(source.read())
'''
    run_command(py, '-c', dl_script, check : true)
  endif

  inc_dir = include_directories('deps/sdl3_win/include')
  lib_path = meson.current_source_dir() / 'deps/sdl3_win/lib/x64'

  # Modern GCC (MinGW), Clang, and MSVC can all link directly against SDL3.lib
  sdl3_lib = c.find_library('SDL3', dirs : lib_path)

  sdl3_dep = declare_dependency(
    include_directories : inc_dir,
    dependencies : sdl3_lib
  )

  # Copy SDL3.dll directly to the build folder alongside the executable
  configure_file(
    input : 'deps/sdl3_win/lib/x64/SDL3.dll',
    output : 'SDL3.dll',
    copy : true
  )

else
  # Linux: Standard pkg-config / system dependency lookup
  sdl3_dep = dependency('sdl3', required : true)
endif

# SQLite & standard system dependencies
thread_dep = dependency('threads')
dl_dep = c.find_library('dl', required : false)

# ==============================================================================
# Sources & Executable
# ==============================================================================
sources = [
  'main.cpp',
  'sqlite3.c'
]

executable('fleet_sim',
  sources,
  dependencies : [sdl3_dep, thread_dep, dl_dep],
  install : true
)
