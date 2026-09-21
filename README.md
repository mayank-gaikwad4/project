# project

 git clone --depth=1 https://github.com/mayank-gaikwad4/project.git


 then get into the directory

 aand  

 meson setup <build_dirctory name> --buildtype=release
meson compile -C <build directory name>

also can use option --buildtype=debug (in case it crashes)

meson setup p
meson compile -C p

then go into the directory and run the program which generates the .db file

also if you run it with a number it is used for random number generation
fleet_sim  <seed> 
