# `maliput-integration`

This package contains integration examples and tools that unify `maliput` core
and its possible backends.

# Applications

## `dragway_to_urdf`

## How to run it?

The executable `dragway_to_urdf` allows one create a URDF representation of a
dragway. To run `dragway_to_urdf`, execute:

    dragway_to_urdf \
          --dirpath=[dirpath] \
          --file_name_root=[file name root] \
          --lane_width=[lane width] \
          --length=[length of road in meters] \
          --num_lanes=[number of lanes] \
          --shoulder_width=[width of shoulder in meters]

For an explanation on what the above-mentioned parameters mean, execute:

    dragway_to_urdf --help

One the above command is executed, the following files should exist:

  1. `[dirpath]/[file name root].urdf` -- a [URDF](http://wiki.ros.org/urdf)
     description of the dragway.
  2. `[dirpath]/[file name root].obj` -- a
     [Wavefront OBJ](https://en.wikipedia.org/wiki/Wavefront_.obj_file) mesh of
     the dragway.
  3. `[dirpath]/[file name root].mtl` -- a material file that applies textures
     and colors to the above Wavefront OBJ file.
