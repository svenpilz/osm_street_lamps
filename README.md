# osm_street_lamps

Renders streets and street lamps from a PBF export. Each street (ways with
`highway=*`) will be rendered as line segments, the color will depend on the
`lit=` attribute. Streets that are lit will be yellow, the ones that are tagged
as not lit red and those without the tag blue. On top of that, each node with
`highway=street_lamp` will be rendered as bright yellow dots.

## Dependencies

* cmake
* git
* Boost
* ZLIB
* Protobuf
* OpenCV

## Building 

1. `git clone https://github.com/svenpilz/osm_street_lamps`
2. `mkdir build`
3. `cd build`
4. `cmake -DCMAKE_BUILD_TYPE=Release ../osm_street_lamps`

## Usage

`./osm_street_lamps exported-pbf-file output-image-file`

Exports can for example be downloaded from http://download.geofabrik.de.
