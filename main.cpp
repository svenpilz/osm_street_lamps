#include <algorithm>
#include <fstream>
#include <numbers>

#include <boost/endian.hpp>

#include <range/v3/view.hpp>

#include <spdlog/spdlog.h>

#include <zlib.h>

#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>

#include <osmpbf/fileformat.pb.h>
#include <osmpbf/osmformat.pb.h>

std::optional<std::uint32_t> value_for_key(auto &&element,
                                           const std::uint32_t &key) {
  for (decltype(element.keys().size()) i = 0; i < element.keys().size(); ++i) {
    if (element.keys(i) == key) {
      return element.vals(i);
    }
  }

  return std::nullopt;
}

bool has_key(auto &&element, const std::uint32_t &key) {
  return std::ranges::find(element.keys(), key) != element.keys().end();
}

bool has_attribute(auto &&element, const std::uint32_t &key,
                   const std::uint32_t &value) {
  const auto entries = ranges::views::zip(element.keys(), element.vals());
  return std::ranges::find(entries, std::make_pair(key, value)) !=
         entries.end();
}

bool has_attribute(auto &&keys, auto &&vals, const std::uint32_t &key,
                   const std::uint32_t &value) {
  const auto entries = ranges::views::zip(keys, vals);
  return std::ranges::find(entries, std::make_pair(key, value)) !=
         entries.end();
}

struct StringAttributeMap {
  StringAttributeMap(const OSMPBF::PrimitiveBlock &block) {
    for (int i = 0; i < block.stringtable().s_size(); ++i) {
      const auto &s = block.stringtable().s(i);
      if (s == "highway") {
        highway = i;
      } else if (s == "street_lamp") {
        street_lamp = i;
      } else if (s == "lit") {
        lit = i;
      } else if (s == "no") {
        no = i;
      }
    }
  }

  std::uint32_t highway = 0;
  std::uint32_t street_lamp = 0;
  std::uint32_t lit = 0;
  std::uint32_t no = 0;
};

struct BoundingBox {
  static BoundingBox fromPoints(auto &&points) {
    BoundingBox result;

    for (const auto &point : points) {
      result.min.x = std::min(result.min.x, point.x);
      result.min.y = std::min(result.min.y, point.y);
      result.max.x = std::max(result.max.x, point.x);
      result.max.y = std::max(result.max.y, point.y);
    }

    return result;
  }

  auto ratio() const { return (max.x - min.x) / (max.y - min.y); }

  using numeric_limits = std::numeric_limits<cv::Point2d::value_type>;
  cv::Point2d min{numeric_limits::infinity(), numeric_limits::infinity()};
  cv::Point2d max{-numeric_limits::infinity(), -numeric_limits::infinity()};
};

struct Node {
  Node(double lon, double lat, bool is_street_lamp)
      : lon{lon}, lat{lat}, is_street_lamp{is_street_lamp} {
    using std::numbers::pi;
    using namespace std;
    const auto rad = [](const auto &deg) { return deg / 180.0 * pi; };

    x = 1.0 / (2 * pi) * 2 * (rad(lon) + pi);
    y = 1.0 / (2 * pi) * 2 * (pi - log(tan(pi / 4.0 + rad(lat) / 2.0)));
  }

  double lon;
  double lat;
  bool is_street_lamp;

  double x;
  double y;

  size_t refcount = 0;
};

using NodeRef = std::int64_t;

struct Road {
  std::vector<NodeRef> nodes;
  std::optional<bool> is_lit;
};

std::vector<char> zlib(const OSMPBF::Blob &blob) {
  decltype(zlib(blob)) result;

  if (!blob.has_zlib_data()) {
    return result;
  }

  result.resize(blob.raw_size());
  size_t uncompressedSize = result.size();
  if (uncompress(reinterpret_cast<Bytef *>(result.data()), &uncompressedSize,
                 reinterpret_cast<Bytef *>(
                     const_cast<char *>(blob.zlib_data().c_str())),
                 blob.zlib_data().size()) != Z_OK) {
    spdlog::critical("[zlib] uncompress failed.");
  }

  if (uncompressedSize != result.size()) {
    spdlog::critical(
        "[zlib] uncompressed size does not match expected one in header.");
  }

  return result;
}

size_t read_pbf_segment(std::ifstream &stream,
                        std::unordered_map<NodeRef, Node> &points,
                        std::vector<Road> &roads) {
  boost::endian::big_int32_t size;
  stream.read(reinterpret_cast<char *>(&size), sizeof(size));

  if (size == 0) {
    return 0;
  }

  std::vector<char> buffer(size);
  stream.read(buffer.data(), buffer.size());

  OSMPBF::BlobHeader blobheader;
  if (!blobheader.ParseFromArray(buffer.data(), buffer.size())) {
    spdlog::critical(
        "Failed to parse OSMPBF::BlobHeader (size is given as {} bytes).",
        size);
    return 0;
  }

  buffer.resize(blobheader.datasize());
  stream.read(buffer.data(), buffer.size());
  OSMPBF::Blob blob;
  if (!blob.ParseFromArray(buffer.data(), buffer.size())) {
    spdlog::critical(
        "Failed to parse OSMPBF::Blob (size is given as {} bytes).",
        blobheader.datasize());
    return 0;
  }

  if (!blob.has_zlib_data() || blob.has_lzma_data() || blob.has_raw()) {
    spdlog::critical(
        "[read_pbf_segment] Found a segment that is not zlib compressed. Only "
        "zlib compressed segments are supported.");
    return 0;
  }

  auto blockbuffer = zlib(blob);

  if (blobheader.type() == "OSMHeader") {

  } else if (blobheader.type() == "OSMData") {
    OSMPBF::PrimitiveBlock block;
    if (!block.ParseFromArray(blockbuffer.data(), blockbuffer.size())) {
      spdlog::critical("Failed to parse block");
    }

    const auto unpackLongitude = [&](const auto &packedLon) {
      return 0.000000001 *
             (block.lon_offset() + (block.granularity() * packedLon));
    };

    const auto unpackLatitude = [&](const auto &packedLat) {
      return 0.000000001 *
             (block.lat_offset() + (block.granularity() * packedLat));
    };

    const StringAttributeMap strings{block};

    for (const auto &group : block.primitivegroup()) {
      //
      // Nodes
      //
      std::transform(
          group.nodes().begin(), group.nodes().end(),
          std::inserter(points, points.begin()),
          [&](const auto &node) -> std::decay_t<decltype(points)>::value_type {
            const auto is_street_lamp =
                has_attribute(node, strings.highway, strings.street_lamp);
            return {std::piecewise_construct, std::forward_as_tuple(node.id()),
                    std::forward_as_tuple(unpackLongitude(node.lon()),
                                          unpackLatitude(node.lat()),
                                          is_street_lamp)};
          });

      //
      // DenseNodes
      //
      if (group.has_dense()) {
        const auto &dense = group.dense();

        std::int64_t id = 0;
        std::int64_t packedLat = 0;
        std::int64_t packedLon = 0;

        using namespace ranges::views;
        for (const auto &[delta_id, delta_lat, delta_lon, attr] :
             zip(dense.id(), dense.lat(), dense.lon(),
                 dense.keys_vals() | split(0))) {
          id += delta_id;
          packedLat += delta_lat;
          packedLon += delta_lon;

          const auto keys = attr | stride(2);
          const auto vals = attr | tail | stride(2);
          const auto is_street_lamp =
              has_attribute(keys, vals, strings.highway, strings.street_lamp);

          points.emplace(std::piecewise_construct, std::forward_as_tuple(id),
                         std::forward_as_tuple(unpackLongitude(packedLon),
                                               unpackLatitude(packedLat),
                                               is_street_lamp));
        }
      }

      //
      // Ways
      //
      const auto is_highway = [&](const auto &way) {
        return has_key(way, strings.highway);
      };

      std::ranges::transform(group.ways() | std::views::filter(is_highway),
                             std::back_inserter(roads), [&](const auto &road) {
                               std::decay_t<decltype(roads)>::value_type result;

                               if (const auto lit_attr =
                                       value_for_key(road, strings.lit)) {
                                 result.is_lit = *lit_attr != strings.no;
                               }

                               std::int64_t node = 0;
                               result.nodes.reserve(road.refs().size());
                               for (const auto &delta_ref : road.refs()) {
                                 node += delta_ref;
                                 result.nodes.emplace_back(node);
                                 ++points.at(node).refcount;
                               }

                               return result;
                             });
    }
  }

  return blob.raw_size();
}

int main(int argc, char *argv[]) {
  GOOGLE_PROTOBUF_VERIFY_VERSION;

  if (argc != 3) {
    spdlog::critical("Usage: {} pbf-file out-image-file", argv[0]);
    return -1;
  }

  const auto inputFile = argv[1];
  const auto outputFile = argv[2];

  //
  // Parse OSM export
  //

  std::ifstream file{inputFile, std::ios::binary};
  if (!file.is_open()) {
    spdlog::critical("Failed to open PBF file \"{}\".", inputFile);
    return -2;
  }

  std::unordered_map<NodeRef, Node> nodes;
  std::vector<Road> roads;

  spdlog::info("Start parsing of PBF file.");

  size_t raw_size = 0;
  while (!file.eof()) {
    raw_size += read_pbf_segment(file, nodes, roads);
  }

  spdlog::info("Parsed {} bytes of data containing {} nodes and {} ways (with "
               "highway attribute).",
               raw_size, nodes.size(), roads.size());

  //
  // Define canvas
  //
  const auto bb = BoundingBox::fromPoints(
      nodes | std::views::values | std::views::filter([](const auto &point) {
        return point.refcount > 0 || point.is_street_lamp;
      }));
  cv::Mat image = cv::Mat::zeros(1024 * 5 / bb.ratio(), 1024 * 5, CV_8UC3);

  const auto imagePostion = [&](const auto &point) -> cv::Point2d {
    const auto x = [&](const auto &p) {
      return (p.x - bb.min.x) / (bb.max.x - bb.min.x) * image.cols;
    };

    const auto y = [&](const auto &p) {
      return (p.y - bb.min.y) / (bb.max.y - bb.min.y) * image.rows;
    };

    return {x(point), y(point)};
  };

  //
  // Draw Roads
  //
  size_t numLitRoads = 0;
  size_t numNonLitRoads = 0;
  size_t numRoadsWithoutLitTag = 0;

  for (const auto &road : roads) {
    const auto color = [&]() -> cv::Scalar {
      if (road.is_lit.has_value()) {
        if (road.is_lit.value()) {
          ++numLitRoads;
          return {0, 190, 190};
        } else {
          ++numNonLitRoads;
          return {0, 0, 63};
        }
      } else {
        ++numRoadsWithoutLitTag;
        return {63, 0, 0};
      }
    }();

    for (const auto &[startIndex, endIndex] :
         ranges::views::zip(road.nodes | ranges::views::drop_last(1),
                            road.nodes | ranges::views::drop(1))) {
      const auto &start = nodes.at(startIndex);
      const auto &end = nodes.at(endIndex);
      cv::line(image, imagePostion(start), imagePostion(end), color);
    }
  }
  spdlog::info("Lit: Yes={}, No={}, Unknown={}.", numLitRoads, numNonLitRoads,
               numRoadsWithoutLitTag);

  //
  // Draw Street Lamps
  //
  size_t numStreetLamps = 0;
  for (const auto &lamp :
       nodes | std::views::values | std::views::filter([](const auto &point) {
         return point.is_street_lamp;
       })) {
    cv::circle(image, imagePostion(lamp), 2, {0, 255, 255}, cv::FILLED);
    ++numStreetLamps;
  }
  spdlog::info("Found {} street lamps.", numStreetLamps);

  if (!cv::imwrite(outputFile, image)) {
    spdlog::critical("Failed to write output to \"{}\".", outputFile);
    return -3;
  }

  return 0;
}
