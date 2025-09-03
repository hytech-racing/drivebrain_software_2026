#ifndef __PROTOBUFUTILS_H__
#define __PROTOBUFUTILS_H__

#include <google/protobuf/descriptor.h>
#include <google/protobuf/descriptor.pb.h>

#include <optional>
#include <unordered_set>
#include <algorithm>
#include <string>

// TODO figure out if this is the best place to include this or nah
#include <hytech_msgs.pb.h>
#include <tuple>
#include <vector>
#include <string>
#include <queue>
namespace util
{
    std::vector<const google::protobuf::FileDescriptor *> get_pb_descriptors(const std::vector<std::string> &filenames);
    std::optional<std::unordered_map<std::string, uint32_t>> generate_name_to_id_map(const std::vector<std::string> &filenames);
    google::protobuf::FileDescriptorSet build_file_descriptor_set(const google::protobuf::Descriptor *toplevelDescriptor);
}
#endif // __PROTOBUFUTILS_H__