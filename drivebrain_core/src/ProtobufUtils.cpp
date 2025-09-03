#include <ProtobufUtils.hpp>
#include <spdlog/spdlog.h>

namespace util
{
    std::vector<const google::protobuf::FileDescriptor *> get_pb_descriptors(const std::vector<std::string> &filenames)
    {
        std::vector<const google::protobuf::FileDescriptor *> descriptors;
        for (const auto &name : filenames)
        {
            const google::protobuf::FileDescriptor *file_descriptor =
                google::protobuf::DescriptorPool::generated_pool()->FindFileByName(name);
            if (!file_descriptor)
            {
                spdlog::error("File descriptor for '{}' not found!", name);
            }
            else
            {
                descriptors.push_back(file_descriptor);
            }
        }
        return descriptors;
    }

    std::optional<std::unordered_map<std::string, uint32_t>> generate_name_to_id_map(const std::vector<std::string> &filenames)
    {
        auto descriptors = get_pb_descriptors(filenames);
        std::unordered_map<std::string, uint32_t> map;

        int running_index = 1;
        for (const auto &desc : descriptors)
        {
            for (int i = 0; i < (desc->message_type_count()); ++i)
            {
                const google::protobuf::Descriptor *message_descriptor = desc->message_type(i);
                map[message_descriptor->name()] = running_index;
                running_index++;
            }
        }
        return map;
    }
    
    google::protobuf::FileDescriptorSet build_file_descriptor_set(
        const google::protobuf::Descriptor *toplevelDescriptor)
    {
        google::protobuf::FileDescriptorSet fdSet;
        std::queue<const google::protobuf::FileDescriptor *> toAdd;
        toAdd.push(toplevelDescriptor->file());
        std::unordered_set<std::string> seenDependencies;
        while (!toAdd.empty())
        {
            const google::protobuf::FileDescriptor *next = toAdd.front();
            toAdd.pop();
            next->CopyTo(fdSet.add_file());
            for (int i = 0; i < next->dependency_count(); ++i)
            {
                const auto &dep = next->dependency(i);
                if (seenDependencies.find(dep->name()) == seenDependencies.end())
                {
                    seenDependencies.insert(dep->name());
                    toAdd.push(dep);
                }
            }
        }
        return fdSet;
    }
}