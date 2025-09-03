if (Protobuf_FOUND AND Nanopb_FOUND)
    message(SEND_ERROR "You should not be using both nanopb and protobuf.")
elseif(Protobuf_FOUND) 
    add_library(hytech_msgs_cpp_lib)

    set(PROTO_OUT_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR})

    protobuf_generate(
        TARGET hytech_msgs_cpp_lib
        LANGUAGE cpp
        IMPORT_DIRS proto
        DESCRIPTORS
        PROTOC_OUT_DIR ${PROTO_OUT_DIR}
        PROTOS
        proto/hytech_msgs.proto
        proto/base_msgs.proto
    )

    target_include_directories(hytech_msgs_cpp_lib PUBLIC ${PROTO_OUT_DIRECTORY})

    target_link_libraries(hytech_msgs_cpp_lib protobuf::libprotobuf)
elseif(Nanopb_FOUND)
    set(PROTO_OUT_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR})

    NANOPB_GENERATE_CPP(
        TARGET hytech_msgs_nanopb_lib
        proto/hytech_msgs.proto
        proto/base_msgs.proto
    )

    target_include_directories(hytech_msgs_nanopb_lib PUBLIC ${PROTO_OUT_DIRECTORY})

    target_link_libraries(hytech_msgs_nanopb_lib nanopb)
else()
    message(SEND_ERROR "Neither nanopb or protobuf was found. Cannot compile protocol buffer library. Please call find_package with either Protobuf or Nanopb as one of the inputs.")
endif()