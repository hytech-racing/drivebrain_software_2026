file(REMOVE_RECURSE
  "libhytech_msgs_cpp_lib.a"
  "libhytech_msgs_cpp_lib.pdb"
)

# Per-language clean rules from dependency scanning.
foreach(lang CXX)
  include(CMakeFiles/hytech_msgs_cpp_lib.dir/cmake_clean_${lang}.cmake OPTIONAL)
endforeach()
