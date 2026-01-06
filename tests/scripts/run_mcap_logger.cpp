#include <MCAPLogger.hpp>
#include <mcap/writer.hpp>

using namespace core;

int main() {
  mcap::McapWriterOptions options("");
  MCAPLogger::initialize("", options);
  auto& logger = MCAPLogger::get_instance();
  return 0;
}
