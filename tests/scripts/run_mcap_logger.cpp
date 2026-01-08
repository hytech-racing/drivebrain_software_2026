#include <MCAPLogger.hpp>
#include <mcap/writer.hpp>

using namespace core;

int main() {
  mcap::McapWriterOptions options("");
  MCAPLogger::create("", options);
  
  MCAPLogger& logger = MCAPLogger::instance();
  return 0;
}
