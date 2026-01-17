#include <variant>      
#include <type_traits>
#include <mutex>
#include <string>
#include <unordered_map>

namespace core {
  using ParamType = std::variant<bool, int, float, double, std::string, std::monostate>;
  struct Param { bool live; ParamType val; };

  class Configurable {
    public:
      std::string name() { return _name; } 
      std::unordered_map<std::string, Param> get_params() { return _params; };

      bool register_param(std::string component_name, Param param) {}


    private:
      std::string _name;
      std::unordered_map<std::string, Param> _params;

      std::mutex mtx;
  };



}
