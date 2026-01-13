#include <variant>      
#include <type_traits>
#include <mutex>
#include <string>
#include <unordered_map>

namespace core {
  using ParamType = std::variant<bool, int, float, double, std::string, std::monostate>;

  template<typename T, bool live>
  struct Param {
    T val;
  };
  
  template<typename ParamStruct>
  class Configurable {
    public:
      std::string name() { return _name;  }  
      bool register_param() {}
      std::unordered_map<std::string, ParamType> get_params();


    private:
      std::string _name;
      std::unordered_map<std::string, ParamType> _params;

      std::mutex mtx;
      friend ParamStruct;
  };


  /* Syntax-sugar macros for cleanly defining and registering params */

  #define BEGIN_PARAMS(StructName) \
    struct StructName { \
      template<typename ConfigType> \
      void register_all(ConfigType* self) {



  #define END_PARAMS() \
    } \
  };
}
