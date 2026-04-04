#ifndef __JSONUTILS_H__
#define __JSONUTILS_H__

#include <vector>

#define LOAD_PARAM_OR_FAIL(param_name, param_type, config_struct)                                  \
    do {                                                                                           \
        auto opt_##param_name = get_parameter_value<param_type>(#param_name);                      \
        if (!opt_##param_name)                                                                     \
            return false;                                                                          \
        (config_struct).param_name = *opt_##param_name;                                            \
    } while (0)

#define LOAD_LIVE_PARAM_OR_FAIL(param_name, param_type, config_struct)                             \
    do {                                                                                           \
        auto opt_##param_name = get_live_parameter<param_type>(#param_name);                       \
        if (!opt_##param_name)                                                                     \
            return false;                                                                          \
        (config_struct).param_name = *opt_##param_name;                                            \
    } while (0)

#define HANDLE_LIVE_PARAM_LOCK_AND_LOAD(param_name, param_type, config_struct, config_mutex,       \
                                        param_map)                                                 \
    do {                                                                                           \
        auto it = (param_map).find(#param_name);                                                   \
        if (it != (param_map).end()) {                                                             \
            if (auto pval = std::get_if<param_type>(&it->second)) {                                \
                std::unique_lock<std::mutex> lk(config_mutex);                                     \
                (config_struct).param_name = *pval;                                                \
            }                                                                                      \
        }                                                                                          \
    } while (0)


#endif // __JSONUTILS_H__