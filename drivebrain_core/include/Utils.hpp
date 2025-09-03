
#ifndef __UTILS_H__
#define __UTILS_H__

// yoinked from: https://github.com/hytech-racing/MCU/blob/f36ebdafc33327b2d57ce9240225160c66afc9fa/lib/systems/include/Utility.h

// TODO turn these into functions 


/// @brief generic data vector type that can be used with tire and / or anything to do with 4 corners of the car.
#include <cstddef>
template <typename T>
struct veh_vec
{
    T FL;
    T FR;
    T RL;
    T RR;

    template<size_t ind>
    void set_from_index(T val)
    {
        static_assert(ind <= 3, "ERROR: index cannot be greater than 3");
        if constexpr (ind == 0)
        {
            FL = val;
        } else if(ind == 1)
        {
            FR = val;
        } else if(ind == 2)
        {
            RL = val;
        } else {
            RR = val;
        }
    }
    
    template <size_t ind>
    T get_from_index()
    {
        static_assert(ind <= 3, "ERROR: index cannot be greater than 3");
        if constexpr (ind == 0)
        {
            return FL;
        } else if(ind == 1)
        {
            return FR;
        } else if(ind == 2)
        {
            return RL;
        } else {
            return RR;
        }
    }
};

template <typename T>
struct xyz_vec
{
    T x;
    T y;
    T z;
};

template <typename T>
struct ypr_vec 
{   
    T yaw;
    T pitch;
    T roll;
};


#endif // __UTILS_H__
