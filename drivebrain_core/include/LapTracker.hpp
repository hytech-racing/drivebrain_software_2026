#include <StateTracker.hpp> 
#include <hytech_msgs.pb.h> 
#include <atomic>
#include <chrono>
#include <memory>

// TODO Add any imports you might think are neccesary here

namespace core {

    class LapTracker {

        public: 

            /* Singleton move semantics */
            LapTracker(const LapTracker&) = delete;
            LapTracker& operator=(const LapTracker&) = delete;

            /**
             * Steps the lap tracker 
             * 
             * @param latest_state the latest state (will be updated in the main loop after get_latest_state_and_validity 
             * is called by the state tracker)
             */
            void step_tracker(core::VehicleState& latest_state);

            /**
             * Creates a new instance of the lap tracker
             */
            static void create(); 

            /**
             * Returns the lap tracker instance
             */
            static LapTracker& instance(); 
            

        private:

            LapTracker();
        
            /** Internal State */
            inline static std::atomic<LapTracker*> _s_instance;
            
            // TODO put variables here to keep track of the lap tracker's internal state
            int _lapcount = 0;
            float _laptime_seconds = 0.0f;
            bool _has_start_reference = false;
            bool _lap_started = false;
            bool _was_outside_start_zone = true;
            double _start_lat = 0.0;
            double _start_lon = 0.0;
            std::chrono::steady_clock::time_point _lap_start_timestamp;
            
    };

}