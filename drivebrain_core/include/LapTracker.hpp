#include <StateTracker.hpp> 
#include <hytech_msgs.pb.h> 
#include <memory>

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
            
            int _lapcount = 0;
            float _laptime = 0.0f;
            double _start_lat;
            double _start_lon;
            bool _started = false;
            std::chrono::steady_clock::time_point _last_timestamp;
            
    };

}