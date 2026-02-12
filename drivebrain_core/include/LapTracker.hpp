#include <StateTracker.hpp> 
#include <hytech_msgs.pb.h> 
#include <memory>
#include <chrono>

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

            /***
             * distance formula calculator
             * 
             * @param start start position
             * @param curr current position
             */
            void distance_formula(core::Position& start, core::Position& curr);

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
            
            int num_laps = 0; // number of laps completed
            bool valid_lap;
            bool start_set = false;
            
            std::chrono::steady_clock::time_point start_time;
            float start_yaw;

            core::Position start_Position;
            core::Position current_Position;
            float current_yaw;

    };

}