#include <StateTracker.hpp>
#include <hytech_msgs.pb.h>
#include <chrono>
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

            void set_start_finish_line(double start_lat, double start_lon,
                                      double end_lat, double end_lon);

        private:

            LapTracker();

            /** Internal State */
            inline static std::atomic<LapTracker*> _s_instance;

            std::chrono::steady_clock::time_point _lap_start_time;
            int _lap_count{0};
            float _max_lap_speed_mps{0.f};

            double _line_start_lat{0.};
            double _line_start_lon{0.};
            double _line_end_lat{0.};
            double _line_end_lon{0.};
            double _prev_side{0.};
            bool _has_prev_position{false};
    };

}