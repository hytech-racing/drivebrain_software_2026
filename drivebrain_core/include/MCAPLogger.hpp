#include <mcap/writer.hpp>
#include <deque> 
#include <thread> 
#include <mutex> 
#include <condition_variable> 
#include <queue> 
#include <google/protobuf/descriptor.pb.h>
#include <foxglove/websocket/base64.hpp>

namespace core {

    struct RawMessage_s {
        std::string serialized_data;
        std::string message_name;
        mcap::Timestamp log_time;
    };

    class MCAPLogger {

        public: 

            /**
             * Constructor for creating a new MCAP Logger instance
             * 
             * @param base_dir the directory in which the log file should be created
             * @param options options to create the mcap with 
             */
            MCAPLogger(const std::string &base_dir, const mcap::McapWriterOptions &options);

            /**
             * Opens a new mcap file by adding options and all protobuf schema
             * 
             * @param name name of the new file to be opened 
             * @return 0 on success, negative err code on failure
             */
            int open_new_mcap(const std::string &name);

            /**
             * Closes the current mcap file. Only runs if there is a file open. 
             * @return 0 on success, negative err code on failure
             */
            int close_active_mcap();

            /**
             * Adds params schema to allow for parameter logging
             * @return 0 on success, negative err code on failure
             */
            // int add_params_schema();  TODO 

            /**
             * Logs a protobuf message 
             * 
             * @param message the message to be logged
             * @return 0 on success, negative err code on failure
             */
            int log_protobuf_message(std::shared_ptr<google::protobuf::Message> message); 

            /**
             * Logs the json params. Doesn't need an input because the schema has already been created. 
             * 
             * @return 0 on success, negative err code on failure
             */
            // int log_params(); TODO

        private: 

            /**
             * Spawned by thread, loops until end of program life or error occurs. 
             * Logs all queued messages to a file. 
             */
            void _handle_log_to_file();     

            std::deque<RawMessage_s> _input_buffer; 
            std::mutex _input_buffer_mutex;
            std::condition_variable _input_buffer_cv; 

            mcap::McapWriter _writer;   
            mcap::McapWriterOptions _options;
            
            std::unordered_map<std::string, uint32_t> _name_to_id_map;
            bool _running = true;

    };

}