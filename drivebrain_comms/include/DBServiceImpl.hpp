#ifndef DBSERVICE_IMPL_HPP
#define DBSERVICE_IMPL_HPP

#include "VNComms.hpp"
#include <db_service/v1/service/db_interface.grpc.pb.h>
#include <google/protobuf/message.h>
#include <grpcpp/ext/proto_server_reflection_plugin.h>
#include <iostream>
#include <memory>
#include <string>

#include <grpcpp/grpcpp.h>

#include <MsgLogger.hpp>
#include <Controllers.hpp>
#include <ControllerManager.hpp>

class DBInterfaceImpl final : public db_service::v1::service::DBInterface::Service {

    grpc::Status RequestStopLogging(grpc::ServerContext* context, const google::protobuf::Empty *rq, db_service::v1::service::LoggerStatus * response) override; 
    grpc::Status RequestStartLogging(grpc::ServerContext* context, const google::protobuf::Empty *rq, db_service::v1::service::LoggerStatus * response) override;
    grpc::Status RequestCurrentLoggerStatus(grpc::ServerContext* context, const google::protobuf::Empty* rq, db_service::v1::service::LoggerStatus* response) override;
    grpc::Status RequestControllerChange(grpc::ServerContext* context, const db_service::v1::service::DesiredController* rq, db_service::v1::service::ControllerChangeStatus* response) override; 

    public: 
        DBInterfaceImpl(std::function<bool(size_t)> mode_switch);        
        void run_server(); 
        void stop_server();
        void update_msg_logger(std::shared_ptr<core::MsgLogger<std::shared_ptr<google::protobuf::Message>>> logger_inst) {
            _logger_inst = logger_inst;
        }
    private:
        std::shared_ptr<core::MsgLogger<std::shared_ptr<google::protobuf::Message>>> _logger_inst;
        std::function<bool(size_t)> _mode_switch;
        std::unique_ptr<grpc::Server> _server;  // Store server instance here

};

#endif