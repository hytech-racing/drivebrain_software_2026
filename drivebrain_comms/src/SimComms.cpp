#include <SimComms.hpp>

namespace comms {

static std::shared_ptr<google::protobuf::Message> parse_by_name(const std::string& type_name, const void* data, size_t len) 
{
    std::shared_ptr<google::protobuf::Message> msg;

    const google::protobuf::Descriptor *desc = google::protobuf::DescriptorPool::generated_pool()->FindMessageTypeByName(type_name);
    if (!desc) return nullptr; 

    msg.reset(google::protobuf::MessageFactory::generated_factory()->GetPrototype(desc)->New());

    if (!msg) return nullptr; 

    if (!msg->ParseFromArray(data, static_cast<int>(len))) return nullptr;
    return msg;
}

static std::string endpoint(uint16_t port) {
    return "ipc:///tmp/drivebrain_sim_" + std::to_string(port);
}


bool SimComms::init() {
    if (_running.exchange(true)) return false;

    bool rc = _setup_recv_socket(_veh_data_recv_socket, 6767);
    if (!rc) return false;
    // TODO add lidar and camera sockets
    // TODO add send sockets

    _veh_recv_thread = std::thread(&SimComms::_veh_recv_loop, this);

    return true;
}

bool SimComms::close() {
    if (!_running.exchange(false)) return false; 
    if (_veh_recv_thread.joinable()) _veh_recv_thread.join();
    return true;
}

bool SimComms::_setup_recv_socket(zmq::socket_t& s, uint16_t port) {
  try {
    s = zmq::socket_t(_ctx, zmq::socket_type::pull);
    s.set(zmq::sockopt::rcvhwm, 10000);
    s.set(zmq::sockopt::rcvtimeo, 100);
    s.bind(endpoint(port));
  } catch (const zmq::error_t& e) {
    spdlog::error("replay bind failed on port {}: {}", port, e.what());
    _running = false; 
    return false;
  }
  return true;
}

void SimComms::_veh_recv_loop() {
    while (_running) {
        zmq::multipart_t m;
        try {
            if (!m.recv(_veh_data_recv_socket)) continue;  
        } catch (const zmq::error_t& e) {
        if (e.num() == ETERM) break;   
            spdlog::error("SimComms recv error: {}", e.what());
            continue;
        }
        if (m.size() < 2) { spdlog::error("SimComms: short frame, {} parts", m.size()); continue; }
        auto type_name = m.popstr(); 
        auto body = m.pop();

        std::shared_ptr<google::protobuf::Message> msg(parse_by_name(type_name, body.data(), body.size()));

        spdlog::info(type_name);

        if (msg) {
            core::log(msg); 
            core::StateTracker::instance().handle_receive_protobuf_message(msg); 
        }
    }
}

}