/**
 * HelloWorld for redis publisher.
 * Compile with 
 * $ g++ helloRedisPublisher.cpp -o helloRedisPublisher -std=c++11 -lcpp_redis -ltacopie -lpthread
 * 
 * Created with CPP_Redis: https://github.com/cpp-redis/cpp_redis
 * 
 * Client Doku: https://cylix.github.io/cpp_redis/html/classcpp__redis_1_1client.html
 */

#include <cpp_redis/cpp_redis>

#include <iostream>


int
main(void) {
  //! Enable logging
  cpp_redis::active_logger = std::unique_ptr<cpp_redis::logger>(new cpp_redis::logger);

  cpp_redis::client client;

  client.connect("127.0.0.1", 6379, [](const std::string& host, std::size_t port, cpp_redis::client::connect_state status) {
    if (status == cpp_redis::client::connect_state::dropped) {
      std::cout << "client disconnected from " << host << ":" << port << std::endl;
    }
  });

  client.publish("subscribeTest", "Hello Test");

  // commands are pipelined and only sent when client.commit() is called
  // client.commit();

  // synchronous commit, no timeout
  client.sync_commit();

// synchronous commit, timeout
// client.sync_commit(std::chrono::milliseconds(100));

  return 0;
}