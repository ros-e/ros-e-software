// The MIT License (MIT)
//
// Copyright (c) 2015-2017 Simon Ninon <simon.ninon@gmail.com>
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in all
// copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.

/**
 * HelloWorld for subscribing redis channels.
 * Compile with 
 * $ g++ helloRedisSubscriber.cpp -o helloRedisSubscriber -std=c++11 -lcpp_redis -ltacopie -lpthread
 * 
 * Created with CPP_Redis: https://github.com/cpp-redis/cpp_redis
 * 
 * Subscriber Doku: https://cylix.github.io/cpp_redis/html/classcpp__redis_1_1subscriber.html
 */

#include <cpp_redis/cpp_redis>
#include <tacopie/tacopie>

#include <condition_variable>
#include <iostream>
#include <mutex>
#include <signal.h>

std::condition_variable should_exit;

void sigint_handler(int) {
  should_exit.notify_all();
}

int main(void) {

  //! Enable logging
  cpp_redis::active_logger = std::unique_ptr<cpp_redis::logger>(new cpp_redis::logger);

  cpp_redis::subscriber sub;


  sub.connect("127.0.0.1", 6379, [](const std::string& host, std::size_t port, cpp_redis::subscriber::connect_state status) {
    if (status == cpp_redis::subscriber::connect_state::dropped) {
      std::cout << "client disconnected from " << host << ":" << port << std::endl;
      should_exit.notify_all();
    }
  });

  //! authentication if server-server requires it
  // sub.auth("some_password", [](const cpp_redis::reply& reply) {
  //   if (reply.is_error()) { std::cerr << "Authentication failed: " << reply.as_string() << std::endl; }
  //   else {
  //     std::cout << "successful authentication" << std::endl;
  //   }
  // });

  sub.subscribe("subscribeTest", [](const std::string& chan, const std::string& msg) {
    std::cout << "MESSAGE " << chan << ": " << msg << std::endl;
  });
  sub.psubscribe("*", [](const std::string& chan, const std::string& msg) {
    std::cout << "PMESSAGE " << chan << ": " << msg << std::endl;
  });
  sub.commit();

  signal(SIGINT, &sigint_handler);
  std::mutex mtx;
  std::unique_lock<std::mutex> l(mtx);
  should_exit.wait(l);

  return 0;
}