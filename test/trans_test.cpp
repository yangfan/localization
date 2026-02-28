#include "Trans.hpp"

#include <iostream>
#include <thread>

#include <glog/logging.h>
#include <gtest/gtest.h>

void customer() {
  int res = 0;
  LOG(INFO) << "res: " << res;
  while (res != 1) {
    res = Trans<int>::instance().consume();
    LOG(INFO) << "getting res: " << res;
    std::this_thread::sleep_for(std::chrono::milliseconds(300));
  }
}
void producer() {
  for (int i = 200; i > 0; --i) {
    Trans<int>::instance().produce(i);
    LOG(INFO) << "making product " << i;
    std::this_thread::sleep_for(std::chrono::milliseconds(300));
  }
}

TEST(TRANS, Basic) {
  std::thread tc(customer);
  std::thread tc2(customer);
  std::thread tp(producer);

  if (tp.joinable()) {
    tp.join();
  }

  if (tc.joinable()) {
    tc.join();
  }
  if (tc2.joinable()) {
    tc2.join();
  }
}

int main(int argc, char **argv) {
  testing::InitGoogleTest(&argc, argv);
  FLAGS_stderrthreshold = google::INFO;
  FLAGS_colorlogtostderr = true;
  google::InitGoogleLogging(argv[0]);

  return RUN_ALL_TESTS();
}
