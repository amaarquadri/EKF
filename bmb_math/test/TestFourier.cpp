#include <bmb_math/Fourier.h>
#include <gtest/gtest.h>
#include <chrono>
#include <cmath>
#include <complex>
#include <iostream>
#include <random>
#include <vector>

static unsigned seed = std::chrono::system_clock::now()
                           .time_since_epoch()
                           .count();  // NOLINT(cert-err58-cpp)
static auto random_engine =
    std::default_random_engine(seed);  // NOLINT(cert-err58-cpp)
static std::uniform_real_distribution<double> distribution(
    -100, 100);  // NOLINT(cert-err58-cpp)

#define BENCHMARK 0

void testFFT(const size_t& N) {
  std::vector<std::complex<double>> test1;
  std::vector<std::complex<double>> test2;
  test1.reserve(N);
  test2.reserve(N);
  for (size_t i = 0; i < N; i++) {
    double real = distribution(random_engine);
    double imag = distribution(random_engine);
    test1.emplace_back(real, imag);
    test2.emplace_back(real, imag);
  }
#if BENCHMARK == 1
  auto t1 = std::chrono::high_resolution_clock::now();
  Fourier<double>::fft(test1.begin(), test1.end());
  auto t2 = std::chrono::high_resolution_clock::now();
  std::chrono::duration<double, std::milli> fft_ms = t2 - t1;

  t1 = std::chrono::high_resolution_clock::now();
  Fourier<double>::dft(test2.begin(), 1, Fourier<double>::getRootsOfUnity(N),
                       N);
  t2 = std::chrono::high_resolution_clock::now();
  std::chrono::duration<double, std::milli> dft_ms = t2 - t1;

  std::cout << "FFT is " << dft_ms.count() / fft_ms.count()
            << " times faster for N = " << N << '\n';
#else
  Fourier<double>::fft(test1.begin(), test1.end());
  Fourier<double>::dft(test2.begin(), 1, Fourier<double>::getRootsOfUnity(N),
                       N);
#endif

  for (size_t i = 0; i < N; i++) {
    std::complex<double> error = test1[i] - test2[i];
    assert(std::abs(error.real()) / abs(test2[i]) < 0.0001);
    assert(std::abs(error.imag()) / abs(test2[i]) < 0.0001);
  }
}

TEST(TestFourier, testFFT) {
  for (size_t i = 2; i < 1000; i++) {
    testFFT(i);
  }
  testFFT(10000);
}

/**
 * We need to explicitly declare this function so that it can be declared a
 * friend in Fourier.h
 */
void testMultiplicationCount() {
  Fourier<double>::FactorTree* root = Fourier<double>::planFFT(100);
  unsigned int count1 = root->multiplicationCount();
  delete root;

  root = new Fourier<double>::FactorTree();
  root->value = 100;
  root->left = new Fourier<double>::FactorTree();
  root->left->value = 10;
  root->left->left = new Fourier<double>::FactorTree();
  root->left->left->value = 2;
  root->left->right = new Fourier<double>::FactorTree();
  root->left->right->value = 5;
  root->right = new Fourier<double>::FactorTree();
  root->right->value = 10;
  root->right->left = new Fourier<double>::FactorTree();
  root->right->left->value = 2;
  root->right->right = new Fourier<double>::FactorTree();
  root->right->right->value = 5;

  unsigned int count2 = root->multiplicationCount();
  delete root;

  ASSERT_EQ(count1, count2);
}

TEST(TestFourier, testMultiplicationCount) { testMultiplicationCount(); }

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
