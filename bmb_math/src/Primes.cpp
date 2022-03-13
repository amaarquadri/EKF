#include "bmb_math/Primes.h"
#include <cmath>
#include <cstddef>
#include <stdexcept>
#include <vector>

std::vector<size_t> primesUpTo(const size_t& n) {
  // returns all primes <= n using the sieve of Eratosthenes
  if (n < 2) {
    return {};
  }

  std::vector<bool> is_prime(n - 1,
                             true);  // index i corresponds to the number i + 2
  std::vector<size_t> primes;

  // use the prime number theorem to guess the number of primes
  // this will reduce the number of times the vector needs to be resized
  primes.reserve((size_t)(n / std::log(n)));

  // TODO: use the segmented sieve of Eratosthenes to reduce memory usage for
  //  large n
  for (size_t i = 2; i <= n; i++) {
    if (is_prime[i - 2]) {
      primes.push_back(i);
      for (size_t j = 2 * i; j <= n; j += i) is_prime[j - 2] = false;
    }
  }
  primes.shrink_to_fit();
  return primes;
}

std::vector<size_t> primeFactorization(size_t n) {
  std::vector<size_t> factors;
  for (size_t candidate : primesUpTo((size_t)std::sqrt(n))) {
    while (n % candidate == 0) {
      factors.push_back(candidate);
      n /= candidate;
    }
  }
  if (n != 1) {
    factors.push_back(n);
  }
  return factors;
}

std::vector<size_t> uniquePrimeFactors(size_t n) {
  std::vector<size_t> factors;
  for (size_t candidate : primesUpTo((size_t)std::sqrt(n))) {
    if (n % candidate == 0) {
      factors.push_back(candidate);
      n /= candidate;
    }
    while (n % candidate == 0) {
      n /= candidate;
    }
  }
  if (n != 1) {
    factors.push_back(n);
  }
  return factors;
}

size_t lowestPrimitiveRootOfPrime(const size_t& p) {
  // p must be prime
  // Based on:
  // https://math.stackexchange.com/questions/124408/finding-a-primitive-root-of-a-prime-number
  if (p == 2) return 1;
  std::vector<size_t> test_cases = uniquePrimeFactors(p - 1);
  for (size_t& test_case : test_cases) test_case = p / test_case;
  for (size_t n = 2; n < p; n++) {
    bool is_primitive_root = true;
    for (const size_t& test_case : test_cases) {
      if (powMod(n, test_case, p) == 1) {
        is_primitive_root = false;
        break;
      }
    }
    if (is_primitive_root) return n;
  }
  throw std::runtime_error(
      "No primitive root found. This function only supports prime arguments!");
}

size_t powMod(size_t base, size_t exponent, const size_t& m) {
  // computes (base ** exponent) mod m
  size_t result = 1;
  while (exponent > 0) {
    if (exponent & 1) result = (result * base) % m;
    base = (base * base) % m;
    exponent /= 2;
  }
  return result;
}
