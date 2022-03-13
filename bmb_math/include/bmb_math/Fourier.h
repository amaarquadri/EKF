#pragma once

#include <bmb_math/Primes.h>
#include <complex>
#include <vector>

template <typename T>
class Fourier {
 public:
  using iterator = typename std::vector<std::complex<T>>::iterator;

  /**
   * @brief Computes the discrete fourier transform
   *
   * @param begin Iterator to the beginning of the data
   * @param end Iterator to the end of the data
   */
  static void fft(iterator begin, iterator end) {
    // Based on:
    // https://numericalrecipes.wordpress.com/2009/05/29/the-cooley-tukey-fft-algorithm-for-general-factorizations/
    const size_t N = end - begin;
    if (N <= 1) return;
    const FactorTree* const factor_tree = planFFT(N);
    const std::vector<std::complex<T>> roots_of_unity = getRootsOfUnity(N);
    cooleyTukeyFFT(begin, end, 1, factor_tree, roots_of_unity);
    delete factor_tree;
  }

  /**
   * @brief Computes the inverse discrete fourier transform
   *
   * @param begin Iterator to the beginning of the data
   * @param end Iterator to the end of the data
   */
  static void inverseFFT(iterator begin, iterator end) {
    for (iterator it = begin; it < end; it++) {
      (*it) = std::conj(*it);
    }

    fft(begin, end);

    const size_t N = (end - begin);
    for (iterator it = begin; it < end; it++) {
      (*it) = std::conj(*it) / N;
    }
  }

 private:
  struct FactorTree {
    size_t value;
    FactorTree* left = nullptr;
    FactorTree* right = nullptr;

    /**
     * @return Whether or not this FactorTree is a leaf node
     */
    [[nodiscard]] bool isLeaf() const { return left == nullptr; }

    ~FactorTree() {
      delete left;
      delete right;
    }

    [[nodiscard]] unsigned int multiplicationCount() const {
      if (isLeaf()) return value * value;
      return right->value * left->multiplicationCount() +
             left->value * right->multiplicationCount() + value;
    }
  };

  /**
   * @brief Computes a FactorTree for N
   * Each non-leaf node in the tree corresponds to a recursive call of the
   * Cooley-Tukey algorithm
   *
   * @param N The number to compute the FactorTree for. Must be larger than 1
   * @return A FactorTree for N
   */
  static FactorTree* planFFT(const size_t& N) {
    const std::vector<size_t> factors = primeFactorization(N);

    auto* tree = new FactorTree();
    tree->value = factors[0];

    for (size_t i = 1; i < factors.size(); i++) {
      FactorTree* right = new FactorTree();
      right->value = factors[i];
      FactorTree* left = tree;

      tree = new FactorTree();
      tree->value = left->value * right->value;
      tree->left = left;
      tree->right = right;
    }
    return tree;
  }

  /**
   * @brief Performs an in-place discrete fourier transform using the
   * Cooley-Tukey algorithm
   *
   * @param begin An iterator to the beginning of the array
   * @param end An iterator to the end of the array
   * @param incr The spacing between elements of the array
   * @param factor_tree A FactorTree for the number of elements in the array
   * @param roots_of_unity The Nth complex roots of unity, where N is the number
   * of elements in the array times incr
   */
  static void cooleyTukeyFFT(
      iterator begin, iterator end, const size_t& incr,
      const FactorTree* const factor_tree,
      const std::vector<std::complex<T>>& roots_of_unity) {
    const size_t& N = factor_tree->value;

    if (factor_tree->isLeaf()) {
      // prime number of elements
      dft(begin, incr, roots_of_unity, N);
      return;
    }
    const size_t& P = factor_tree->left->value;
    const size_t& Q = factor_tree->right->value;

    for (size_t i = 0; i < Q; i++) {
      cooleyTukeyFFT(begin + i * incr, end, incr * Q, factor_tree->left,
                     roots_of_unity);
    }

    // twiddle factors
    for (size_t p = 0; p < P; p++)
      for (size_t q = 0; q < Q; q++) {
        begin[(Q * p + q) * incr] *= roots_of_unity[p * q * incr];
      }

    for (size_t i = 0; i < P; i++) {
      cooleyTukeyFFT(begin + i * Q * incr, end, incr, factor_tree->right,
                     roots_of_unity);
    }

    transpose(begin, incr, P, Q);
  }

  /**
   * @brief Performs an in-place discrete fourier transform
   * This uses the naive O(n^2) algorithm, so should only be used as a base case
   * when N is prime
   *
   * @param begin An iterator to the start of array
   * @param incr The spacing between elements of the array
   * @param roots_of_unity The complex roots of unity
   * @param N The number of points in the array
   */
  static void dft(iterator begin, const size_t& incr,
                  const std::vector<std::complex<T>>& roots_of_unity,
                  const size_t& N) {
    // should only be used for prime sized data
    const size_t ratio = roots_of_unity.size() / N;
    std::vector<std::complex<T>> result(N);
    for (size_t i = 0; i < N; i++) {
      for (size_t j = 0; j < N; j++) {
        result[i] += begin[j * incr] * roots_of_unity[((i * j) % N) * ratio];
      }
    }

    for (size_t i = 0; i < N; i++) {
      begin[i * incr] = result[i];
    }
  }

  /**
   * @brief Computes the complex roots of unity
   * The kth element of the output is e^[i*(-2*M_PI*k/N)], where i is the
   * imaginary unit
   *
   * @param N The number of roots to calculate
   * @return The Nth complex roots of unity
   */
  static std::vector<std::complex<T>> getRootsOfUnity(const size_t& N) {
    // the kth element of the output is e^[i*(-2*pi*k/N)]
    // compute roots of unity via repeated multiplication
    // this avoids repetitive use of std::polar which is slower since it
    // requires sin and cos
    std::vector<std::complex<T>> roots_of_unity(N);
    roots_of_unity[0] = std::complex<T>(static_cast<T>(1), static_cast<T>(0));
    roots_of_unity[1] =
        std::polar(static_cast<T>(1), static_cast<T>(-2 * M_PI / N));
    for (size_t i = 2; i < N; i++) {
      roots_of_unity[i] = roots_of_unity[1] * roots_of_unity[i - 1];
    }
    return roots_of_unity;
  }

  /**
   * @brief Performs an in-place matrix transposition
   * The matrix is assumed to be in row major order
   *
   * @param begin An Iterator to the start of the matrix
   * @param incr The spacing between elements of the matrix
   * @param P The number of rows
   * @param Q The number of columns
   */
  static void transpose(iterator begin, const size_t& incr, const size_t& P,
                        const size_t& Q) {
    // Based on:
    // https://en.wikipedia.org/wiki/In-place_matrix_transposition#Properties_of_the_permutation
    const size_t N = P * Q;
    // first and last points of the matrix never need to move, so we only need N
    // - 2 bits of auxiliary storage thus, whether the ith element of the matrix
    // has been moved is stored in the (i - 1)th index of moved
    std::vector<bool> moved(N - 2);
    for (size_t i = 1; i < N - 1; i++) {
      if (!moved[i - 1]) {
        std::complex<T> last_value = begin[i * incr];
        moved[i - 1] = true;
        size_t next_index = (P * i) % (N - 1);
        while (next_index != i) {
          moved[next_index - 1] = true;
          std::swap(last_value, begin[next_index * incr]);
          next_index = (P * next_index) % (N - 1);
        }
        begin[i * incr] = last_value;
      }
    }
  }

  friend void testFFT(const size_t& N);
  friend void testMultiplicationCount();
};
