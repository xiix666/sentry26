// 2021-09-18, https://github.com/spectral3d/hilbert_hpp is under MIT license.

//Copyright (c) 2019 David Beynon
//
//Permission is hereby granted, free of charge, to any person obtaining a copy
//of this software and associated documentation files (the "Software"), to deal
//in the Software without restriction, including without limitation the rights
//to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
//copies of the Software, and to permit persons to whom the Software is
//furnished to do so, subject to the following conditions:
//
//The above copyright notice and this permission notice shall be included in all
//copies or substantial portions of the Software.
//
//THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
//IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
//FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
//AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
//LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
//OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
//SOFTWARE.

#ifndef INCLUDED_HILBERT_HPP
#define INCLUDED_HILBERT_HPP
#pragma once

#include <algorithm>
#include <array>
#include <cstdint>
#include <limits>
#include <type_traits>

namespace hilbert
{

namespace v1
{
namespace internal
{

template <typename T, size_t N>
std::array<T, N> UntransposeBits(std::array<T, N> const & in)
{
  const size_t bits = std::numeric_limits<T>::digits;
  const T high_bit(T(1) << (bits - 1));
  const size_t bit_count(bits * N);

  std::array<T, N> out;

  std::fill(out.begin(), out.end(), 0);

  for (size_t b = 0; b < bit_count; b++) {
    size_t src_bit, dst_bit, src, dst;
    src = b % N;
    dst = b / bits;
    src_bit = b / N;
    dst_bit = b % bits;

    out[dst] |= (((in[src] << src_bit) & high_bit) >> dst_bit);
  }

  return out;
}

template <typename T, size_t N>
std::array<T, N> TransposeBits(std::array<T, N> const & in)
{
  const size_t bits = std::numeric_limits<T>::digits;
  const T high_bit(T(1) << (bits - 1));
  const size_t bit_count(bits * N);

  std::array<T, N> out;

  std::fill(out.begin(), out.end(), 0);

  for (size_t b = 0; b < bit_count; b++) {
    size_t src_bit, dst_bit, src, dst;
    src = b / bits;
    dst = b % N;
    src_bit = b % bits;
    dst_bit = b / N;

    out[dst] |= (((in[src] << src_bit) & high_bit) >> dst_bit);
  }

  return out;
}
}

template <typename T, size_t N>
std::array<T, N> IndexToPosition(std::array<T, N> const & in)
{

  std::array<T, N> out(internal::TransposeBits(in));

  {
    T tmp = out[N - 1] >> 1;

    for (size_t n = N - 1; n; n--) {
      out[n] ^= out[n - 1];
    }

    out[0] ^= tmp;
  }

  {
    T cur_bit(2), low_bits;

    while (cur_bit) {
      low_bits = cur_bit - 1;

      size_t n(N);

      do {
        n--;
        if (out[n] & cur_bit) {

          out[0] ^= low_bits;
        } else {

          T t((out[n] ^ out[0]) & low_bits);
          out[n] ^= t;
          out[0] ^= t;
        }
      } while (n);

      cur_bit <<= 1;
    }
  }

  return out;
}

template <typename T, size_t N>
std::array<T, N> PositionToIndex(std::array<T, N> const & in)
{
  const size_t bits = std::numeric_limits<T>::digits;

  std::array<T, N> out(in);

  {
    T cur_bit(T(1) << (bits - 1)), low_bits;

    do {
      low_bits = cur_bit - 1;

      for (size_t n = 0; n < N; n++) {
        if (out[n] & cur_bit) {

          out[0] ^= low_bits;
        } else {

          T t((out[n] ^ out[0]) & low_bits);
          out[n] ^= t;
          out[0] ^= t;
        }
      }

      cur_bit >>= 1;
    } while (low_bits > 1);
  }

  {
    T cur_bit(T(1) << (bits - 1)), t(0);

    for (size_t n = 1; n < N; n++) {
      out[n] ^= out[n - 1];
    }

    do {
      if (out[N - 1] & cur_bit) {
        t ^= (cur_bit - 1);
      }
      cur_bit >>= 1;
    } while (cur_bit > 1);

    for (auto & v : out) {
      v ^= t;
    }
  }

  return internal::UntransposeBits(out);
}
}

namespace v2
{
namespace internal
{

namespace tmp
{
template <typename T, size_t N, size_t D>
T TransposeBits2(
  std::array<T, N> const &, std::integral_constant<size_t, D>, std::integral_constant<size_t, 0>)
{
  return T(0);
}

template <typename T, size_t N, size_t D, size_t B>
T TransposeBits2(
  std::array<T, N> const & in, std::integral_constant<size_t, D>, std::integral_constant<size_t, B>)
{
  const size_t size = std::numeric_limits<T>::digits;
  const size_t src = ((D + ((size - B) * N)) / size);
  const size_t src_bit = size - (1 + ((D + ((size - B) * N)) % size));
  const T src_bit_val = T(1) << src_bit;
  const size_t dst_bit = (B - 1);
  const T dst_bit_val = T(1) << dst_bit;

  T bit = ((in[src] & src_bit_val) >> src_bit) * dst_bit_val;

  return bit + TransposeBits2(
                 in, std::integral_constant<size_t, D>(), std::integral_constant<size_t, B - 1>());
}

template <typename T, size_t N>
void TransposeBits(std::array<T, N> const &, std::array<T, N> &, std::integral_constant<size_t, 0>)
{
}

template <typename T, size_t N, size_t D>
void TransposeBits(
  std::array<T, N> const & in, std::array<T, N> & out, std::integral_constant<size_t, D>)
{
  out[D - 1] = TransposeBits2(
    in, std::integral_constant<size_t, D - 1>(),
    std::integral_constant<size_t, std::numeric_limits<T>::digits>());

  TransposeBits(in, out, std::integral_constant<size_t, D - 1>());
}

template <typename T, size_t N, size_t D>
T UntransposeBits2(
  std::array<T, N> const &, std::integral_constant<size_t, D>, std::integral_constant<size_t, 0>)
{
  return T(0);
}

template <typename T, size_t N, size_t D, size_t B>
T UntransposeBits2(
  std::array<T, N> const & in, std::integral_constant<size_t, D>, std::integral_constant<size_t, B>)
{
  const size_t size = std::numeric_limits<T>::digits;
  const size_t src = ((D * size) + (size - B)) % N;
  const size_t src_bit = size - (((((D * size) + (size - B))) / N) + 1);
  const T src_bit_val = T(1) << src_bit;
  const size_t dst_bit(B - 1);
  const T dst_bit_val = T(1) << dst_bit;

  T bit = ((in[src] & src_bit_val) >> src_bit) * dst_bit_val;

  return bit + UntransposeBits2(
                 in, std::integral_constant<size_t, D>(),
                 std::integral_constant<size_t, size_t(B - 1)>());
}

template <typename T, size_t N>
void UntransposeBits(
  std::array<T, N> const &, std::array<T, N> &, std::integral_constant<size_t, 0>)
{
}

template <typename T, size_t N, size_t D>
void UntransposeBits(
  std::array<T, N> const & in, std::array<T, N> & out, std::integral_constant<size_t, D>)
{
  out[D - 1] = UntransposeBits2(
    in, std::integral_constant<size_t, D - 1>(),
    std::integral_constant<size_t, std::numeric_limits<T>::digits>());

  UntransposeBits(in, out, std::integral_constant<size_t, D - 1>());
}

template <typename T, size_t N>
void ApplyGrayCode1(
  std::array<T, N> const & in, std::array<T, N> & out, std::integral_constant<size_t, 0>)
{
  out[0] ^= in[N - 1] >> 1;
}

template <typename T, size_t N, size_t I>
void ApplyGrayCode1(
  std::array<T, N> const & in, std::array<T, N> & out, std::integral_constant<size_t, I>)
{
  out[I] ^= out[I - 1];

  ApplyGrayCode1(in, out, std::integral_constant<size_t, I - 1>());
}

template <typename T, size_t N>
void RemoveGrayCode1(std::array<T, N> &, std::integral_constant<size_t, 0>)
{
}

template <typename T, size_t N, size_t D>
void RemoveGrayCode1(std::array<T, N> & in, std::integral_constant<size_t, D>)
{
  const size_t src_idx = N - (D + 1);
  const size_t dst_idx = N - D;

  in[dst_idx] ^= in[src_idx];

  RemoveGrayCode1(in, std::integral_constant<size_t, D - 1>());
}

template <typename T>
T RemoveGrayCode2(T, std::integral_constant<size_t, 1>)
{
  return T(0);
}

template <typename T, size_t B>
T RemoveGrayCode2(T v, std::integral_constant<size_t, B>)
{
  const T cur_bit(T(1) << (B - 1));
  const T low_bits(cur_bit - 1);

  if (v & cur_bit) {
    return low_bits ^ RemoveGrayCode2(v, std::integral_constant<size_t, B - 1>());
  } else {
    return RemoveGrayCode2(v, std::integral_constant<size_t, B - 1>());
  }
}

template <typename T, size_t N, size_t B>
void GrayToHilbert2(
  std::array<T, N> &, std::integral_constant<size_t, B>, std::integral_constant<size_t, 0>)
{
}

template <typename T, size_t N, size_t B, size_t I>
void GrayToHilbert2(
  std::array<T, N> & out, std::integral_constant<size_t, B>, std::integral_constant<size_t, I>)
{
  const size_t n(I - 1);
  const T cur_bit(T(1) << (std::numeric_limits<T>::digits - B));
  const T low_bits(cur_bit - 1);

  if (out[n] & cur_bit) {

    out[0] ^= low_bits;
  } else {

    T t((out[n] ^ out[0]) & low_bits);
    out[n] ^= t;
    out[0] ^= t;
  }

  GrayToHilbert2(out, std::integral_constant<size_t, B>(), std::integral_constant<size_t, I - 1>());
}

template <typename T, size_t N>
void GrayToHilbert(std::array<T, N> &, std::integral_constant<size_t, 0>)
{
}

template <typename T, size_t N, size_t B>
void GrayToHilbert(std::array<T, N> & out, std::integral_constant<size_t, B>)
{
  GrayToHilbert2(out, std::integral_constant<size_t, B>(), std::integral_constant<size_t, N>());

  GrayToHilbert(out, std::integral_constant<size_t, B - 1>());
}

template <typename T, size_t N, size_t B>
void HilbertToGray2(
  std::array<T, N> &, std::integral_constant<size_t, B>, std::integral_constant<size_t, 0>)
{
}

template <typename T, size_t N, size_t B, size_t I>
void HilbertToGray2(
  std::array<T, N> & out, std::integral_constant<size_t, B>, std::integral_constant<size_t, I>)
{
  const size_t cur_bit(T(1) << B);
  const size_t low_bits(cur_bit - 1);
  const size_t n(N - I);

  if (out[n] & cur_bit) {

    out[0] ^= low_bits;
  } else {

    T t((out[n] ^ out[0]) & low_bits);
    out[n] ^= t;
    out[0] ^= t;
  }

  HilbertToGray2(out, std::integral_constant<size_t, B>(), std::integral_constant<size_t, I - 1>());
}

template <typename T, size_t N>
void HilbertToGray(std::array<T, N> &, std::integral_constant<size_t, 0>)
{
}

template <typename T, size_t N, size_t B>
void HilbertToGray(std::array<T, N> & out, std::integral_constant<size_t, B>)
{
  HilbertToGray2(out, std::integral_constant<size_t, B>(), std::integral_constant<size_t, N>());

  HilbertToGray(out, std::integral_constant<size_t, B - 1>());
}

template <typename T, size_t N>
void ApplyMaskToArray(std::array<T, N> &, T, std::integral_constant<size_t, 0>)
{
}

template <typename T, size_t N, size_t I>
void ApplyMaskToArray(std::array<T, N> & a, T mask, std::integral_constant<size_t, I>)
{
  a[I - 1] ^= mask;

  ApplyMaskToArray(a, mask, std::integral_constant<size_t, I - 1>());
}
}

template <typename T, size_t N>
std::array<T, N> TransposeBits(std::array<T, N> const & in)
{
  std::array<T, N> out;

  std::fill(out.begin(), out.end(), 0);

  tmp::TransposeBits(in, out, std::integral_constant<size_t, N>());

  return out;
}

template <typename T, size_t N>
std::array<T, N> UntransposeBits(std::array<T, N> const & in)
{
  std::array<T, N> out;

  std::fill(out.begin(), out.end(), 0);

  tmp::UntransposeBits(in, out, std::integral_constant<size_t, N>());

  return out;
}

template <typename T, size_t N>
std::array<T, N> ApplyGrayCode(std::array<T, N> const & in)
{
  std::array<T, N> out(in);

  tmp::ApplyGrayCode1(in, out, std::integral_constant<size_t, N - 1>());

  return out;
}

template <typename T, size_t N>
std::array<T, N> RemoveGrayCode(std::array<T, N> const & in)
{
  const size_t bits = std::numeric_limits<T>::digits;
  std::array<T, N> out(in);

  {

    tmp::RemoveGrayCode1(out, std::integral_constant<size_t, N - 1>());

    T t = tmp::RemoveGrayCode2(out[N - 1], std::integral_constant<size_t, bits>());

    tmp::ApplyMaskToArray(out, t, std::integral_constant<size_t, N>());
  }

  return out;
}

template <typename T, size_t N>
std::array<T, N> GrayToHilbert(std::array<T, N> const & in)
{
  std::array<T, N> out(in);

  tmp::GrayToHilbert(out, std::integral_constant<size_t, std::numeric_limits<T>::digits - 1>());

  return out;
}

template <typename T, size_t N>
std::array<T, N> HilbertToGray(std::array<T, N> const & in)
{
  std::array<T, N> out(in);

  tmp::HilbertToGray(out, std::integral_constant<size_t, std::numeric_limits<T>::digits - 1>());

  return out;
}
}

template <typename T, size_t N>
std::array<T, N> IndexToPosition(std::array<T, N> const & in)
{

  return internal::GrayToHilbert(internal::ApplyGrayCode(internal::TransposeBits(in)));
}

template <typename T, size_t N>
std::array<T, N> PositionToIndex(std::array<T, N> const & in)
{
  return internal::UntransposeBits(internal::RemoveGrayCode(internal::HilbertToGray(in)));
}
}
}

#endif
