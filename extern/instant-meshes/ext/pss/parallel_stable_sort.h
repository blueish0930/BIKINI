/*
 * Blender oneTBB compatibility shim for Instant Meshes.
 * Upstream parallel_stable_sort used the removed classic tbb::task API.
 * Fall back to std::stable_sort (same results; only used for ordering collapses / hierarchy edges).
 */
#pragma once

#include <algorithm>
#include <functional>
#include <iterator>

namespace pss {

template <typename RandomAccessIterator, typename Compare>
inline void parallel_stable_sort(RandomAccessIterator xs,
                                 RandomAccessIterator xe,
                                 Compare comp)
{
  std::stable_sort(xs, xe, comp);
}

template <typename RandomAccessIterator>
inline void parallel_stable_sort(RandomAccessIterator xs, RandomAccessIterator xe)
{
  typedef typename std::iterator_traits<RandomAccessIterator>::value_type T;
  std::stable_sort(xs, xe, std::less<T>());
}

}  // namespace pss
