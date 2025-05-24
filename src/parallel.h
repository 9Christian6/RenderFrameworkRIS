#ifndef PARALLEL_H
#define PARALLEL_H

#include <iterator>

#ifdef USE_STD_THREAD

#include <execution>
#include <algorithm>

class RangeIter {
public:
    using difference_type = size_t;
    using value_type = size_t;
    using pointer = const size_t*;
    using reference = const size_t&;
    using iterator_category = std::forward_iterator_tag;

    RangeIter(size_t pos = 0) : pos(pos) {}

    RangeIter& operator++() {
        ++pos;
        return *this;
    }

    RangeIter operator++(int) {
        auto old(*this);
        ++(*this);
        return old;
    }

    bool operator ==(const RangeIter& other) const {
        return pos == other.pos;
    }

    bool operator !=(const RangeIter& other) const {
        return !(*this == other);
    }

    size_t operator*() const { return pos; }

private:
    size_t pos;
};

template<typename Fn>
void parallel_for(size_t from, size_t to, Fn fn) {
    std::for_each(std::execution::par, RangeIter(0), RangeIter(to), [&](size_t i){ fn(i); });
}

#else

template<typename Fn>
void parallel_for(size_t from, size_t to, Fn fn) {
    #pragma omp parallel for
    for (size_t i = from; i < to; ++i) fn(i);
}

#endif

#endif // PARALLEL_H