#ifndef HYDROLIB_STREAM_TRAITS_H_
#define HYDROLIB_STREAM_TRAITS_H_

#include <type_traits>
#include <utility>  // std::declval, std::void_t

namespace hydrolib::concepts::stream {

template <typename T, typename = void>
struct has_write : std::false_type {};

template <typename T>
struct has_write<T, std::void_t<decltype(
    write(std::declval<T&>(), std::declval<const void*>(), std::declval<unsigned>())
)>> : std::is_convertible<
    decltype(write(std::declval<T&>(), nullptr, 0u)),
    int
> {};


template <typename T, typename = void>
struct has_read : std::false_type {};

template <typename T>
struct has_read<T, std::void_t<decltype(
    read(std::declval<T&>(), std::declval<void*>(), std::declval<unsigned>())
)>> : std::is_convertible<
    decltype(read(std::declval<T&>(), nullptr, 0u)),
    int
> {};


template <typename T>
inline constexpr bool is_byte_writable_stream_v = has_write<T>::value;

template <typename T>
inline constexpr bool is_byte_readable_stream_v = has_read<T>::value;

template <typename T>
inline constexpr bool is_byte_full_stream_v =
    is_byte_writable_stream_v<T> && is_byte_readable_stream_v<T>;

} // namespace hydrolib::concepts::stream

#endif